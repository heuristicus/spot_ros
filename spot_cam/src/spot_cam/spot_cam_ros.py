import logging
import threading
import typing
import wave
import functools
import actionlib

import numpy as np
import rospy
import tf
import tf2_ros
from bosdyn.client.exceptions import (
    InvalidRequestError,
    InternalServerError,
    RetryableUnavailableError,
)
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import Image
from spot_cam.msg import (
    BITStatus,
    Degradation,
    PowerStatus,
    PTZDescription,
    PTZDescriptionArray,
    PTZLimits,
    PTZState,
    PTZStateArray,
    StreamParams,
    StringMultiArray,
    Temperature,
    TemperatureArray,
    LookAtPointAction,
    LookAtPointGoal,
    LookAtPointResult,
)
from spot_cam.srv import (
    LoadSound,
    LookAtPoint,
    PlaySound,
    SetBool,
    SetFloat,
    SetIRColormap,
    SetIRMeterOverlay,
    SetPTZState,
    SetStreamParams,
    SetString,
)
from spot_wrapper.cam_wrapper import SpotCamWrapper
from std_msgs.msg import Float32MultiArray, Float32
from std_srvs.srv import Trigger

from spot_driver.ros_helpers import GetSystemFaults


class ROSHandler:
    def __init__(self):
        self.logger = logging.getLogger("rosout")


class LightingHandlerROS(ROSHandler):
    """
    Handle ROS interactions for LEDs
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.lighting_client = wrapper.lighting
        self.led_publisher = rospy.Publisher(
            "spot/cam/status/leds", Float32MultiArray, latch=True, queue_size=1
        )
        self.led_subscriber = rospy.Subscriber(
            "spot/cam/set_leds", Float32, callback=self.leds_callback
        )
        # Allow time for publisher to initialise before publishing led state
        self._publish_leds()
        self.logger.info("Initialised lighting handler")

    def leds_callback(self, msg: Float32):
        """
        Args:
            msg: Float32 message with the brightness the LEDs should be set to
        """

        self.set_leds(msg.data)

    def set_leds(self, brightness: float):
        """
        Set all LEDs on the cam to the specified brightness

        Also publishes an update to the topic

        Args:
            brightness: Brightness to which LEDs should be set. This will be clamped to valid values.
        """
        self.lighting_client.set_led_brightness(brightness)
        self._publish_leds()

    def _publish_leds(self):
        """
        Publish the current state of the LEDs to the topic
        """
        self.led_publisher.publish(
            Float32MultiArray(data=self.lighting_client.get_led_brightness())
        )


class PowerHandlerROS(ROSHandler):
    """
    Handles interaction with power controls
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.power_client = wrapper.power
        self.power_publisher = rospy.Publisher(
            "spot/cam/status/power", PowerStatus, queue_size=1, latch=True
        )
        self.power_subscriber = rospy.Subscriber(
            "spot/cam/set_power", PowerStatus, self.power_callback
        )
        self.power_subscriber = rospy.Subscriber(
            "spot/cam/cycle_power", PowerStatus, self.cycle_callback
        )
        self._publish_power()
        self.logger.info("Initialised power handler")

    def power_callback(self, msg: PowerStatus):
        """
        Set the power status for each device

        Args:
            msg: PowerStatus message with the state of each device set
        """
        self.set_power(msg.ptz, msg.aux1, msg.aux2, msg.external_mic)

    def cycle_callback(self, msg: PowerStatus):
        """
        Cycle the power status for the requested devices

        Args:
            msg: PowerStatus message with the state of each device set
        """
        self.set_power(msg.ptz, msg.aux1, msg.aux2, msg.external_mic, cycle=True)

    def _convert_state(self, state) -> typing.Optional[bool]:
        """
        Convert a power state from the ROS message to bool or none

        Args:
            state: Power state to convert

        Returns:
            True if the state is POWER_ON, false if the state is POWER_OFF, None if the state is NO_ACTION
        """
        if state == PowerStatus.NO_ACTION:
            return None
        elif state == PowerStatus.POWER_ON:
            return True
        elif state == PowerStatus.POWER_OFF:
            return False
        else:
            self.logger.warning(
                f"Received unknown power state {state}. No action will be taken."
            )
            return None

    def set_power(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
        cycle=False,
    ):
        """
        Set or cycle the power state for the devices
        Args:
            ptz:
            aux1:
            aux2:
            external_mic:
            cycle: If true, cycle power for devices with POWER_ON
        """

        power_states = map(self._convert_state, [ptz, aux1, aux2, external_mic])
        self.logger.info(list(power_states))
        if cycle:
            self.power_client.cycle_power(*power_states)
        else:
            self.power_client.set_power_status(*power_states)
        self._publish_power()

    def _publish_power(self):
        status = self.power_client.get_power_status()
        power_status = PowerStatus()
        (
            power_status.ptz,
            power_status.aux1,
            power_status.aux2,
            power_status.external_mic,
        ) = map(
            lambda x: PowerStatus.POWER_ON if x else PowerStatus.POWER_OFF,
            [status.ptz, status.aux1, status.aux2, status.external_mic],
        )

        self.power_publisher.publish(power_status)


class CompositorHandlerROS(ROSHandler):
    """
    Handles interaction with the compositor
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.compositor
        self.set_screen_service = rospy.Service(
            "/spot/cam/set_screen", SetString, self.handle_set_screen
        )
        self.set_ir_meter_overlay_service = rospy.Service(
            "/spot/cam/set_ir_meter_overlay",
            SetIRMeterOverlay,
            self.handle_set_ir_meter_overlay,
        )
        self.screen_list_pub = rospy.Publisher(
            "/spot/cam/screens", StringMultiArray, queue_size=1, latch=True
        )
        self.screen_list_pub.publish(self.client.list_screens())
        self.set_ir_colormap_service = rospy.Service(
            "/spot/cam/set_ir_colormap", SetIRColormap, self.handle_set_ir_colormap
        )
        self.logger.info("Initialised compositor handler")

    def handle_set_screen(self, req: SetString):
        """
        Handle a request to set the screen displayed by webrtc
        """
        try:
            self.set_screen(req.value)
            return True, f"Successfully set screen to {req.value}"
        except InvalidRequestError as e:
            message = f"{e}.\nValid screens are {self.client.list_screens()}"
            self.logger.error(message)
            return False, message

    def set_screen(self, screen):
        """
        Choose which screen to display. This is how it is possible to view the streams from different cameras

        Args:
            screen: Screen to display
        """
        self.client.set_screen(screen)

    def handle_set_ir_meter_overlay(self, req: SetIRMeterOverlay):
        """
        Handle a request to set the IR overlay
        """
        if req.x < 0 or req.x > 1 or req.y < 0 or req.y > 1:
            return (
                False,
                "requested x or y position out of bounds, valid values are in the range [0,1]",
            )
        self.set_ir_meter_overlay(req.x, req.y, req.enable)
        return True, "Successfully set IR overlay"

    def set_ir_meter_overlay(self, x, y, enable=True):
        """
        Enable or disable and set the reticle position in the IR overlay

        Args:
            x: Horizontal coordinate between 0 and 1
            y: Vertical coordinate between 0 and 1
            enable: If true, show the overlay
        """
        self.client.set_ir_meter_overlay(x, y, enable)

    def handle_set_ir_colormap(self, req: SetIRColormap):
        """
        Handle a request to set the colormap for the IR images
        """
        self.set_ir_colormap(req.colormap, req.min, req.max, req.auto_scale)
        return True, "Successfully set IR colormap"

    def set_ir_colormap(self, colormap, min, max, auto_scale=True):
        """
        Set the IR colormap for images

        Args:
            colormap: Colormap to use
            min: Minimum temperature on the scale
            max: Maximum temperature on the scale
            auto_scale: If true, scale based on the values in the image and ignore min and max
        """
        self.client.set_ir_colormap(colormap, min, max, auto_scale)


class HealthHandlerROS(ROSHandler):
    """
    Handles interaction with health information from the device
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.health
        self.robot = wrapper.robot
        self.temp_publisher = rospy.Publisher(
            "/spot/cam/status/temperatures", TemperatureArray, queue_size=1
        )
        self.status_publisher = rospy.Publisher(
            "/spot/cam/status/built_in_test", BITStatus, queue_size=1
        )

        self.temp_thread = threading.Thread(target=self._publish_temperatures_loop)
        self.temp_thread.start()

        self.status_thread = threading.Thread(target=self._publish_status_loop)
        self.status_thread.start()
        self.logger.info("Initialised health handler")

    def get_status(self):
        """
        Get the fault and degradation status of the device
        """
        events, degradations = self.client.get_bit_status()
        status = BITStatus()
        status.events = GetSystemFaults(events, self.robot)

        for degradation in degradations:
            status.degradations.append(
                Degradation(type=degradation[0], description=degradation[1])
            )

        return status

    def publish_status(self):
        """
        Publish the current status of the device
        """
        try:
            self.status_publisher.publish(self.get_status())
        except RetryableUnavailableError as e:
            self.logger.error(f"Error while getting BIT status: {e}")

    def _publish_status_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                self.publish_status()
            except RetryableUnavailableError as e:
                self.logger.warning(f"Failed to get health status: {e}")
            rate.sleep()

    def get_temperatures(self):
        """
        Get the temperatures for various components of the device
        """
        temps = self.client.get_temperature()
        temp_array = TemperatureArray()
        for temp in temps:
            temp_msg = Temperature()
            temp_msg.channel_name = temp[0]
            temp_msg.temperature = temp[1]
            temp_array.temperatures.append(temp_msg)

        return temp_array

    def publish_temperatures(self):
        """
        Publish the current temperatures of the device
        """
        self.temp_publisher.publish(self.get_temperatures())

    def _publish_temperatures_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                self.publish_temperatures()
            except RetryableUnavailableError as e:
                self.logger.warning(f"Failed to get temperatures: {e}")

            rate.sleep()


class AudioHandlerROS(ROSHandler):
    """
    Handles audio interaction with the device
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.audio
        self.set_volume_service = rospy.Service(
            "/spot/cam/audio/set_volume", SetFloat, self.handle_set_volume
        )
        self.play_sound_service = rospy.Service(
            "/spot/cam/audio/play", PlaySound, self.handle_play_sound
        )
        self.load_sound_service = rospy.Service(
            "/spot/cam/audio/load", LoadSound, self.handle_load_sound
        )
        self.delete_sound_service = rospy.Service(
            "/spot/cam/audio/delete", SetString, self.handle_delete_sound
        )
        self.logger.info("Initialised audio handler")

    def handle_set_volume(self, req):
        """
        Handle a request to set audio volume on the device
        """
        self.set_volume(req.value)
        return True, "Successfully set volume"

    def set_volume(self, percentage):
        """
        Set the audio volume on the device

        Args:
            percentage: Volume as a percentage
        """
        self.client.set_volume(percentage)

    def handle_play_sound(self, req):
        """
        Handle a service call to play a sound
        """
        try:
            self.play_sound(req.name, req.gain)
        except InvalidRequestError as e:
            message = f"{e}.\nValid sounds are {self.client.list_screens()}"
            self.logger.error(message)
            return False, message

        return True, f"Played sound {req.name}"

    def play_sound(self, sound_name, gain=1):
        """
        Play a sound from the device

        Args:
            sound_name: Name of the sound to play
            gain: Multiplicative gain on the sound volume
        Raises:
            InvalidRequestError: If a sound with the given name does not exist on the device
        """
        self.client.play_sound(sound_name, gain)

    def handle_load_sound(self, req):
        """
        Handle a service call to load a sound
        """
        try:
            self.load_sound(req.wav_path, req.name)
        except (IOError, wave.Error) as e:
            message = f"Failed to load sound: {e}"
            self.logger.error(message)
            return False, str(message)

        return True, "Successfully loaded sound"

    def load_sound(self, sound_file, name):
        """
        Load a sound from the file system onto the device
        Args:
            sound_file: File which should be added to the device sounds
            name: Name under which the sound should be added
        Raises:
            wave.Error: If the file was not a wav file
            IOError: If the provided string was not a file
        """
        self.client.load_sound(sound_file, name)
        self.logger.info(f"Loaded new sound {name}")

    def handle_delete_sound(self, req):
        """
        Handle a service call to delete a sound
        """
        try:
            self.client.delete_sound(req.value)
        except InternalServerError as e:
            return False, f"Failed to delete sound {req.value}, perhaps it didn't exist"

        return True, f"Successfully deleted sound {req.value}"

    def delete_sound(self, sound_name):
        """
        Delete a sound from the device

        Args:
            sound_name: Name of the sound to delete
        Raises:
            InternalServerError: If the sound does not exist
        """
        self.client.delete_sound(sound_name)
        self.logger.info(f"Deleted sound {sound_name}")


class StreamQualityHandlerROS(ROSHandler):
    """
    Handles interaction with stream quality controls
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.stream_quality
        self.params_service = rospy.Service(
            "/spot/cam/stream/set_params", SetStreamParams, self.handle_params
        )
        self.congestion_control_service = rospy.Service(
            "/spot/cam/stream/enable_congestion_control",
            SetBool,
            self.handle_congestion_control,
        )
        self.params_pub = rospy.Publisher(
            "/spot/cam/stream/params", StreamParams, queue_size=1, latch=True
        )

        # Allow time for publisher initialisation
        self.publish_stream_params()
        self.logger.info("Initialised stream quality handler")

    def handle_params(self, req):
        """
        Handle a request to set the stream quality parameters
        """
        try:
            self.set_stream_params(
                req.params.target_bitrate,
                req.params.refresh_interval,
                req.params.idr_interval,
                req.params.awb,
            )
        except InvalidRequestError as e:
            message = (
                f"Bad request while setting params {e}. This might be because you tried to turn auto white "
                f"balance off. "
            )
            self.logger.error(message)
            return False, message
        return True, "Set stream parameters"

    def set_stream_params(self, target_bitrate, refresh_interval, idr_interval, awb):
        """
        Set the stream quality parameters

        Note: It is currently not possible to turn off the auto white balance. You will get a crash

        Args:
            target_bitrate: Compression target in bits per second
            refresh_interval: How often to refresh the whole feed (in frames)
            idr_interval: How often to send an IDR message (in frames)
            awb: Mode for automatic white balance
        """
        # Convert the auto white balance to the bosdyn version. Had to do this to ensure we could distinguish between
        # turning the awb off vs leaving it as it was
        if awb == -1:
            awb = 0
        elif awb == 0:
            awb = None
        params = [
            target_bitrate if target_bitrate != 0 else None,
            refresh_interval if refresh_interval != 0 else None,
            idr_interval if idr_interval != 0 else None,
            awb,
        ]

        if not any(params[:3]) and awb is None:
            rospy.logwarn(
                "Received set params request where everything was empty. Not doing anything."
            )
            return

        self.client.set_stream_params(*params)
        self.publish_stream_params()

    def get_stream_params(self):
        """
        Get the current parameters for the stream quality
        """
        return self.client.get_stream_params()

    def publish_stream_params(self):
        """
        Publish the current stream quality parameters
        """
        params = self.get_stream_params()
        params_msg = StreamParams()
        params_msg.target_bitrate = params["target_bitrate"]
        params_msg.refresh_interval = params["refresh_interval"]
        params_msg.idr_interval = params["idr_interval"]
        params_msg.awb = params["awb"]
        # We had to mess with the ROS message to be able to distinguish between an unset variable and a request to
        # turn off, so must do conversion here
        if params_msg.awb == 0:
            params_msg.awb = StreamParams.OFF

        self.params_pub.publish(params_msg)

    def handle_congestion_control(self, req):
        """
        Handle a request to enable or disable congestion control
        """
        self.enable_congestion_control(req.value)

        enabled_str = "enabled" if req.value else "disabled"
        message = f"Stream congestion control is {enabled_str}"
        self.logger.info(message)
        return True, message

    def enable_congestion_control(self, enable):
        """
        Enable or disable the congestion control

        Args:
            enable: Enable if true
        """
        self.client.enable_congestion_control(enable)


class TransformHandlerROS(ROSHandler):
    """
    Handles publication of transforms for the cameras
    """

    STATIC_CAMERAS = [
        "c0",
        "c1",
        "c2",
        "c3",
        "c4",
        "pano",
        "tiled-ring",
        "projected-ring",
    ]
    # Unclear where this is defined. Documentation (
    # https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#camera) says it can be retrieved from a
    # frametreesnapshot provided by the payload registration service, but there doesn't seem to be a function there
    # which returns one of those.
    PAYLOAD_FRAME = "spot_cam_payload_frame"

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.media_log
        self.payload_info = wrapper.payload_details

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.static_transforms = []
        # Generate a static transform for the spot cam payload itself relative to the body
        self.payload_transform = self._se3_proto_to_tf(
            self.payload_info.body_tform_payload
        )
        self.payload_transform.header.frame_id = "body"
        self.payload_transform.child_frame_id = self.PAYLOAD_FRAME
        self.static_transforms.append(self.payload_transform)

        # Generate static transforms for the fixed cameras
        self.static_transforms.extend(self._get_camera_transforms(self.STATIC_CAMERAS))
        self.static_transforms.append(self._get_ptz_zero_transform())

        # Publish all the static transforms in a block. Doing each one individually can cause issues.
        self.tf_static_broadcaster.sendTransform(self.static_transforms)

        self.transform_thread = threading.Thread(target=self.publish_transforms)
        self.transform_thread.start()

    def _se3_proto_to_tf(self, se3, set_stamp=True) -> TransformStamped:
        """
        Convert an se3 proto to a transform stamped

        Args:
            se3: se3 proto
            set_stamp: If true, set the timestamp to the time of generation

        Returns:
            TransformStamped
        """
        tfm = TransformStamped()
        if set_stamp:
            tfm.header.stamp = rospy.Time.now()
        tfm.transform.translation.x = se3.position.x
        tfm.transform.translation.y = se3.position.y
        tfm.transform.translation.z = se3.position.z
        tfm.transform.rotation.w = se3.rotation.w
        tfm.transform.rotation.x = se3.rotation.x
        tfm.transform.rotation.y = se3.rotation.y
        tfm.transform.rotation.z = se3.rotation.z

        return tfm

    def publish_transforms(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.tf_broadcaster.sendTransform(
                    self._get_camera_transforms(self.STATIC_CAMERAS, exclude=True)
                )
            except RetryableUnavailableError as e:
                self.logger.warning(f"Failed to list cameras: {e}")
            except rospy.ROSException as e:
                self.logger.error(
                    f"ROS Error when trying to publish cam transforms: {e}. Will try to reset broadcaster."
                )
                self.tf_broadcaster = tf2_ros.TransformBroadcaster()

            rate.sleep()

    def _get_ptz_zero_transform(self) -> TransformStamped:
        """
         To correctly point the PTZ, it is useful to have a ptz zero frame which can be used to compute required pan
         and tilt to point it at a point. This only applies to the Spot CAM or CAM+IR. This is taken by using the
         set_position service to set the ptz to zero, then using rostopic echo tf tf_echo spot_cam_payload_frame ptz

        rosrun tf tf_echo spot_cam_payload_frame ptz
        At time 1680528877.844
        - Translation: [0.000, -0.154, 0.000]
        - Rotation: in Quaternion [-0.000, 0.000, -0.000, 1.000]
                    in RPY (radian) [-0.000, 0.000, -0.000]
                    in RPY (degree) [-0.000, 0.022, -0.000]


         Returns:
             TransformStamped containing the zero position for the ptz
        """
        ptz_zero_in_body = TransformStamped()
        ptz_zero_in_body.header.stamp = rospy.Time.now()
        ptz_zero_in_body.header.frame_id = "spot_cam_payload_frame"
        ptz_zero_in_body.child_frame_id = "ptz_zero"
        # Only translation in the y axis from the spot cam payload frame. Everything else is zeroed
        ptz_zero_in_body.transform.translation.y = -0.154
        ptz_zero_in_body.transform.rotation.w = 1

        return ptz_zero_in_body

    def _get_camera_transforms(
        self, camera_names: typing.Optional[typing.List[str]], exclude=False
    ) -> typing.List[TransformStamped]:
        """
        Get a list of camera transforms

        Args:
            camera_names: List of names of cameras for which transforms should be returned
            exclude: If true, return transforms for those cameras which are not present in the provided list of names

        Returns:
            List of TransformStamped
        """
        transforms = []
        for camera in self.client.list_cameras():
            if (not exclude and camera.name in camera_names) or (
                exclude and camera.name not in camera_names
            ):
                camera_transform = self._se3_proto_to_tf(camera.base_tform_sensor)
                camera_transform.child_frame_id = camera.name
                # This is the transform from the payload frame to the camera frame
                camera_transform.header.frame_id = self.PAYLOAD_FRAME
                transforms.append(camera_transform)

        return transforms


class PTZHandlerROS(ROSHandler):
    """
    Handles interaction with a ptz
    """

    # See Boston Dyanmics specifications https://support.bostondynamics.com/s/article/Spot-CAM-Spot-CAM-Spot-CAM-IR#Specifications
    SHORTEST_FOCAL_LENGTH = 4.3e-3  # focal lengths between 4.3mm to 129.0mm
    DIAGONAL_SENSOR_SIZE = 6.46e-3  # 1/2.8" sensor with 6.46mm diagonal

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.ptz

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Timer used when the cam is tracking a point
        self._track_timer = None

        self.ptz_list_publisher = rospy.Publisher(
            "/spot/cam/ptz/list", PTZDescriptionArray, queue_size=1, latch=True
        )
        self.ptz_velocity_publisher = rospy.Publisher(
            "/spot/cam/ptz/velocities", PTZStateArray, queue_size=1
        )
        self.ptz_position_publisher = rospy.Publisher(
            "/spot/cam/ptz/positions", PTZStateArray, queue_size=1
        )
        self.ptz_position_service = rospy.Service(
            "/spot/cam/ptz/set_position", SetPTZState, self.handle_set_ptz_position
        )
        self.ptz_velocity_service = rospy.Service(
            "/spot/cam/ptz/set_velocity", SetPTZState, self.handle_set_ptz_velocity
        )
        self.autofocus_service = rospy.Service(
            "/spot/cam/ptz/reset_autofocus", Trigger, self.handle_reset_autofocus
        )
        self.look_at_point_service = rospy.Service(
            "/spot/cam/ptz/look_at_point", LookAtPoint, self.handle_look_at_point
        )

        self.look_at_point_as = actionlib.SimpleActionServer(
            "/spot/cam/ptz/look_at_point",
            LookAtPointAction,
            execute_cb=self.handle_look_at_point_action,
            auto_start=False,
        )
        self.look_at_point_as.start()

        self.publish_ptz_list()

        self.position_thread = threading.Thread(target=self._publish_ptz_positions)
        self.position_thread.start()

        self.velocity_thread = threading.Thread(target=self._publish_ptz_velocities)
        self.velocity_thread.start()

        self.logger.info("Initialised ptz handler")

    def _ptz_limit_to_ros(self, limits) -> PTZLimits:
        """
        Convert a limit proto to ROS

        Args:
            limits: A limits proto

        Returns:
            PTZLimits
        """
        return PTZLimits(min=limits.min.value, max=limits.max.value)

    def _ptz_description_to_ros(self, description) -> PTZDescription:
        """
        Convert a ptz description proto to ROS

        Args:
            description: ptz description proto

        Returns:
            PTZDescription
        """
        desc = PTZDescription()
        desc.name = description.name
        desc.pan_limit = self._ptz_limit_to_ros(description.pan_limit)
        desc.tilt_limit = self._ptz_limit_to_ros(description.tilt_limit)
        desc.zoom_limit = self._ptz_limit_to_ros(description.zoom_limit)

        return desc

    def _ptz_state_to_ros(self, state, set_stamp=True) -> PTZState:
        """
        Convert a ptz velocity or position to ros

        Args:
            state: ptz velocity/position proto
            set_stamp: If true, set the timestamp for the proto as the current time

        Returns:
            PTZState
        """
        ros_state = PTZState()

        if set_stamp:
            ros_state.header.stamp = rospy.Time.now()

        ros_state.ptz = self._ptz_description_to_ros(state.ptz)
        ros_state.pan = state.pan.value
        ros_state.tilt = state.tilt.value
        ros_state.zoom = state.zoom.value

        return ros_state

    def publish_ptz_list(self):
        """
        Publish the list of available ptzs
        """
        self.ptz_list_publisher.publish(
            PTZDescriptionArray(
                ptzs=[self._ptz_description_to_ros(desc) for desc in self.list_ptzs()]
            )
        )

    def list_ptzs(self):
        """
        List available ptzs on the device
        """
        return self.client.list_ptz()

    def handle_set_ptz_velocity(self, req):
        """
        Handle a request to set the ptz position
        """
        self.set_ptz_velocity(
            req.command.ptz.name, req.command.pan, req.command.tilt, req.command.zoom
        )

        return (
            True,
            f"Successfully set ptz {req.command.ptz.name} to requested velocity",
        )

    def set_ptz_velocity(self, ptz_name, pan, tilt, zoom):
        """
        Set the position of the specified ptz

        Args:
            ptz_name: Name of the ptz to move
            pan: Pan in degrees per second
            tilt: Tilt in degrees per second
            zoom: Zoom in zoom levels per second
        """
        self.client.set_ptz_velocity(ptz_name, pan, tilt, zoom)

    def _publish_ptz_velocities(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            state_arr = PTZStateArray()
            for ptz_name in self.client.ptzs.keys():
                try:
                    state = self._ptz_state_to_ros(
                        self.client.get_ptz_velocity(ptz_name)
                    )
                    state_arr.ptzs.append(state)
                except RetryableUnavailableError as e:
                    self.logger.warning(f"Failed to get velocity for {ptz_name}: {e}")

            self.ptz_velocity_publisher.publish(state_arr)
            rate.sleep()

    def handle_set_ptz_position(self, req):
        """
        Handle a request to set the ptz position
        """
        if not req.command.ptz.name:
            return False, "PTZ name not specified."
        self.set_ptz_position(
            req.command.ptz.name, req.command.pan, req.command.tilt, req.command.zoom
        )
        return (
            True,
            f"Successfully set ptz {req.command.ptz.name} to requested velocity",
        )

    def set_ptz_position(self, ptz_name, pan, tilt, zoom):
        """
        Set the position of the specified ptz

        Args:
            ptz_name: Name of the ptz to move
            pan: Pan in degrees
            tilt: Tilt in degrees
            zoom: Zoom in zoom levels
        """
        self.client.set_ptz_position(ptz_name, pan, tilt, zoom)

    def _publish_ptz_positions(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            state_arr = PTZStateArray()
            for ptz_name in self.client.ptzs.keys():
                try:
                    state = self._ptz_state_to_ros(
                        self.client.get_ptz_position(ptz_name)
                    )
                    state_arr.ptzs.append(state)
                except RetryableUnavailableError as e:
                    self.logger.warning(f"Failed to get position for {ptz_name}: {e}")

            self.ptz_position_publisher.publish(state_arr)
            rate.sleep()

    def handle_reset_autofocus(self, _):
        """
        Handle a request to initialise or reset the ptz lens autofocus
        """
        try:
            self.initialise_lens()
        except Exception as e:
            return False, "Failed to reset or initialise autofocus"

        return True, "Reset autofocus"

    def initialise_lens(self):
        """
        Initialise or reset the ptz lens autofocus
        """
        self.client.initialise_lens()

    def handle_look_at_point_action(self, action: LookAtPointGoal):
        """
        Handle a call to the look at point actionserver
        """
        success, message = self.look_at_point(
            action.target,
            zoom_level=action.zoom_level,
            image_diagonal=action.image_width,
            track=action.track,
        )
        result = LookAtPointResult(success, message)

        if success:
            self.look_at_point_as.set_succeeded(result)
        else:
            self.look_at_point_as.set_aborted(result)

    def handle_look_at_point(self, req):
        """
        Handle a request to look at a point in space
        """

        return self.look_at_point(
            req.target,
            zoom_level=req.zoom_level,
            image_diagonal=req.image_width,
            track=req.track,
        )

    def look_at_point(
        self,
        target: PointStamped,
        zoom_level: float,
        image_diagonal: float,
        track: bool,
    ):
        if self._track_timer:
            self._track_timer.shutdown()

        if track:
            # If requesting to hold the point in view, we have to realign the camera every so often to make sure it
            # tracks
            realign_interval = rospy.Duration(0.1)

            def look_at_point_timer_cb(target_, zoom_level_, image_diagonal_, timer):
                """
                Helper function for the timer callback
                """
                res = self._look_at_point(
                    target_, zoom_level=zoom_level_, image_diagonal=image_diagonal_
                )
                if not res:
                    self.logger.error(res[1])

            # This timer will run every realign_interval and recompute the setting of the ptz to track the point
            self._track_timer = rospy.Timer(
                realign_interval,
                callback=functools.partial(
                    look_at_point_timer_cb, target, zoom_level, image_diagonal
                ),
            )
            return True, "Tracking point"
        else:
            return self._look_at_point(
                target, zoom_level=zoom_level, image_diagonal=image_diagonal
            )

    def _look_at_point(
        self,
        target: PointStamped,
        *,
        zoom_level: float = 1,
        image_diagonal: float = 0,
    ) -> typing.Tuple[bool, str]:
        """
        Point the ptz camera at a point in an arbitrary frame.
        The zoom is either given explicitly or calculated from a certain area to inspect around the target that
        should be captured given by its image diagonal in meters.

        Args:
            target: The target point and frame id that the Spot cam should be aimed at
            zoom_level: The zoom level that the Spot cam should be set to
            image_diagonal: The diagonal of the image frame in meters, overrides the zoom_level
        """

        # Look-up transform from ptz to given frame_id
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        frame_id = target.header.frame_id
        timeout = rospy.Duration(1)
        try:
            ptz_to_target_frame = tf_buffer.lookup_transform(
                "ptz_zero", frame_id, rospy.Time(), timeout
            )
        except tf2_ros.LookupException:
            return (
                False,
                "Can't lookup transform between 'ptz_zero' and '" + frame_id + "'!",
            )
        except Exception as e:
            return False, f"Failure while looking up transform: {e}"

        # Apply transform from frame to point
        rot = ptz_to_target_frame.transform.rotation
        trans = ptz_to_target_frame.transform.translation
        ptz_to_target = np.dot(
            np.dot(
                tf.transformations.translation_matrix([trans.x, trans.y, trans.z]),
                tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w]),
            ),
            tf.transformations.translation_matrix(
                [target.point.x, target.point.y, target.point.z]
            ),
        )

        point_in_ptz_frame = np.dot(ptz_to_target, np.array([0.0, 0.0, 0.0, 1.0]))[0:3]
        # To point the ptz at the point, must point the z axis at the point
        # Angle between the +z axis and the point projected onto the x-z plane. Applying this pan angle will have the
        # z axis in the correct orientation on the x-z plane
        pan_angle = np.rad2deg(np.arctan2(point_in_ptz_frame[0], point_in_ptz_frame[2]))
        # For the tilt, working in the y-z plane. How much rotation is needed to orient the z axis to the point?
        # Assuming we are in the rotated frame, the z coordinate of the target point is the hypotenuse of the right
        # triangle made from the x and z coordinates
        dxz = np.hypot(point_in_ptz_frame[0], point_in_ptz_frame[2])
        # The angle required to point the z axis at the coordinate is then the angle between the z axis and the
        # target point in the y-z plane
        tilt_angle = -np.rad2deg(np.arctan2(point_in_ptz_frame[1], dxz))
        if tilt_angle > 90:
            self.logger.warning(
                "Required tilt angle is greater than 90 degrees. This probably shouldn't happen."
            )
        if tilt_angle < -30:
            self.logger.warning(
                f"Required tilt angle {tilt_angle} is too low for the ptz. Will be clamped to -30"
            )

        distance_from_camera = np.hypot(point_in_ptz_frame[1], dxz)

        if image_diagonal > 0:
            # Optical zoom calculated with pinhole camera model
            # See https://en.wikipedia.org/wiki/Pinhole_camera_model#Formulation
            focal_length = (
                self.DIAGONAL_SENSOR_SIZE / image_diagonal * distance_from_camera
            )
            zoom_factor = focal_length / self.SHORTEST_FOCAL_LENGTH
            if zoom_factor < 1.0:
                self.logger.warning(
                    f"Desired zoom {zoom_factor} too small: Clamping at 1."
                )
                zoom_factor = 1.0
            elif zoom_factor > 30.0:
                self.logger.warning(
                    f"Desired zoom {zoom_factor} too large: Clamping at 30."
                )
                zoom_factor = 30.0
        else:
            zoom_factor = zoom_level

        self.logger.info(
            f"Setting ptz to pan: {pan_angle}, tilt: {tilt_angle}, zoom: {zoom_factor}"
        )
        try:
            self.set_ptz_position("mech", pan_angle, tilt_angle, zoom_factor)
            return True, "Set ptz to look at point"
        except Exception as e:
            return False, f"Failed to look at point: {e}"


class ImageStreamHandlerROS(ROSHandler):
    """
    This handles the image stream coming from the Spot CAM. Its output depends on the screen that has been chosen on
    the compositor.
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.image_client = wrapper.image
        # Need the compositor to get information about what camera is currently on screen so we can correctly set the
        # frame id of the image
        self.compositor_client = wrapper.compositor
        self.cv_bridge = CvBridge()
        self.image_pub = rospy.Publisher("/spot/cam/image", Image, queue_size=1)
        self.loop_thread = threading.Thread(target=self._publish_images_loop)
        self.loop_thread.start()
        self.logger.info("Initialised image stream handler")

    def _publish_images_loop(self):
        """
        We run this handler in a separate thread so it can loop and publish whenever the image is updated
        """
        loop_rate = rospy.Rate(50)
        last_image_time = self.image_client.last_image_time
        while not rospy.is_shutdown():
            if last_image_time != self.image_client.last_image_time:
                image = self.cv_bridge.cv2_to_imgmsg(
                    self.image_client.last_image, "bgr8"
                )
                image.header.stamp = (
                    rospy.Time.now()
                )  # Not strictly correct... but close enough?
                try:
                    visible_cameras = self.compositor_client.get_visible_cameras()
                    if len(visible_cameras) == 1:
                        # Can't set the frame id unless there is only a single camera displayed. There is no frame for a
                        # combined image
                        # The camera name here is in the form name:window, we just want the camera name
                        # https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#getvisiblecamerasresponse-stream
                        image.header.frame_id = visible_cameras[0].camera.name.split(
                            ":"
                        )[0]
                except RetryableUnavailableError as e:
                    self.logger.warning(f"Failed to get visible cameras: {e}")

                # TODO: This has to do frame switching in the published message headers depending on the compositor view
                self.image_pub.publish(image)
                last_image_time = self.image_client.last_image_time

            loop_rate.sleep()


class SpotCamROS:
    def __init__(self):
        self.username = rospy.get_param("~username", "default_value")
        self.password = rospy.get_param("~password", "default_value")
        self.hostname = rospy.get_param("~hostname", "default_value")

        self.logger = logging.getLogger("rosout")

        self.wrapper = SpotCamWrapper(
            self.hostname, self.username, self.password, self.logger
        )

        self.lighting = LightingHandlerROS(self.wrapper)
        self.power = PowerHandlerROS(self.wrapper)
        self.compositor = CompositorHandlerROS(self.wrapper)
        self.image = ImageStreamHandlerROS(self.wrapper)
        self.health = HealthHandlerROS(self.wrapper)
        self.audio = AudioHandlerROS(self.wrapper)
        self.stream_quality = StreamQualityHandlerROS(self.wrapper)
        self.transforms = TransformHandlerROS(self.wrapper)
        self.ptz = PTZHandlerROS(self.wrapper)

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.logger.info("Spot CAM ROS shutdown called")
        self.wrapper.shutdown()

    def main(self):
        rospy.spin()
