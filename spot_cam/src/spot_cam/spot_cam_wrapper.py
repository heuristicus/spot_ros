import asyncio
import datetime
import enum
import os.path
import threading
import typing
import wave

import bosdyn.client
import cv2
import numpy as np
from aiortc import RTCConfiguration
from bosdyn.client import Robot
from bosdyn.client import spot_cam
from bosdyn.client.spot_cam.compositor import CompositorClient
from bosdyn.client.spot_cam.lighting import LightingClient
from bosdyn.client.spot_cam.power import PowerClient
from bosdyn.client.spot_cam.health import HealthClient
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.client.spot_cam.streamquality import StreamQualityClient
from bosdyn.client.spot_cam.ptz import PtzClient
from bosdyn.client.spot_cam.media_log import MediaLogClient
from bosdyn.client.payload import PayloadClient
from bosdyn.api.spot_cam.ptz_pb2 import PtzDescription, PtzVelocity, PtzPosition
from bosdyn.api.spot_cam import audio_pb2

from spot_cam.webrtc_client import WebRTCClient
from spot_driver.spot_wrapper import SpotWrapper


class LightingWrapper:
    """
    Wrapper for LED brightness interaction
    """

    class LEDPosition(enum.Enum):
        """
        Values indicate the position of the specified LED in the brightness list
        """

        REAR_LEFT = 0
        FRONT_LEFT = 1
        FRONT_RIGHT = 2
        REAR_RIGHT = 3

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: LightingClient = robot.ensure_client(
            LightingClient.default_service_name
        )

    def set_led_brightness(self, brightness):
        """
        Set the brightness of the LEDs to the specified brightness

        Args:
            brightness: LEDs will all be set to this brightness, which should be in the range [0, 1]. The value will
                        be clipped if outside this range.
        """
        # Clamp brightness to [0,1] range
        brightness = min(max(brightness, 0), 1)
        self.client.set_led_brightness([brightness] * 4)

    def get_led_brightness(self) -> typing.List[float]:
        """
        Get the brightness of the LEDS

        Returns:
            List of floats indicating current brightness of each LED, in the order they are specified in the
            LEDPosition enum
        """
        return self.client.get_led_brightness()


class PowerWrapper:
    """
    Wrapper for power interaction
    """

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: PowerClient = robot.ensure_client(PowerClient.default_service_name)

    def get_power_status(self):
        """
        Get power status for the devices
        """
        return self.client.get_power_status()

    def set_power_status(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
    ):
        """
        Set power status for each of the devices

        Args:
            ptz:
            aux1: ??
            aux2: ??
            external_mic:
        """
        self.client.set_power_status(ptz, aux1, aux2, external_mic)

    def cycle_power(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
    ):
        """
        Cycle power of the specified devices

        Args:
            ptz:
            aux1:
            aux2:
            external_mic:
        """
        self.client.cycle_power(ptz, aux1, aux2, external_mic)


class CompositorWrapper:
    """
    Wrapper for compositor interaction
    """

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: CompositorClient = robot.ensure_client(
            CompositorClient.default_service_name
        )

    def list_screens(self) -> typing.List[str]:
        """
        List the available screens - this includes individual cameras and also panoramic and other stitched images
        provided by the camera

        Returns:
             List of strings indicating available screens
        """
        return [screen.name for screen in self.client.list_screens()]

    def get_visible_cameras(self):
        """
        Get the camera data for the camera currently visible on the stream

        Returns:
            List of visible camera streams
        """
        return self.client.get_visible_cameras()

    def set_screen(self, screen):
        """
        Set the screen to be streamed over the network

        Args:
            screen: Screen to show
        """
        self.client.set_screen(screen)

    def set_ir_colormap(self, colormap, min_temp, max_temp, auto_scale=True):
        """
        Set the colormap used for the IR camera

        Args:
            colormap: Colormap to use, options are https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ircolormap-colormap
            min_temp: Minimum temperature on the scale
            max_temp: Maximum temperature on the scale
            auto_scale: Auto-scales the colormap according to the image. If this is set min_temp and max_temp are
                        ignored
        """
        self.client.set_ir_colormap(colormap, min_temp, max_temp, auto_scale)

    def set_ir_meter_overlay(self, x, y, enable=True):
        """
        Set the reticle position on the Spot CAM IR.

        Args:
            x: Horizontal coordinate between 0 and 1
            y: Vertical coordinate between 0 and 1
            enable: If true, enable the reticle on the display
        """
        self.client.set_ir_meter_overlay(x, y, enable)


class HealthWrapper:
    """
    Wrapper for health details
    """

    def __init__(self, robot, logger):
        self.client: HealthClient = robot.ensure_client(
            HealthClient.default_service_name
        )
        self.logger = logger

    def get_bit_status(
        self,
    ) -> typing.Tuple[typing.List[str], typing.List[typing.Tuple[int, str]]]:
        """
        Get fault events and degradations

        Returns:
            Dictionary

        """
        bit_status = self.client.get_bit_status()
        events = []
        for event in bit_status[0]:
            events.append(event)

        degradations = []
        for degradation in bit_status[1]:
            degradations.append((degradation.type, degradation.description))
        return events, degradations

    def get_temperature(self) -> typing.Tuple[str, float]:
        """
        Get temperatures of various components of the camera

        Returns:
            Tuple of string and float indicating the component and its temperature in celsius
        """
        return [
            (composite.channel_name, composite.temperature / 1000.0)
            for composite in self.client.get_temperature()
        ]

    # def get_system_log(self):
    #     """
    #     This seems to always time out
    #     """
    #     return self.client.get_system_log()


class AudioWrapper:
    """
    Wrapper for audio commands on the camera
    """

    def __init__(self, robot, logger):
        self.client: AudioClient = robot.ensure_client(AudioClient.default_service_name)
        self.logger = logger

    def list_sounds(self) -> typing.List[str]:
        """
        List sounds available on the device

        Returns:
            List of names of available sounds
        """
        return self.client.list_sounds()

    def set_volume(self, percentage):
        """
        Set the volume at which sounds should be played

        Args:
            percentage: How loud sounds should be from 0 to 100%
        """
        self.client.set_volume(percentage)

    def get_volume(self):
        """
        Get the current volume at which sounds are played

        Returns:
            Current volume as a percentage
        """
        return self.client.get_volume()

    def play_sound(self, sound_name, gain=1.0):
        """
        Play a sound which is on the device

        Args:
            sound_name: Name of the sound to play
            gain: Volume gain multiplier
        """
        sound = audio_pb2.Sound(name=sound_name)
        self.client.play_sound(sound, gain)

    def load_sound(self, sound_file, name):
        """
        Load a sound from a wav file and save it with the given name onto the device
        Args:
            sound_file: Wav file to read from
            name: Name to assign to the sound

        Raises:
            IOError: If the given file is not a file
            wave.Error: If the given file is not a wav file
        """
        full_path = os.path.abspath(os.path.expanduser(sound_file))
        print(full_path)
        if not os.path.isfile(full_path):
            raise IOError(f"Tried to load sound from {full_path} but it is not a file.")

        sound = audio_pb2.Sound(name=name)

        with wave.open(full_path, "rb") as fh:
            # Use this to make sure that the file is actually a wav file
            pass

        with open(full_path, "rb") as fh:
            data = fh.read()

        self.client.load_sound(sound, data)

    def delete_sound(self, name):
        """
        Delete a sound from the device

        Args:
            name: Name of the sound to delete
        """
        self.client.delete_sound(audio_pb2.Sound(name=name))


class StreamQualityWrapper:
    """
    Wrapper for stream quality commands
    """

    def __init__(self, robot, logger):
        self.client: StreamQualityClient = robot.ensure_client(
            StreamQualityClient.default_service_name
        )
        self.logger = logger

    def set_stream_params(self, target_bitrate, refresh_interval, idr_interval, awb):
        """
        Set image compression and postprocessing parameters

        Note: It is currently not possible to turn off the auto white balance. You will get a crash

        Args:
            target_bitrate: Compression level target in bits per second
            refresh_interval: How often the whole feed should be refreshed (in frames)
            idr_interval: How often an IDR message should be sent (in frames)
            awb: Mode for automatic white balance
        """
        self.client.set_stream_params(target_bitrate, refresh_interval, idr_interval, 0)

    def get_stream_params(self) -> typing.Dict[str, int]:
        """
        Get the current stream quality parameters

        Returns:
            Dictionary containing the parameters
        """
        params = self.client.get_stream_params()
        param_dict = {
            "target_bitrate": params.targetbitrate.value,
            "refresh_interval": params.refreshinterval.value,
            "idr_interval": params.idrinterval.value,
            "awb": params.awb.awb,
        }

        return param_dict

    def enable_congestion_control(self, enable):
        """
        Enable congestion control on the receiver... not sure what this does

        Args:
            enable: If true, enable congestion control
        """
        self.client.enable_congestion_control(enable)


class MediaLogWrapper:
    """
    Wrapper for interacting with the media log. And importantly, information about the cameras themselves
    """

    def __init__(self, robot, logger):
        self.client: MediaLogClient = robot.ensure_client(
            MediaLogClient.default_service_name
        )
        self.logger = logger

    def list_cameras(self) -> typing.List:
        """
        List the cameras on the spot cam
        """
        return self.client.list_cameras()


class PTZWrapper:
    """
    Wrapper for controlling the PTZ unit
    """

    def __init__(self, robot, logger):
        self.client: PtzClient = robot.ensure_client(PtzClient.default_service_name)
        self.logger = logger
        self.ptzs = {}
        descriptions = self.client.list_ptz()
        for description in descriptions:
            self.ptzs[description.name] = description

    def list_ptz(self) -> typing.Dict[str, typing.Dict]:
        """
        List the available ptz units on the device

        Returns:
            Dict of descriptions of ptz units
        """
        ptzs = []

        descriptions = self.client.list_ptz()
        for ptz_desc in descriptions:
            ptzs.append(ptz_desc)
            # Also update the internal list of raw ptz definitions
            self.ptzs[ptz_desc.name] = ptz_desc

        return ptzs

    def _get_ptz_description(self, name):
        """
        Get the bosdyn version of the ptz description

        Args:
            name: Get description for this ptz

        Returns:
            PtzDescription
        """
        if name not in self.ptzs:
            self.logger.warn(
                f"Tried to retrieve description for ptz {name} but it does not exist."
            )
            return None

        return self.ptzs[name]

    def _clamp_value_to_limits(self, value, limits: PtzDescription.Limits):
        """
        Clamp the given value to the specified limits. If the limits are unspecified (i.e. both 0), the value is not
        clamped

        Args:
            value: Value to clamp
            limits: PTZ description limit proto

        Returns:
            Value clamped to limits
        """
        if limits.max.value == 0 and limits.min.value == 0:
            # If both max and min are zero, this means the limit is unset. The documentation states that if a limit
            # is unset, then all positions are valid.
            # https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ptzdescription
            return value

        return max(min(value, limits.max.value), limits.min.value)

    def _clamp_request_to_limits(
        self, ptz_name, pan, tilt, zoom
    ) -> typing.Tuple[float, float, float]:
        """

        Args:
            ptz_name: Name of the ptz for which the pan, tilt, and zoom should be clamped

        Returns:
            Tuple of pan, tilt, zoom, clamped to the limits of the requested ptz
        """
        ptz_desc = self._get_ptz_description(ptz_name)

        return (
            self._clamp_value_to_limits(pan, ptz_desc.pan_limit),
            self._clamp_value_to_limits(tilt, ptz_desc.tilt_limit),
            self._clamp_value_to_limits(zoom, ptz_desc.zoom_limit),
        )

    def get_ptz_position(self, ptz_name) -> PtzPosition:
        """
        Get the position of the ptz with the given name

        Args:
            ptz_name: Name of the ptz

        Returns:
            ptz position proto
        """
        return self.client.get_ptz_position(PtzDescription(name=ptz_name))

    def set_ptz_position(self, ptz_name, pan, tilt, zoom):
        """
        Set the position of the specified ptz

        Args:
            ptz_name: Name of the ptz
            pan: Set the pan to this value in degrees
            tilt: Set the tilt to this value in degrees
            zoom: Set the zoom to this zoom level
        """
        pan, tilt, zoom = self._clamp_request_to_limits(ptz_name, pan, tilt, zoom)
        self.client.set_ptz_position(
            self._get_ptz_description(ptz_name), pan, tilt, zoom
        )

    def get_ptz_velocity(self, ptz_name) -> PtzVelocity:
        """
        Get the velocity of the ptz with the given name

        Args:
            ptz_name: Name of the ptz

        Returns:
            ptz velocity proto
        """
        return self.client.get_ptz_velocity(PtzDescription(name=ptz_name))

    def set_ptz_velocity(self, ptz_name, pan, tilt, zoom):
        """
        Set the velocity of the various axes of the specified ptz

        Args:
            ptz_name: Name of the ptz
            pan: Set the pan to this value in degrees per second
            tilt: Set the tilt to this value in degrees per second
            zoom: Set the zoom to this value in zoom level per second
        """
        # We do not clamp the velocity to the limits, as it is a rate
        self.client.set_ptz_velocity(
            self._get_ptz_description(ptz_name), pan, tilt, zoom
        )

    def initialise_lens(self):
        """
        Initialises or resets ptz autofocus
        """
        self.client.initialize_lens()


class ImageStreamWrapper:
    """
    A wrapper for the image stream from WebRTC

    Can view the same stream at https://192.168.50.3:31102/h264.sdp.html (depending on the IP of the robot)

    Contains functions adapted from
    https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/spot_cam/webrtc.py
    """

    def __init__(
        self,
        hostname: str,
        robot,
        logger,
        sdp_port=31102,
        sdp_filename="h264.sdp",
        cam_ssl_cert_path=None,
    ):
        """
        Initialise the wrapper

        Args:
            hostname: Hostname/IP of the robot
            robot: Handle for the robot the camera is on
            logger: Logger to use
            sdp_port: SDP port of Spot's WebRTC server
            sdp_filename: File being streamed from the WebRTC server
            cam_ssl_cert_path: Path to the Spot CAM's client cert to check with Spot CAM server
        """
        self.shutdown_flag = threading.Event()
        self.logger = logger
        self.last_image_time = None
        self.image_lock = threading.Lock()
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        config = RTCConfiguration(iceServers=[])
        self.client = WebRTCClient(
            hostname,
            sdp_port,
            sdp_filename,
            cam_ssl_cert_path if cam_ssl_cert_path else False,
            robot.user_token,
            config,
        )

        asyncio.gather(
            self.client.start(),
            self._process_func(),
            self._monitor_shutdown(),
        )
        # Put the async loop into a separate thread so we can continue initialisation
        self.async_thread = threading.Thread(target=loop.run_forever)
        self.async_thread.start()

    async def _monitor_shutdown(self):
        while not self.shutdown_flag.is_set():
            await asyncio.sleep(1.0)

        self.logger.info("Image stream wrapper received shutdown flag")
        await self.client.pc.close()
        asyncio.get_event_loop().stop()

    async def _process_func(self):
        while asyncio.get_event_loop().is_running():
            try:
                frame = await self.client.video_frame_queue.get()

                pil_image = frame.to_image()
                cv_image = np.array(pil_image)
                # OpenCV needs BGR
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                with self.image_lock:
                    self.last_image_time = datetime.datetime.now()
                    self.last_image = cv_image
            except Exception as e:
                self.logger.error(f"Image stream wrapper exception {e}")
            try:
                # discard audio frames
                while not self.client.audio_frame_queue.empty():
                    await self.client.audio_frame_queue.get()
            except Exception as e:
                self.logger.error(
                    f"Image stream wrapper exception while discarding audio frames {e}"
                )

        self.shutdown_flag.set()


class SpotCamWrapper:
    def __init__(self, hostname, username, password, logger):
        self._hostname = hostname
        self._username = username
        self._password = password
        self._logger = logger

        # Create robot object and authenticate.
        self.sdk = bosdyn.client.create_standard_sdk("Spot CAM Client")
        spot_cam.register_all_service_clients(self.sdk)

        self.robot = self.sdk.create_robot(self._hostname)
        SpotWrapper.authenticate(
            self.robot, self._hostname, self._username, self._password, self._logger
        )

        self.payload_client: PayloadClient = self.robot.ensure_client(
            PayloadClient.default_service_name
        )
        self.payload_details = None
        for payload in self.payload_client.list_payloads():
            if payload.is_enabled and "Spot CAM" in payload.name:
                self.payload_details = payload

        if not self.payload_details:
            raise SystemError(
                "Expected an enabled payload with Spot CAM in the name. This does not appear to exist. "
                "Please verify that the spot cam is correctly configured in the payload list on the "
                "admin interface"
            )

        self.lighting = LightingWrapper(self.robot, self._logger)
        self.power = PowerWrapper(self.robot, self._logger)
        self.compositor = CompositorWrapper(self.robot, self._logger)
        self.image = ImageStreamWrapper(self._hostname, self.robot, self._logger)
        self.health = HealthWrapper(self.robot, self._logger)
        self.audio = AudioWrapper(self.robot, self._logger)
        self.stream_quality = StreamQualityWrapper(self.robot, self._logger)
        self.media_log = MediaLogWrapper(self.robot, self._logger)
        self.ptz = PTZWrapper(self.robot, self._logger)

        self._logger.info("Finished setting up spot cam wrapper components")

    def shutdown(self):
        self._logger.info("Shutting down Spot CAM wrapper")
        self.image.shutdown_flag.set()
