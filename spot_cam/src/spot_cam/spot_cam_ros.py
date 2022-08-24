import logging
import threading
import typing

import rospy
from bosdyn.client.exceptions import InvalidRequestError
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from spot_cam.msg import PowerStatus, Temperature, TemperatureArray, BITStatus, Degradation
from spot_cam.srv import SetIRColormap, SetIRMeterOverlay, SetString
from std_msgs.msg import Float32MultiArray, Float32, String

from spot_cam.spot_cam_wrapper import SpotCamWrapper
from spot_driver.ros_helpers import getSystemFaults


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
            "spot/cam/leds", Float32MultiArray, latch=True, queue_size=1
        )
        self.led_subscriber = rospy.Subscriber(
            "spot/cam/set_leds", Float32, callback=self.leds_callback
        )
        # Allow time for publisher to initialise before publishing led state
        rospy.sleep(0.3)
        self._publish_leds()

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
    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.power_client = wrapper.power
        self.power_publisher = rospy.Publisher(
            "spot/cam/power", PowerStatus, queue_size=1, latch=True
        )
        self.power_subscriber = rospy.Subscriber(
            "spot/cam/set_power", PowerStatus, self.power_callback
        )
        self.power_subscriber = rospy.Subscriber(
            "spot/cam/cycle_power", PowerStatus, self.cycle_callback
        )
        self._publish_power()

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
        self.set_ir_colormap_service = rospy.Service(
            "/spot/cam/set_ir_colormap", SetIRColormap, self.handle_set_ir_colormap
        )

    def handle_set_screen(self, req: SetString):
        try:
            self.set_screen(req.value)
            return True, f"Successfully set screen to {req.value}"
        except InvalidRequestError as e:
            message = f"{e}.\nValid screens are {self.client.list_screens()}"
            self.logger.error(message)
            return False, message

    def set_screen(self, screen):
        """

        Args:
            screen: Screen to display

        Returns:
        """
        self.client.set_screen(screen)

    def handle_set_ir_meter_overlay(self, req: SetIRMeterOverlay):
        self.set_ir_meter_overlay(req.x, req.y, req.enable)
        return True, "Successfully set IR overlay"

    def set_ir_meter_overlay(self, x, y, enable=True):
        self.client.set_ir_meter_overlay(x, y, enable)

    def handle_set_ir_colormap(self, req: SetIRColormap):
        self.set_ir_colormap(req.colormap, req.min, req.max, req.auto_scale)
        return True, "Successfully set IR colormap"

    def set_ir_colormap(self, colormap, min, max, auto_scale=True):
        self.client.set_ir_colormap(colormap, min, max, auto_scale)

class HealthHandlerROS(ROSHandler):

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.health
        self.robot = wrapper.robot
        self.temp_publisher = rospy.Publisher("/spot/cam/temperatures", TemperatureArray, queue_size=1)
        self.status_publisher = rospy.Publisher("/spot/cam/status", BITStatus, queue_size=1)

        self.temp_thread = threading.Thread(target=self._publish_temperatures_loop)
        self.temp_thread.start()

        self.status_thread = threading.Thread(target=self._publish_status_loop)
        self.status_thread.start()

    def get_status(self):
        events, degradations = self.client.get_bit_status()
        status = BITStatus()
        status.events = getSystemFaults(events, self.robot)

        for degradation in degradations:
            status.degradations.append(Degradation(type=degradation[0], description=degradation[1]))

        return status

    def publish_status(self):
        self.status_publisher.publish(self.get_status())

    def _publish_status_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_status()
            rate.sleep()

    def get_temperatures(self):
        temps = self.client.get_temperature()
        temp_array = TemperatureArray()
        for temp in temps:
            temp_msg = Temperature()
            temp_msg.channel_name = temp[0]
            temp_msg.temperature = temp[1]
            temp_array.temperatures.append(temp_msg)

        return temp_array

    def publish_temperatures(self):
        self.temp_publisher.publish(self.get_temperatures())

    def _publish_temperatures_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_temperatures()
            rate.sleep()


class ImageStreamHandlerROS(ROSHandler):
    """
    This handles the image stream coming from the Spot CAM. Its output depends on the screen that has been chosen on
    the compositor.
    """

    def __init__(self, wrapper: SpotCamWrapper):
        super().__init__()
        self.client = wrapper.image
        self.cv_bridge = CvBridge()
        self.image_pub = rospy.Publisher("/spot/cam/image", Image, queue_size=1)
        self.loop_thread = threading.Thread(target=self._publish_images_loop)
        self.loop_thread.start()

    def _publish_images_loop(self):
        """
        We run this handler in a separate thread so it can loop and publish whenever the image is updated
        """
        loop_rate = rospy.Rate(50)
        last_image_time = self.client.last_image_time
        while not rospy.is_shutdown():
            if last_image_time != self.client.last_image_time:
                image = self.cv_bridge.cv2_to_imgmsg(self.client.last_image, "bgr8")
                image.header.stamp = (
                    rospy.Time.now()
                )  # Not strictly correct... but close enough?
                # TODO: This has to do frame switching in the published message headers depending on the compositor view
                self.image_pub.publish(image)
                last_image_time = self.client.last_image_time

            loop_rate.sleep()


class SpotCam:
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

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.logger.info("Spot CAM ROS shutdown called")
        self.wrapper.shutdown()

    def main(self):
        rospy.spin()
