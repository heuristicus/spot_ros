import typing
import rospy
import threading
import logging
from spot_cam.spot_cam_wrapper import SpotCamWrapper
from std_msgs.msg import Float32MultiArray, Float32
from spot_cam.msg import PowerStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


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

        self.loop_thread = threading.Thread(target=self._run)
        self.loop_thread.start()

    def _run(self):
        """
        We run this handler in a separate thread so it can loop and publish whenever the image is updated
        """
        loop_rate = rospy.Rate(20)
        last_image_time = self.client.last_image_time
        while not rospy.is_shutdown():
            if last_image_time != self.client.last_image_time:
                self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.client.last_image, "bgr8"))
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
        self.power = ImageStreamHandlerROS(self.wrapper)

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.logger.info("Spot CAM ROS shutdown called")
        self.wrapper.shutdown()

    def main(self):
        rospy.spin()