import rospy
import logging
from spot_cam.spot_cam_wrapper import SpotCamWrapper
from std_msgs.msg import Float32MultiArray, Float32


class LightingHandler:
    def __init__(self, wrapper: SpotCamWrapper):
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

    def leds_callback(self, msg):
        self.set_leds(msg.data)

    def set_leds(self, brightness):
        self.lighting_client.set_led_brightness(brightness)
        self._publish_leds()

    def _publish_leds(self):
        self.led_publisher.publish(
            Float32MultiArray(data=self.lighting_client.get_led_brightness())
        )


class SpotCam:
    def __init__(self):
        # self.rates = rospy.get_param('~rates', {})
        # if "loop_frequency" in self.rates:
        #     loop_rate = self.rates["loop_frequency"]
        # else:
        #     loop_rate = 50
        #
        # for param, rate in self.rates.items():
        #     if rate > loop_rate:
        #         rospy.logwarn("{} has a rate of {} specified, which is higher than the loop rate of {}. It will not "
        #                       "be published at the expected frequency".format(param, rate, loop_rate))

        #        rate = rospy.Rate(loop_rate)
        self.username = rospy.get_param("~username", "default_value")
        self.password = rospy.get_param("~password", "default_value")
        self.hostname = rospy.get_param("~hostname", "default_value")

        self.logger = logging.getLogger("rosout")

        self.wrapper = SpotCamWrapper(
            self.hostname, self.username, self.password, self.logger
        )

        self.lighting = LightingHandler(self.wrapper)

    def main(self):
        rospy.spin()
