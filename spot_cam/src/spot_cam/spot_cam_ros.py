import rospy
import logging
from spot_cam.spot_cam_wrapper import SpotCamWrapper


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

        wrapper = SpotCamWrapper(
            self.hostname, self.username, self.password, self.logger
        )

    def main(self):
        rospy.spin()
