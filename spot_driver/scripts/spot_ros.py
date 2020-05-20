#!/usr/bin/env python3
import rospy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped

from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray, LeaseResource
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray

from ros_helpers import *
from spot_wrapper import SpotWrapper

import logging

class SpotROS():
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self):
        self.spot_wrapper = None

        self.callbacks = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["robot_state"] = self.RobotStateCB
        self.callbacks["metrics"] = self.MetricsCB
        self.callbacks["robot_command"] = self.RobotCommandCB
        self.callbacks["power"] = self.PowerCB
        self.callbacks["lease"] = self.LeaseCB
        self.callbacks["front_image"] = self.FrontImageCB
        self.callbacks["side_image"] = self.SideImageCB
        self.callbacks["rear_image"] = self.RearImageCB
        self.callbacks["estop"] = self.EstopCB

    def RobotStateCB(self, results):
        """Callback for when the Spot Wrapper gets new robot state data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        state = self.spot_wrapper.robot_state

        if state:
            ## joint states ##
            joint_state = GetJointStatesFromState(state)
            self.joint_state_pub.publish(joint_state)

            ## TF ##
            tf_msg = GetTFFromState(state)
            if len(tf_msg.transforms) > 0:
                self.tf_pub.publish(tf_msg)

            # Odom Twist #
            twist_odom_msg = GetOdomTwistFromState(state)
            self.odom_twist_pub.publish(twist_odom_msg)

            # Feet #
            foot_array_msg = GetFeetFromState(state)
            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = GetEStopStateFromState(state)
            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = GetWifiFromState(state)
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = GetBatteryStatesFromState(state)
            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = GetPowerStatesFromState(state)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = GetSystemFaultsFromState(state)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = getBehaviorFaultsFromState(state)
            self.behavior_faults_pub.publish(behavior_fault_state_msg)

    def MetricsCB(self, results):
        """Callback for when the Spot Wrapper gets new metrics data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            metrics_msg.header.stamp = rospy.Time(metrics.timestamp.seconds, metrics.timestamp.nanos)

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                if metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                if metric.label == "time moving":
                    metrics_msg.time_moving = rospy.Time(metric.duration.seconds, metric.duration.nanos)
                if metric.label == "electric power":
                    metrics_msg.electric_power = rospy.Time(metric.duration.seconds, metric.duration.nanos)

            self.metrics_pub.publish(metrics_msg)

    def RobotCommandCB(self, results):
        """Callback for when the Spot Wrapper gets new robot command data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        # TODO: All of this
        rospy.logdebug("##### COMMAND #####")
        #rospy.loginfo(str(self.spot_wrapper.robot_command))

    def PowerCB(self, results):
        """Callback for when the Spot Wrapper gets new power data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        # TODO: All of this
        rospy.logdebug("##### POWER #####")
        #rospy.logwarn(str(self.spot_wrapper.power))

    def LeaseCB(self, results):
        """Callback for when the Spot Wrapper gets new lease data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease
        if lease_list:
            for resource in lease_list:
                new_resource = LeaseResource()
                new_resource.resource = resource.resource
                new_resource.lease.resource = resource.lease.resource
                new_resource.lease.epoch = resource.lease.epoch

                for seq in resource.lease.sequence:
                    new_resource.lease.sequence.append(seq)

                new_resource.lease_owner.client_name = resource.lease_owner.client_name
                new_resource.lease_owner.user_name = resource.lease_owner.user_name

                lease_array_msg.resources.append(new_resource)

            self.lease_pub.publish(lease_array_msg)

    def FrontImageCB(self, results):
        """Callback for when the Spot Wrapper gets new front image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.front_images
        if data:
            image_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(data[0])
            self.frontleft_image_pub.publish(image_msg0)
            self.frontleft_image_info_pub.publish(camera_info_msg0)
            self.tf_pub.publish(tf_msg0)
            image_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(data[1])
            self.frontright_image_pub.publish(image_msg1)
            self.frontright_image_info_pub.publish(camera_info_msg1)
            self.tf_pub.publish(tf_msg1)
            image_msg2, camera_info_msg2, camera_tf_msg2 = getImageMsg(data[2])
            self.frontleft_depth_pub.publish(image_msg2)
            self.frontleft_depth_info_pub.publish(camera_info_msg2)
            self.tf_pub.publish(tf_msg2)
            image_msg3, camera_info_msg3, camera_tf_msg3 = getImageMsg(data[3])
            self.frontright_depth_pub.publish(image_msg3)
            self.frontright_depth_info_pub.publish(camera_info_msg3)
            self.tf_pub.publish(tf_msg3)

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.side_images
        if data:
            image_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(data[0])
            self.left_image_pub.publish(image_msg0)
            self.left_image_info_pub.publish(camera_info_msg0)
            self.tf_pub.publish(tf_msg0)
            image_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(data[1])
            self.right_image_pub.publish(image_msg1)
            self.right_image_info_pub.publish(camera_info_msg1)
            self.tf_pub.publish(tf_msg1)
            image_msg2, camera_info_msg2, camera_tf_msg2 = getImageMsg(data[2])
            self.left_depth_pub.publish(image_msg2)
            self.left_depth_info_pub.publish(camera_info_msg2)
            self.tf_pub.publish(tf_msg2)
            image_msg3, camera_info_msg3, camera_tf_msg3 = getImageMsg(data[3])
            self.right_depth_pub.publish(image_msg3)
            self.right_depth_info_pub.publish(camera_info_msg3)
            self.tf_pub.publish(tf_msg3)

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.rear_images
        if data:
            mage_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(data[0])
            self.back_image_pub.publish(mage_msg0)
            self.back_image_info_pub.publish(camera_info_msg0)
            self.tf_pub.publish(tf_msg0)
            mage_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(data[1])
            self.back_depth_pub.publish(mage_msg1)
            self.back_depth_info_pub.publish(camera_info_msg1)
            self.tf_pub.publish(tf_msg1)

    def EstopCB(self, results):
        """Callback for when the Spot Wrapper gets new estop data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        rospy.logdebug("##### ESTOP #####")

    def main(self):
        """Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.  Holds lease from wrapper and updates all async tasks at the ROS rate"""
        rospy.init_node('spot_ros', anonymous=True)
        rate = rospy.Rate(50)

        self.rates = rospy.get_param('~rates', {})
        self.username = rospy.get_param('~username', 'default_value')
        self.password = rospy.get_param('~password', 'default_value')
        self.app_token = rospy.get_param('~app_token', 'default_value')
        self.hostname = rospy.get_param('~hostname', 'default_value')

        self.logger = logging.getLogger('rosout')

        rospy.loginfo("Starting")
        self.spot_wrapper = SpotWrapper(self.username, self.password, self.app_token, self.hostname, self.logger, self.rates, self.callbacks)

        if self.spot_wrapper._robot:
            # Images #
            self.back_image_pub = rospy.Publisher('camera/back/image', Image, queue_size=10)
            self.frontleft_image_pub = rospy.Publisher('camera/frontleft/image', Image, queue_size=10)
            self.frontright_image_pub = rospy.Publisher('camera/frontright/image', Image, queue_size=10)
            self.left_image_pub = rospy.Publisher('camera/left/image', Image, queue_size=10)
            self.right_image_pub = rospy.Publisher('camera/right/image', Image, queue_size=10)
            # Depth #
            self.back_depth_pub = rospy.Publisher('depth/back/image', Image, queue_size=10)
            self.frontleft_depth_pub = rospy.Publisher('depth/frontleft/image', Image, queue_size=10)
            self.frontright_depth_pub = rospy.Publisher('depth/frontright/image', Image, queue_size=10)
            self.left_depth_pub = rospy.Publisher('depth/left/image', Image, queue_size=10)
            self.right_depth_pub = rospy.Publisher('depth/right/image', Image, queue_size=10)

            # Image Camera Info #
            self.back_image_info_pub = rospy.Publisher('camera/back/camera_info', CameraInfo, queue_size=10)
            self.frontleft_image_info_pub = rospy.Publisher('camera/frontleft/camera_info', CameraInfo, queue_size=10)
            self.frontright_image_info_pub = rospy.Publisher('camera/frontright/camera_info', CameraInfo, queue_size=10)
            self.left_image_info_pub = rospy.Publisher('camera/left/camera_info', CameraInfo, queue_size=10)
            self.right_image_info_pub = rospy.Publisher('camera/right/camera_info', CameraInfo, queue_size=10)
            # Depth Camera Info #
            self.back_depth_info_pub = rospy.Publisher('depth/back/camera_info', CameraInfo, queue_size=10)
            self.frontleft_depth_info_pub = rospy.Publisher('depth/frontleft/camera_info', CameraInfo, queue_size=10)
            self.frontright_depth_info_pub = rospy.Publisher('depth/frontright/camera_info', CameraInfo, queue_size=10)
            self.left_depth_info_pub = rospy.Publisher('depth/left/camera_info', CameraInfo, queue_size=10)
            self.right_depth_info_pub = rospy.Publisher('depth/right/camera_info', CameraInfo, queue_size=10)

            self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
            """Defining a TF publisher manually because of conflicts between Python3 and tf"""
            self.tf_pub = rospy.Publisher('tf', TFMessage, queue_size=10)
            self.metrics_pub = rospy.Publisher('status/metrics', Metrics, queue_size=10)
            self.lease_pub = rospy.Publisher('status/leases', LeaseArray, queue_size=10)
            self.odom_twist_pub = rospy.Publisher('odometry/twist', TwistWithCovarianceStamped, queue_size=10)
            self.feet_pub = rospy.Publisher('status/feet', FootStateArray, queue_size=10)
            self.estop_pub = rospy.Publisher('status/estop', EStopStateArray, queue_size=10)
            self.wifi_pub = rospy.Publisher('status/wifi', WiFiState, queue_size=10)
            self.power_pub = rospy.Publisher('status/power_state', PowerState, queue_size=10)
            self.battery_pub = rospy.Publisher('status/battery_states', BatteryStateArray, queue_size=10)
            self.behavior_faults_pub = rospy.Publisher('status/behavior_faults', BehaviorFaultState, queue_size=10)
            self.system_faults_pub = rospy.Publisher('status/system_faults', SystemFaultState, queue_size=10)

            rospy.loginfo("Connecting")
            self.spot_wrapper.connect()
            rospy.loginfo("Running")
            with self.spot_wrapper.getLease():
                while not rospy.is_shutdown():
                    self.spot_wrapper.updateTasks()
                    rate.sleep()

if __name__ == "__main__":
    SR = SpotROS()
    SR.main()
