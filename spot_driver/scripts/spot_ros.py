#!/usr/bin/env python3
import rospy

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, Pose

from bosdyn.api.geometry_pb2 import Quaternion
import bosdyn.geometry

from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray, LeaseResource
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray
from spot_msgs.msg import Feedback

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
        self.callbacks["lease"] = self.LeaseCB
        self.callbacks["front_image"] = self.FrontImageCB
        self.callbacks["side_image"] = self.SideImageCB
        self.callbacks["rear_image"] = self.RearImageCB

    def RobotStateCB(self, results):
        """Callback for when the Spot Wrapper gets new robot state data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        state = self.spot_wrapper.robot_state

        if state:
            ## joint states ##
            joint_state = GetJointStatesFromState(state, self.spot_wrapper)
            self.joint_state_pub.publish(joint_state)

            ## TF ##
            tf_msg = GetTFFromState(state, self.spot_wrapper)
            if len(tf_msg.transforms) > 0:
                self.tf_pub.publish(tf_msg)

            # Odom Twist #
            twist_odom_msg = GetOdomTwistFromState(state, self.spot_wrapper)
            self.odom_twist_pub.publish(twist_odom_msg)

            # Feet #
            foot_array_msg = GetFeetFromState(state, self.spot_wrapper)
            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = GetEStopStateFromState(state, self.spot_wrapper)
            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = GetWifiFromState(state, self.spot_wrapper)
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = GetBatteryStatesFromState(state, self.spot_wrapper)
            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = GetPowerStatesFromState(state, self.spot_wrapper)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = GetSystemFaultsFromState(state, self.spot_wrapper)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = getBehaviorFaultsFromState(state, self.spot_wrapper)
            self.behavior_faults_pub.publish(behavior_fault_state_msg)

    def MetricsCB(self, results):
        """Callback for when the Spot Wrapper gets new metrics data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            local_time = self.spot_wrapper.robotToLocalTime(metrics.timestamp)
            metrics_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)

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
            image_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.frontleft_image_pub.publish(image_msg0)
            self.frontleft_image_info_pub.publish(camera_info_msg0)
            self.tf_pub.publish(camera_tf_msg0)
            image_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.frontright_image_pub.publish(image_msg1)
            self.frontright_image_info_pub.publish(camera_info_msg1)
            self.tf_pub.publish(camera_tf_msg1)
            image_msg2, camera_info_msg2, camera_tf_msg2 = getImageMsg(data[2], self.spot_wrapper)
            self.frontleft_depth_pub.publish(image_msg2)
            self.frontleft_depth_info_pub.publish(camera_info_msg2)
            self.tf_pub.publish(camera_tf_msg2)
            image_msg3, camera_info_msg3, camera_tf_msg3 = getImageMsg(data[3], self.spot_wrapper)
            self.frontright_depth_pub.publish(image_msg3)
            self.frontright_depth_info_pub.publish(camera_info_msg3)
            self.tf_pub.publish(camera_tf_msg3)

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.side_images
        if data:
            image_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.left_image_pub.publish(image_msg0)
            self.left_image_info_pub.publish(camera_info_msg0)
            self.tf_pub.publish(camera_tf_msg0)
            image_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.right_image_pub.publish(image_msg1)
            self.right_image_info_pub.publish(camera_info_msg1)
            self.tf_pub.publish(camera_tf_msg1)
            image_msg2, camera_info_msg2, camera_tf_msg2 = getImageMsg(data[2], self.spot_wrapper)
            self.left_depth_pub.publish(image_msg2)
            self.left_depth_info_pub.publish(camera_info_msg2)
            self.tf_pub.publish(camera_tf_msg2)
            image_msg3, camera_info_msg3, camera_tf_msg3 = getImageMsg(data[3], self.spot_wrapper)
            self.right_depth_pub.publish(image_msg3)
            self.right_depth_info_pub.publish(camera_info_msg3)
            self.tf_pub.publish(camera_tf_msg3)

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.rear_images
        if data:
            mage_msg0, camera_info_msg0, camera_tf_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.back_image_pub.publish(mage_msg0)
            self.back_image_info_pub.publish(camera_info_msg0)
            self.tf_pub.publish(camera_tf_msg0)
            mage_msg1, camera_info_msg1, camera_tf_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.back_depth_pub.publish(mage_msg1)
            self.back_depth_info_pub.publish(camera_info_msg1)
            self.tf_pub.publish(camera_tf_msg1)

    def handle_claim(self, req):
        """ROS service handler for the claim service"""
        resp = self.spot_wrapper.claim()
        return TriggerResponse(resp[0], resp[1])

    def handle_release(self, req):
        """ROS service handler for the release service"""
        resp = self.spot_wrapper.release()
        return TriggerResponse(resp[0], resp[1])

    def handle_stop(self, req):
        """ROS service handler for the stop service"""
        resp = self.spot_wrapper.stop()
        return TriggerResponse(resp[0], resp[1])

    def handle_self_right(self, req):
        """ROS service handler for the self-right service"""
        resp = self.spot_wrapper.self_right()
        return TriggerResponse(resp[0], resp[1])

    def handle_sit(self, req):
        """ROS service handler for the sit service"""
        resp = self.spot_wrapper.sit()
        return TriggerResponse(resp[0], resp[1])

    def handle_stand(self, req):
        """ROS service handler for the stand service"""
        resp = self.spot_wrapper.stand()
        return TriggerResponse(resp[0], resp[1])

    def handle_power_on(self, req):
        """ROS service handler for the power-on service"""
        resp = self.spot_wrapper.power_on()
        return TriggerResponse(resp[0], resp[1])

    def handle_safe_power_off(self, req):
        """ROS service handler for the safe-power-off service"""
        resp = self.spot_wrapper.safe_power_off()
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_hard(self, req):
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        resp = self.spot_wrapper.assertEStop(True)
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_soft(self, req):
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting power to the motors"""
        resp = self.spot_wrapper.assertEStop(False)
        return TriggerResponse(resp[0], resp[1])

    def cmdVelCallback(self, data):
        """Callback for cmd_vel command"""
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z)

    def bodyPoseCallback(self, data):
        """Callback for cmd_vel command"""
        q = Quaternion()
        q.x = data.orientation.x
        q.y = data.orientation.y
        q.z = data.orientation.z
        q.w = data.orientation.w

        euler_zxy = q.to_euler_zxy()
        self.spot_wrapper.set_mobility_params(data.position.z, euler_zxy)

    def shutdown(self):
        rospy.loginfo("Shutting down ROS driver for Spot")
        self.spot_wrapper.sit()
        rospy.Rate(0.25).sleep()
        self.spot_wrapper.disconnect()

    def main(self):
        """Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.  Holds lease from wrapper and updates all async tasks at the ROS rate"""
        rospy.init_node('spot_ros', anonymous=True)
        rate = rospy.Rate(50)

        self.rates = rospy.get_param('~rates', {})
        self.username = rospy.get_param('~username', 'default_value')
        self.password = rospy.get_param('~password', 'default_value')
        self.app_token = rospy.get_param('~app_token', 'default_value')
        self.hostname = rospy.get_param('~hostname', 'default_value')
        self.motion_deadzone = rospy.get_param('~deadzone', 0.05)

        self.logger = logging.getLogger('rosout')

        rospy.loginfo("Starting ROS driver for Spot")
        self.spot_wrapper = SpotWrapper(self.username, self.password, self.app_token, self.hostname, self.logger, self.rates, self.callbacks)

        if self.spot_wrapper.is_valid:
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

            # Status Publishers #
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

            self.feedback_pub = rospy.Publisher('status/feedback', Feedback, queue_size=10)

            rospy.Subscriber('cmd_vel', Twist, self.cmdVelCallback)
            rospy.Subscriber('body_pose', Pose, self.bodyPoseCallback)

            rospy.Service("claim", Trigger, self.handle_claim)
            rospy.Service("release", Trigger, self.handle_release)
            rospy.Service("stop", Trigger, self.handle_stop)
            rospy.Service("self_right", Trigger, self.handle_self_right)
            rospy.Service("sit", Trigger, self.handle_sit)
            rospy.Service("stand", Trigger, self.handle_stand)
            rospy.Service("power_on", Trigger, self.handle_power_on)
            rospy.Service("power_off", Trigger, self.handle_safe_power_off)

            rospy.Service("estop/hard", Trigger, self.handle_estop_hard)
            rospy.Service("estop/gentle", Trigger, self.handle_estop_soft)

            rospy.on_shutdown(self.shutdown)

            self.spot_wrapper.resetEStop()

            self.auto_claim = rospy.get_param('~auto_claim', False)
            self.auto_power_on = rospy.get_param('~auto_power_on', False)
            self.auto_stand = rospy.get_param('~auto_stand', False)

            if self.auto_claim:
                self.spot_wrapper.claim()
                if self.auto_power_on:
                    self.spot_wrapper.power_on()
                    if self.auto_stand:
                        self.spot_wrapper.stand()

            while not rospy.is_shutdown():
                self.spot_wrapper.updateTasks()
                feedback_msg = Feedback()
                feedback_msg.standing = self.spot_wrapper.is_standing
                feedback_msg.sitting = self.spot_wrapper.is_sitting
                feedback_msg.moving = self.spot_wrapper.is_moving
                id = self.spot_wrapper.id
                try:
                    feedback_msg.serial_number = id.serial_number
                    feedback_msg.species = id.species
                    feedback_msg.version = id.version
                    feedback_msg.nickname = id.nickname
                    feedback_msg.computer_serial_number = id.computer_serial_number
                except:
                    pass
                self.feedback_pub.publish(feedback_msg)
                rate.sleep()

if __name__ == "__main__":
    SR = SpotROS()
    SR.main()
