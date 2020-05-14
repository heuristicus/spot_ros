#!/usr/bin/env python3
import rospy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
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

from spot_wrapper import SpotWrapper
import logging

class SpotROS():
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self):
        self.spot_wrapper = None

        self.friendly_joint_names = {}
        """Dictionary for mapping BD joint names to more friendly names"""
        self.friendly_joint_names["fl.hx"] = "front_left_hip_x"
        self.friendly_joint_names["fl.hy"] = "front_left_hip_y"
        self.friendly_joint_names["fl.kn"] = "front_left_knee"
        self.friendly_joint_names["fr.hx"] = "front_right_hip_x"
        self.friendly_joint_names["fr.hy"] = "front_right_hip_y"
        self.friendly_joint_names["fr.kn"] = "front_right_knee"
        self.friendly_joint_names["hl.hx"] = "rear_left_hip_x"
        self.friendly_joint_names["hl.hy"] = "rear_left_hip_y"
        self.friendly_joint_names["hl.kn"] = "rear_left_knee"
        self.friendly_joint_names["hr.hx"] = "rear_right_hip_x"
        self.friendly_joint_names["hr.hy"] = "rear_right_hip_y"
        self.friendly_joint_names["hr.kn"] = "rear_right_knee"

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
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time(state.kinematic_state.acquisition_timestamp.seconds, state.kinematic_state.acquisition_timestamp.nanos)
            for joint in state.kinematic_state.joint_states:
                joint_state.name.append(self.friendly_joint_names.get(joint.name, "ERROR"))
                joint_state.position.append(joint.position.value)
                joint_state.velocity.append(joint.velocity.value)
                joint_state.effort.append(joint.load.value)

            self.joint_state_pub.publish(joint_state)

            ## TF ##
            tf_msg = TFMessage()
            for frame_name in state.kinematic_state.transforms_snapshot.child_to_parent_edge_map:
                if state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
                    transform = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
                    new_tf = TransformStamped()
                    new_tf.header.stamp = rospy.Time(state.kinematic_state.acquisition_timestamp.seconds, state.kinematic_state.acquisition_timestamp.nanos)
                    new_tf.header.frame_id = transform.parent_frame_name
                    new_tf.child_frame_id = frame_name
                    new_tf.transform.translation.x = transform.parent_tform_child.position.x
                    new_tf.transform.translation.y = transform.parent_tform_child.position.y
                    new_tf.transform.translation.z = transform.parent_tform_child.position.z
                    new_tf.transform.rotation.x = transform.parent_tform_child.rotation.x
                    new_tf.transform.rotation.y = transform.parent_tform_child.rotation.y
                    new_tf.transform.rotation.z = transform.parent_tform_child.rotation.z
                    new_tf.transform.rotation.w = transform.parent_tform_child.rotation.w
                    tf_msg.transforms.append(new_tf)
            if len(tf_msg.transforms) > 0:
                self.tf_pub.publish(tf_msg)

            # Odom Twist #
            twist_odom_msg = TwistWithCovarianceStamped()
            twist_odom_msg.header.stamp = rospy.Time(state.kinematic_state.acquisition_timestamp.seconds, state.kinematic_state.acquisition_timestamp.nanos)
            twist_odom_msg.twist.twist.linear.x = state.kinematic_state.velocity_of_body_in_odom.linear.x
            twist_odom_msg.twist.twist.linear.y = state.kinematic_state.velocity_of_body_in_odom.linear.y
            twist_odom_msg.twist.twist.linear.z = state.kinematic_state.velocity_of_body_in_odom.linear.z
            twist_odom_msg.twist.twist.angular.x = state.kinematic_state.velocity_of_body_in_odom.angular.x
            twist_odom_msg.twist.twist.angular.y = state.kinematic_state.velocity_of_body_in_odom.angular.y
            twist_odom_msg.twist.twist.angular.z = state.kinematic_state.velocity_of_body_in_odom.angular.z

            self.odom_twist_pub.publish(twist_odom_msg)

            # Feet #
            foot_array_msg = FootStateArray()
            for foot in state.foot_state:
                foot_msg = FootState()
                foot_msg.foot_position_rt_body.x = foot.foot_position_rt_body.x
                foot_msg.foot_position_rt_body.y = foot.foot_position_rt_body.y
                foot_msg.foot_position_rt_body.z = foot.foot_position_rt_body.z
                foot_msg.contact = foot.contact
                foot_array_msg.states.append(foot_msg)

            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = EStopStateArray()
            for estop in state.estop_states:
                estop_msg = EStopState()
                estop_msg.header.stamp = rospy.Time(estop.timestamp.seconds, estop.timestamp.nanos)
                estop_msg.name = estop.name
                estop_msg.type = estop.type
                estop_msg.state = estop.state
                estop_array_msg.estop_states.append(estop_msg)

            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = WiFiState()
            for comm_state in state.comms_states:
                if comm_state.HasField('wifi_state'):
                    wifi_msg.current_mode = comm_state.wifi_state.current_mode
                    wifi_msg.essid = comm_state.wifi_state.essid
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = BatteryStateArray()
            for battery in state.battery_states:
                battery_msg = BatteryState()
                battery_msg.header.stamp = rospy.Time(battery.timestamp.seconds, battery.timestamp.nanos)

                battery_msg.identifier = battery.identifier
                battery_msg.charge_percentage = battery.charge_percentage.value
                battery_msg.estimated_runtime = rospy.Time(battery.estimated_runtime.seconds, battery.estimated_runtime.nanos)
                battery_msg.current = battery.current.value
                battery_msg.voltage = battery.voltage.value
                for temp in battery.temperatures:
                    battery_msg.temperatures.append(temp)
                battery_msg.status = battery.status
                battery_states_array_msg.battery_states.append(battery_msg)

            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = PowerState()
            power_state_msg.header.stamp = rospy.Time(state.power_state.timestamp.seconds, state.power_state.timestamp.nanos)
            power_state_msg.motor_power_state = state.power_state.motor_power_state
            power_state_msg.shore_power_state = state.power_state.shore_power_state
            power_state_msg.locomotion_charge_percentage = state.power_state.locomotion_charge_percentage.value
            power_state_msg.locomotion_estimated_runtime = rospy.Time(state.power_state.locomotion_estimated_runtime.seconds, state.power_state.locomotion_estimated_runtime.nanos)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = SystemFaultState()
            system_fault_state_msg.faults = self.getSystemFaults(state.system_fault_state.faults)
            system_fault_state_msg.historical_faults = self.getSystemFaults(state.system_fault_state.historical_faults)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = BehaviorFaultState()
            behavior_fault_state_msg.faults = self.getBehaviorFaults(state.behavior_fault_state.faults)
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

    def getImageMsg(self, data):
        """Maps image data from image proto to ROS image message

        Args:
            data: Image proto
        """
        msg = Image()
        msg.header.stamp = rospy.Time(data.shot.acquisition_time.seconds, data.shot.acquisition_time.nanos)
        msg.header.frame_id = data.shot.frame_name_image_sensor
        msg.height = data.shot.image.rows
        msg.width = data.shot.image.cols
        msg.data = data.shot.image.data

        # Color/greyscale formats.
        # JPEG format
        if data.shot.image.format == 1:
            msg.encoding = "rgb8"
            msg.is_bigendian = True
            msg.step = 3 * data.shot.image.cols

        # Uncompressed.  Requires pixel_format.
        if data.shot.image.format == 2:
            # One byte per pixel.
            if data.shot.image.pixel_format == 1:
                msg.encoding = "mono8"
                msg.is_bigendian = True
                msg.step = data.shot.image.cols

            # Three bytes per pixel.
            if data.shot.image.pixel_format == 3:
                msg.encoding = "rgb8"
                msg.is_bigendian = True
                msg.step = 3 * data.shot.image.cols

            # Four bytes per pixel.
            if data.shot.image.pixel_format == 4:
                msg.encoding = "rgba8"
                msg.is_bigendian = True
                msg.step = 4 * data.shot.image.cols

            # Little-endian uint16 z-distance from camera (mm).
            if data.shot.image.pixel_format == 5:
                msg.encoding = "mono16"
                msg.is_bigendian = False
                msg.step = 2 * data.shot.image.cols

        return msg

    def FrontImageCB(self, results):
        """Callback for when the Spot Wrapper gets new front image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.front_images
        if data:
            self.frontleft_fisheye_image_pub.publish(self.getImageMsg(data[0]))
            self.frontright_fisheye_image_pub.publish(self.getImageMsg(data[1]))
            self.frontleft_fisheye_depth_pub.publish(self.getImageMsg(data[2]))
            self.frontright_fisheye_depth_pub.publish(self.getImageMsg(data[3]))

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.side_images
        if data:
            self.left_fisheye_image_pub.publish(self.getImageMsg(data[0]))
            self.right_fisheye_image_pub.publish(self.getImageMsg(data[1]))
            self.left_fisheye_depth_pub.publish(self.getImageMsg(data[2]))
            self.right_fisheye_depth_pub.publish(self.getImageMsg(data[3]))

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.rear_images
        if data:
            self.back_fisheye_image_pub.publish(self.getImageMsg(data[0]))
            self.back_fisheye_depth_pub.publish(self.getImageMsg(data[1]))

    def EstopCB(self, results):
        """Callback for when the Spot Wrapper gets new estop data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        rospy.logdebug("##### ESTOP #####")

    def getBehaviorFaults(self, behavior_faults):
        """Helper function to strip out behavior faults into a list

        Args:
            behavior_faults: List of BehaviorFaults
        """
        faults = []

        for fault in behavior_faults:
            new_fault = BehaviorFault()
            new_fault.behavior_fault_id = fault.behavior_fault_id
            new_fault.header.stamp = rospy.Time(fault.onset_timestamp.seconds, fault.onset_timestamp.nanos)
            new_fault.cause = fault.cause
            new_fault.status = fault.status
            faults.append(new_fault)

        return faults

    def getSystemFaults(self, system_faults):
        """Helper function to strip out system faults into a list

        Args:
            systen_faults: List of SystemFaults
        """
        faults = []

        for fault in system_faults:
            new_fault = SystemFault()
            new_fault.name = fault.name
            new_fault.header.stamp = rospy.Time(fault.onset_timestamp.seconds, fault.onset_timestamp.nanos)
            new_fault.duration = rospy.Time(fault.duration.seconds, fault.duration.nanos)
            new_fault.code = fault.code
            new_fault.uid = fault.uid
            new_fault.error_message = fault.error_message

            for att in fault.attributes:
                new_fault.attributes.append(att)

            new_fault.severity = fault.severity
            faults.append(new_fault)

        return faults

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
            self.back_fisheye_image_pub = rospy.Publisher('camera/back_fisheye_image', Image, queue_size=10)
            self.frontleft_fisheye_image_pub = rospy.Publisher('camera/frontleft_fisheye_image', Image, queue_size=10)
            self.frontright_fisheye_image_pub = rospy.Publisher('camera/frontright_fisheye_image', Image, queue_size=10)
            self.left_fisheye_image_pub = rospy.Publisher('camera/left_fisheye_image', Image, queue_size=10)
            self.right_fisheye_image_pub = rospy.Publisher('camera/right_fisheye_image', Image, queue_size=10)
            # Depth #
            self.back_fisheye_depth_pub = rospy.Publisher('depth/back_depth_in_visual_frame', Image, queue_size=10)
            self.frontleft_fisheye_depth_pub = rospy.Publisher('depth/frontleft_depth_in_visual_frame', Image, queue_size=10)
            self.frontright_fisheye_depth_pub = rospy.Publisher('depth/frontright_depth_in_visual_frame', Image, queue_size=10)
            self.left_fisheye_depth_pub = rospy.Publisher('depth/left_depth_in_visual_frame', Image, queue_size=10)
            self.right_fisheye_depth_pub = rospy.Publisher('depth/right_depth_in_visual_frame', Image, queue_size=10)

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
