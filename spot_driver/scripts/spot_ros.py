#!/usr/bin/env python3
import rospy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray, LeaseResource

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
        self.callbacks["image"] = self.ImageCB
        self.callbacks["estop"] = self.EstopCB

    def RobotStateCB(self, results):
        """Callback for when the Spot Wrapper gets new robot state data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        state = self.spot_wrapper.robot_state

        if state:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            for joint in state.kinematic_state.joint_states:
                joint_state.name.append(self.friendly_joint_names.get(joint.name, "ERROR"))
                joint_state.position.append(joint.position.value)
                joint_state.velocity.append(joint.velocity.value)
                joint_state.effort.append(joint.load.value)

            self.joint_state_pub.publish(joint_state)

            tf_msg = TFMessage()
            for frame_name in state.kinematic_state.transforms_snapshot.child_to_parent_edge_map:
                if state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
                    transform = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
                    new_tf = TransformStamped()
                    new_tf.header.stamp = rospy.Time.now()
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
            rospy.logwarn(lease_list)
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

    def getImageMsg(data):
        """Maps image data from image proto to ROS image message

        Args:
            data: Image proto
        """
        msg = Image()
        #Header header
        msg.height = data.shot.image.rows
        msg.width = data.shot.image.cols
        #string encoding
        #uint8 is_bigendian
        #uint32 step
        msg.data = data.shot.image.data

    def ImageCB(self, results):
        """Callback for when the Spot Wrapper gets new image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        # TODO: All of this
        pass
        #data = self.spot_wrapper.image
        #if data:
        #    back_fisheye_image_pub.publish(getImageMsg(data[0]))
        #    frontleft_fisheye_image.publish(getImageMsg(data[1]))
        #    frontright_fisheye_image.publish(getImageMsg(data[2]))
        #    left_fisheye_image.publish(getImageMsg(data[3]))
        #    right_fisheye_image.publish(getImageMsg(data[4]))

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
            self.back_fisheye_image_pub = rospy.Publisher('back_fisheye_image', Image, queue_size=10)
            self.frontleft_fisheye_image_pub = rospy.Publisher('frontleft_fisheye_image', Image, queue_size=10)
            self.frontright_fisheye_image_pub = rospy.Publisher('frontright_fisheye_image', Image, queue_size=10)
            self.left_fisheye_image_pub = rospy.Publisher('left_fisheye_image', Image, queue_size=10)
            self.right_fisheye_image_pub = rospy.Publisher('right_fisheye_image', Image, queue_size=10)

            self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
            """Defining a TF publisher manually because of conflicts between Python3 and tf"""
            self.tf_pub = rospy.Publisher('tf', TFMessage, queue_size=10)
            self.metrics_pub = rospy.Publisher('metrics', Metrics, queue_size=10)
            self.lease_pub = rospy.Publisher('leases', LeaseArray, queue_size=10)

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
