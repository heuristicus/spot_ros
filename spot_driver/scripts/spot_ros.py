#!/usr/bin/env python3
import rospy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from spot_wrapper import SpotWrapper
import logging

SW = None

back_fisheye_image_pub = rospy.Publisher('back_fisheye_image', Image, queue_size=10)
frontleft_fisheye_image_pub = rospy.Publisher('frontleft_fisheye_image', Image, queue_size=10)
frontright_fisheye_image_pub = rospy.Publisher('frontright_fisheye_image', Image, queue_size=10)
left_fisheye_image_pub = rospy.Publisher('left_fisheye_image', Image, queue_size=10)
right_fisheye_image_pub = rospy.Publisher('right_fisheye_image', Image, queue_size=10)

joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
tf_pub = rospy.Publisher('tf', TFMessage, queue_size=10)

friendly_joint_names = {}
friendly_joint_names["fl.hx"] = "front_left_hip_x"
friendly_joint_names["fl.hy"] = "front_left_hip_y"
friendly_joint_names["fl.kn"] = "front_left_knee"
friendly_joint_names["fr.hx"] = "front_right_hip_x"
friendly_joint_names["fr.hy"] = "front_right_hip_y"
friendly_joint_names["fr.kn"] = "front_right_knee"
friendly_joint_names["hl.hx"] = "rear_left_hip_x"
friendly_joint_names["hl.hy"] = "rear_left_hip_y"
friendly_joint_names["hl.kn"] = "rear_left_knee"
friendly_joint_names["hr.hx"] = "rear_right_hip_x"
friendly_joint_names["hr.hy"] = "rear_right_hip_y"
friendly_joint_names["hr.kn"] = "rear_right_knee"

def RobotStateCB(results):
    state = SW.robot_state

    if state:
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        for joint in state.kinematic_state.joint_states:
            joint_state.name.append(friendly_joint_names.get(joint.name, "ERROR"))
            joint_state.position.append(joint.position.value)
            joint_state.velocity.append(joint.velocity.value)
            joint_state.effort.append(joint.load.value)

        joint_state_pub.publish(joint_state)

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
            tf_pub.publish(tf_msg)

def MetricsCB(results):
    rospy.logdebug("##### METRICS #####")
    #rospy.loginfo(str(SW.metrics))

def RobotCommandCB(results):
    rospy.logdebug("##### COMMAND #####")
    #rospy.loginfo(str(SW.robot_command))

def PowerCB(results):
    rospy.logdebug("##### POWER #####")
    #rospy.loginfo(str(SW.power))

def LeaseCB(results):
    rospy.logdebug("##### LEASE #####")
    #rospy.loginfo(str(SW.lease))

def getImageMsg(data):
    msg = Image()
    #Header header
    msg.height = data.shot.image.rows
    msg.width = data.shot.image.cols
    #string encoding
    #uint8 is_bigendian
    #uint32 step
    msg.data = data.shot.image.data

def ImageCB(results):
    data = SW.image
    #if data:
    #    back_fisheye_image_pub.publish(getImageMsg(data[0]))
    #    frontleft_fisheye_image.publish(getImageMsg(data[1]))
    #    frontright_fisheye_image.publish(getImageMsg(data[2]))
    #    left_fisheye_image.publish(getImageMsg(data[3]))
    #    right_fisheye_image.publish(getImageMsg(data[4]))

def EstopCB(results):
    rospy.logdebug("##### ESTOP #####")

callbacks = {}
callbacks["robot_state"] = RobotStateCB
callbacks["metrics"] = MetricsCB
callbacks["robot_command"] = RobotCommandCB
callbacks["power"] = PowerCB
callbacks["lease"] = LeaseCB
callbacks["image"] = ImageCB
callbacks["estop"] = EstopCB

rospy.init_node('spot_ros', anonymous=True)
rate = rospy.Rate(50)

rates = rospy.get_param('~rates', {})
username = rospy.get_param('~username', 'default_value')
password = rospy.get_param('~password', 'default_value')
app_token = rospy.get_param('~app_token', 'default_value')
hostname = rospy.get_param('~hostname', 'default_value')

logger = logging.getLogger('rosout')

rospy.loginfo("Starting")
SW = SpotWrapper(username, password, app_token, hostname, logger, rates, callbacks)

if SW._robot:
    rospy.loginfo("Connecting")
    SW.connect()
    rospy.loginfo("Running")
    with SW.getLease():
        while not rospy.is_shutdown():
            SW.updateTasks()
            rate.sleep()
