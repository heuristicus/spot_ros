import rospy

from std_msgs.msg import Empty
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

from bosdyn.api import image_pb2

friendly_joint_names = {}
"""Dictionary for mapping BD joint names to more friendly names"""
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

class DefaultCameraInfo(CameraInfo):
    """Blank class extending CameraInfo ROS topic that defaults most parameters"""
    def __init__(self):
        super().__init__()
        self.distortion_model = "plumb_bob"

        self.D.append(0)
        self.D.append(0)
        self.D.append(0)
        self.D.append(0)
        self.D.append(0)

        self.K[1] = 0
        self.K[3] = 0
        self.K[6] = 0
        self.K[7] = 0
        self.K[8] = 1

        self.R[0] = 1
        self.R[1] = 0
        self.R[2] = 0
        self.R[3] = 0
        self.R[4] = 1
        self.R[5] = 0
        self.R[6] = 0
        self.R[7] = 0
        self.R[8] = 1

        self.P[1] = 0
        self.P[3] = 0
        self.P[4] = 0
        self.P[7] = 0
        self.P[8] = 0
        self.P[9] = 0
        self.P[10] = 1
        self.P[11] = 0

def getImageMsg(data, spot_wrapper):
    """Takes the image, camera, and TF data and populates the necessary ROS messages

    Args:
        data: Image proto
        spot_wrapper: A SpotWrapper object
    Returns:
        (tuple):
            * Image: message of the image captured
            * CameraInfo: message to define the state and config of the camera that took the image
            * TFMessage: with the transforms necessary to locate the image frames
    """
    tf_msg = TFMessage()
    for frame_name in data.shot.transforms_snapshot.child_to_parent_edge_map:
        if data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
            transform = data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            new_tf = TransformStamped()
            local_time = spot_wrapper.robotToLocalTime(data.shot.acquisition_time)
            new_tf.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
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

    image_msg = Image()
    local_time = spot_wrapper.robotToLocalTime(data.shot.acquisition_time)
    image_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    image_msg.header.frame_id = data.shot.frame_name_image_sensor
    image_msg.height = data.shot.image.rows
    image_msg.width = data.shot.image.cols

    # Color/greyscale formats.
    # JPEG format
    if data.shot.image.format == image_pb2.Image.FORMAT_JPEG:
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = True
        image_msg.step = 3 * data.shot.image.cols
        image_msg.data = data.shot.image.data

    # Uncompressed.  Requires pixel_format.
    if data.shot.image.format == image_pb2.Image.FORMAT_RAW:
        # One byte per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            image_msg.encoding = "mono8"
            image_msg.is_bigendian = True
            image_msg.step = data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Three bytes per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = True
            image_msg.step = 3 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Four bytes per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            image_msg.encoding = "rgba8"
            image_msg.is_bigendian = True
            image_msg.step = 4 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Little-endian uint16 z-distance from camera (mm).
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            image_msg.encoding = "mono16"
            image_msg.is_bigendian = False
            image_msg.step = 2 * data.shot.image.cols
            image_msg.data = data.shot.image.data

    camera_info_msg = DefaultCameraInfo()
    local_time = spot_wrapper.robotToLocalTime(data.shot.acquisition_time)
    camera_info_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    camera_info_msg.header.frame_id = data.shot.frame_name_image_sensor
    camera_info_msg.height = data.shot.image.rows
    camera_info_msg.width = data.shot.image.cols

    camera_info_msg.K[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.K[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.K[4] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.K[5] = data.source.pinhole.intrinsics.principal_point.y

    camera_info_msg.P[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.P[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.P[5] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.P[6] = data.source.pinhole.intrinsics.principal_point.y

    return image_msg, camera_info_msg, tf_msg

def GetJointStatesFromState(state, spot_wrapper):
    """Maps joint state data from robot state proto to ROS JointState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        JointState message
    """
    joint_state = JointState()
    local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
    joint_state.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    for joint in state.kinematic_state.joint_states:
        joint_state.name.append(friendly_joint_names.get(joint.name, "ERROR"))
        joint_state.position.append(joint.position.value)
        joint_state.velocity.append(joint.velocity.value)
        joint_state.effort.append(joint.load.value)

    return joint_state

def GetEStopStateFromState(state, spot_wrapper):
    """Maps eStop state data from robot state proto to ROS EStopArray message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        EStopArray message
    """
    estop_array_msg = EStopStateArray()
    for estop in state.estop_states:
        estop_msg = EStopState()
        local_time = spot_wrapper.robotToLocalTime(estop.timestamp)
        estop_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
        estop_msg.name = estop.name
        estop_msg.type = estop.type
        estop_msg.state = estop.state
        estop_array_msg.estop_states.append(estop_msg)

    return estop_array_msg

def GetFeetFromState(state, spot_wrapper):
    """Maps foot position state data from robot state proto to ROS FootStateArray message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        FootStateArray message
    """
    foot_array_msg = FootStateArray()
    for foot in state.foot_state:
        foot_msg = FootState()
        foot_msg.foot_position_rt_body.x = foot.foot_position_rt_body.x
        foot_msg.foot_position_rt_body.y = foot.foot_position_rt_body.y
        foot_msg.foot_position_rt_body.z = foot.foot_position_rt_body.z
        foot_msg.contact = foot.contact
        foot_array_msg.states.append(foot_msg)

    return foot_array_msg

def GetOdomTwistFromState(state, spot_wrapper):
    """Maps odometry data from robot state proto to ROS TwistWithCovarianceStamped message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        TwistWithCovarianceStamped message
    """
    twist_odom_msg = TwistWithCovarianceStamped()
    local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
    twist_odom_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    twist_odom_msg.twist.twist.linear.x = state.kinematic_state.velocity_of_body_in_odom.linear.x
    twist_odom_msg.twist.twist.linear.y = state.kinematic_state.velocity_of_body_in_odom.linear.y
    twist_odom_msg.twist.twist.linear.z = state.kinematic_state.velocity_of_body_in_odom.linear.z
    twist_odom_msg.twist.twist.angular.x = state.kinematic_state.velocity_of_body_in_odom.angular.x
    twist_odom_msg.twist.twist.angular.y = state.kinematic_state.velocity_of_body_in_odom.angular.y
    twist_odom_msg.twist.twist.angular.z = state.kinematic_state.velocity_of_body_in_odom.angular.z
    return twist_odom_msg

def GetWifiFromState(state, spot_wrapper):
    """Maps wireless state data from robot state proto to ROS WiFiState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        WiFiState message
    """
    wifi_msg = WiFiState()
    for comm_state in state.comms_states:
        if comm_state.HasField('wifi_state'):
            wifi_msg.current_mode = comm_state.wifi_state.current_mode
            wifi_msg.essid = comm_state.wifi_state.essid

    return wifi_msg

def GetTFFromState(state, spot_wrapper):
    """Maps robot link state data from robot state proto to ROS TFMessage message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        TFMessage message
    """
    tf_msg = TFMessage()

    for frame_name in state.kinematic_state.transforms_snapshot.child_to_parent_edge_map:
        if state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
            transform = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            new_tf = TransformStamped()
            local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
            new_tf.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
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

    return tf_msg

def GetBatteryStatesFromState(state, spot_wrapper):
    """Maps battery state data from robot state proto to ROS BatteryStateArray message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BatteryStateArray message
    """
    battery_states_array_msg = BatteryStateArray()
    for battery in state.battery_states:
        battery_msg = BatteryState()
        local_time = spot_wrapper.robotToLocalTime(battery.timestamp)
        battery_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)

        battery_msg.identifier = battery.identifier
        battery_msg.charge_percentage = battery.charge_percentage.value
        battery_msg.estimated_runtime = rospy.Time(battery.estimated_runtime.seconds, battery.estimated_runtime.nanos)
        battery_msg.current = battery.current.value
        battery_msg.voltage = battery.voltage.value
        for temp in battery.temperatures:
            battery_msg.temperatures.append(temp)
        battery_msg.status = battery.status
        battery_states_array_msg.battery_states.append(battery_msg)

    return battery_states_array_msg

def GetPowerStatesFromState(state, spot_wrapper):
    """Maps power state data from robot state proto to ROS PowerState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        PowerState message
    """
    power_state_msg = PowerState()
    local_time = spot_wrapper.robotToLocalTime(state.power_state.timestamp)
    power_state_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    power_state_msg.motor_power_state = state.power_state.motor_power_state
    power_state_msg.shore_power_state = state.power_state.shore_power_state
    power_state_msg.locomotion_charge_percentage = state.power_state.locomotion_charge_percentage.value
    power_state_msg.locomotion_estimated_runtime = rospy.Time(state.power_state.locomotion_estimated_runtime.seconds, state.power_state.locomotion_estimated_runtime.nanos)
    return power_state_msg

def getBehaviorFaults(behavior_faults, spot_wrapper):
    """Helper function to strip out behavior faults into a list

    Args:
        behavior_faults: List of BehaviorFaults
        spot_wrapper: A SpotWrapper object
    Returns:
        List of BehaviorFault messages
    """
    faults = []

    for fault in behavior_faults:
        new_fault = BehaviorFault()
        new_fault.behavior_fault_id = fault.behavior_fault_id
        local_time = spot_wrapper.robotToLocalTime(fault.onset_timestamp)
        new_fault.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
        new_fault.cause = fault.cause
        new_fault.status = fault.status
        faults.append(new_fault)

    return faults

def getSystemFaults(system_faults, spot_wrapper):
    """Helper function to strip out system faults into a list

    Args:
        systen_faults: List of SystemFaults
        spot_wrapper: A SpotWrapper object
    Returns:
        List of SystemFault messages
    """
    faults = []

    for fault in system_faults:
        new_fault = SystemFault()
        new_fault.name = fault.name
        local_time = spot_wrapper.robotToLocalTime(fault.onset_timestamp)
        new_fault.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
        new_fault.duration = rospy.Time(fault.duration.seconds, fault.duration.nanos)
        new_fault.code = fault.code
        new_fault.uid = fault.uid
        new_fault.error_message = fault.error_message

        for att in fault.attributes:
            new_fault.attributes.append(att)

        new_fault.severity = fault.severity
        faults.append(new_fault)

    return faults

def GetSystemFaultsFromState(state, spot_wrapper):
    """Maps system fault data from robot state proto to ROS SystemFaultState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        SystemFaultState message
    """
    system_fault_state_msg = SystemFaultState()
    system_fault_state_msg.faults = getSystemFaults(state.system_fault_state.faults, spot_wrapper)
    system_fault_state_msg.historical_faults = getSystemFaults(state.system_fault_state.historical_faults, spot_wrapper)
    return system_fault_state_msg

def getBehaviorFaultsFromState(state, spot_wrapper):
    """Maps behavior fault data from robot state proto to ROS BehaviorFaultState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BehaviorFaultState message
    """
    behavior_fault_state_msg = BehaviorFaultState()
    behavior_fault_state_msg.faults = getBehaviorFaults(state.behavior_fault_state.faults, spot_wrapper)
    return behavior_fault_state_msg
