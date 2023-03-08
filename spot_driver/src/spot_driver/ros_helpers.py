import typing
import rospy

from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import TransformStamped, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray
from spot_msgs.msg import DockState
from spot_msgs.srv import SpotCheckRequest, SpotCheckResponse

from bosdyn.api import image_pb2, robot_state_pb2
from bosdyn.api.docking import docking_pb2
from bosdyn.client.spot_check import spot_check_pb2
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.frame_helpers import get_odom_tform_body, get_vision_tform_body

from .spot_wrapper import SpotWrapper

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

# arm joints
friendly_joint_names["arm0.sh0"] = "arm_joint1"
friendly_joint_names["arm0.sh1"] = "arm_joint2"
friendly_joint_names["arm0.el0"] = "arm_joint3"
friendly_joint_names["arm0.el1"] = "arm_joint4"
friendly_joint_names["arm0.wr0"] = "arm_joint5"
friendly_joint_names["arm0.wr1"] = "arm_joint6"
friendly_joint_names["arm0.f1x"] = "arm_gripper"


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


def populateTransformStamped(
    time: rospy.Time, parent_frame: str, child_frame: str, transform: Transform
) -> TransformStamped:
    """Populates a TransformStamped message

    Args:
        time: The time of the transform
        parent_frame: The parent frame of the transform
        child_frame: The child_frame_id of the transform
        transform: A transform to copy into a StampedTransform object. Should have position (x,y,z) and rotation (x,
        y,z,w) members
    Returns:
        TransformStamped message. Empty if transform does not have position or translation attribute
    """
    if hasattr(transform, "position"):
        position = transform.position
    elif hasattr(transform, "translation"):
        position = transform.translation
    else:
        rospy.logerr(
            "Trying to generate StampedTransform but input transform has neither position nor translation "
            "attributes"
        )
        return TransformStamped()

    new_tf = TransformStamped()
    new_tf.header.stamp = time
    new_tf.header.frame_id = parent_frame
    new_tf.child_frame_id = child_frame
    new_tf.transform.translation.x = position.x
    new_tf.transform.translation.y = position.y
    new_tf.transform.translation.z = position.z
    new_tf.transform.rotation.x = transform.rotation.x
    new_tf.transform.rotation.y = transform.rotation.y
    new_tf.transform.rotation.z = transform.rotation.z
    new_tf.transform.rotation.w = transform.rotation.w

    return new_tf


def getImageMsg(
    data: image_pb2.ImageResponse, spot_wrapper: SpotWrapper
) -> typing.Tuple[Image, CameraInfo]:
    """Takes the image and camera data and populates the necessary ROS messages

    Args:
        data: Image proto
        spot_wrapper: A SpotWrapper object
    Returns:
        (tuple):
            * Image: message of the image captured
            * CameraInfo: message to define the state and config of the camera that took the image
    """
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
            image_msg.encoding = "16UC1"
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

    return image_msg, camera_info_msg


def GetJointStatesFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
):
    """Maps joint state data from robot state proto to ROS JointState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        JointState message
    """
    joint_state = JointState()
    local_time = spot_wrapper.robotToLocalTime(
        state.kinematic_state.acquisition_timestamp
    )
    joint_state.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    for joint in state.kinematic_state.joint_states:
        # there is a joint with name arm0.hr0 in the robot state, however this
        # joint has no data and should not be there, this is why we ignore it
        if joint.name == "arm0.hr0":
            continue
        joint_state.name.append(friendly_joint_names.get(joint.name, "ERROR"))
        joint_state.position.append(joint.position.value)
        joint_state.velocity.append(joint.velocity.value)
        joint_state.effort.append(joint.load.value)

    return joint_state


def GetEStopStateFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> EStopStateArray:
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
        estop_msg.state_description = estop.state_description
        estop_array_msg.estop_states.append(estop_msg)

    return estop_array_msg


def GetFeetFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> FootStateArray:
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

        if foot.HasField("terrain"):
            terrain = foot.terrain
            foot_msg.terrain.ground_mu_est = terrain.ground_mu_est
            foot_msg.terrain.frame_name = terrain.frame_name
            foot_msg.terrain.foot_slip_distance_rt_frame = (
                terrain.foot_slip_distance_rt_frame
            )
            foot_msg.terrain.foot_slip_velocity_rt_frame = (
                terrain.foot_slip_velocity_rt_frame
            )
            foot_msg.terrain.ground_contact_normal_rt_frame = (
                terrain.ground_contact_normal_rt_frame
            )
            foot_msg.terrain.visual_surface_ground_penetration_mean = (
                terrain.visual_surface_ground_penetration_mean
            )
            foot_msg.terrain.visual_surface_ground_penetration_std = (
                terrain.visual_surface_ground_penetration_std
            )

        foot_array_msg.states.append(foot_msg)

    return foot_array_msg


def GetOdomTwistFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> TwistWithCovarianceStamped:
    """Maps odometry data from robot state proto to ROS TwistWithCovarianceStamped message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        TwistWithCovarianceStamped message
    """
    twist_odom_msg = TwistWithCovarianceStamped()
    local_time = spot_wrapper.robotToLocalTime(
        state.kinematic_state.acquisition_timestamp
    )
    twist_odom_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    twist_odom_msg.twist.twist.linear.x = (
        state.kinematic_state.velocity_of_body_in_odom.linear.x
    )
    twist_odom_msg.twist.twist.linear.y = (
        state.kinematic_state.velocity_of_body_in_odom.linear.y
    )
    twist_odom_msg.twist.twist.linear.z = (
        state.kinematic_state.velocity_of_body_in_odom.linear.z
    )
    twist_odom_msg.twist.twist.angular.x = (
        state.kinematic_state.velocity_of_body_in_odom.angular.x
    )
    twist_odom_msg.twist.twist.angular.y = (
        state.kinematic_state.velocity_of_body_in_odom.angular.y
    )
    twist_odom_msg.twist.twist.angular.z = (
        state.kinematic_state.velocity_of_body_in_odom.angular.z
    )
    return twist_odom_msg


def GetOdomFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper, use_vision=True
):
    """Maps odometry data from robot state proto to ROS Odometry message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        Odometry message
    """
    odom_msg = Odometry()
    local_time = spot_wrapper.robotToLocalTime(
        state.kinematic_state.acquisition_timestamp
    )
    odom_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)
    if use_vision == True:
        odom_msg.header.frame_id = "vision"
        tform_body = get_vision_tform_body(state.kinematic_state.transforms_snapshot)
    else:
        odom_msg.header.frame_id = "odom"
        tform_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
    odom_msg.child_frame_id = "body"
    pose_odom_msg = PoseWithCovariance()
    pose_odom_msg.pose.position.x = tform_body.position.x
    pose_odom_msg.pose.position.y = tform_body.position.y
    pose_odom_msg.pose.position.z = tform_body.position.z
    pose_odom_msg.pose.orientation.x = tform_body.rotation.x
    pose_odom_msg.pose.orientation.y = tform_body.rotation.y
    pose_odom_msg.pose.orientation.z = tform_body.rotation.z
    pose_odom_msg.pose.orientation.w = tform_body.rotation.w

    odom_msg.pose = pose_odom_msg
    twist_odom_msg = GetOdomTwistFromState(state, spot_wrapper).twist
    odom_msg.twist = twist_odom_msg
    return odom_msg


def GetWifiFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> WiFiState:
    """Maps wireless state data from robot state proto to ROS WiFiState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        WiFiState message
    """
    wifi_msg = WiFiState()
    for comm_state in state.comms_states:
        if comm_state.HasField("wifi_state"):
            wifi_msg.current_mode = comm_state.wifi_state.current_mode
            wifi_msg.essid = comm_state.wifi_state.essid

    return wifi_msg


def GenerateFeetTF(foot_states_msg: FootStateArray) -> TFMessage:
    """
    Generate a tf message containing information about foot states

    Args:
        foot_states_msg: FootStateArray message containing the foot states from the robot state

    Returns: tf message with foot states

    """
    foot_ordering = ["front_left", "front_right", "rear_left", "rear_right"]
    foot_tfs = TFMessage()
    time_now = rospy.Time.now()
    for idx, foot_state in enumerate(foot_states_msg.states):
        foot_transform = Transform()
        # Rotation of the foot is not given
        foot_transform.rotation.w = 1
        foot_transform.translation.x = foot_state.foot_position_rt_body.x
        foot_transform.translation.y = foot_state.foot_position_rt_body.y
        foot_transform.translation.z = foot_state.foot_position_rt_body.z

        foot_tfs.transforms.append(
            populateTransformStamped(
                time_now, "body", foot_ordering[idx] + "_foot", foot_transform
            )
        )

    return foot_tfs


def GetTFFromState(
    state: robot_state_pb2.RobotState,
    spot_wrapper: SpotWrapper,
    inverse_target_frame: str,
) -> TFMessage:
    """Maps robot link state data from robot state proto to ROS TFMessage message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
        inverse_target_frame: A frame name to be inversed to a parent frame.
    Returns:
        TFMessage message
    """
    tf_msg = TFMessage()

    for (
        frame_name
    ) in state.kinematic_state.transforms_snapshot.child_to_parent_edge_map:
        if state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(
            frame_name
        ).parent_frame_name:
            try:
                transform = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(
                    frame_name
                )
                local_time = spot_wrapper.robotToLocalTime(
                    state.kinematic_state.acquisition_timestamp
                )
                tf_time = rospy.Time(local_time.seconds, local_time.nanos)
                if inverse_target_frame == frame_name:
                    geo_tform_inversed = SE3Pose.from_obj(
                        transform.parent_tform_child
                    ).inverse()
                    new_tf = populateTransformStamped(
                        tf_time,
                        frame_name,
                        transform.parent_frame_name,
                        geo_tform_inversed,
                    )
                else:
                    new_tf = populateTransformStamped(
                        tf_time,
                        transform.parent_frame_name,
                        frame_name,
                        transform.parent_tform_child,
                    )
                tf_msg.transforms.append(new_tf)
            except Exception as e:
                spot_wrapper.logger.error("Error: {}".format(e))

    return tf_msg


def GetBatteryStatesFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> BatteryStateArray:
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
        battery_msg.estimated_runtime = rospy.Time(
            battery.estimated_runtime.seconds, battery.estimated_runtime.nanos
        )
        battery_msg.current = battery.current.value
        battery_msg.voltage = battery.voltage.value
        for temp in battery.temperatures:
            battery_msg.temperatures.append(temp)
        battery_msg.status = battery.status
        battery_states_array_msg.battery_states.append(battery_msg)

    return battery_states_array_msg


def GetPowerStatesFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> PowerState:
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
    power_state_msg.locomotion_charge_percentage = (
        state.power_state.locomotion_charge_percentage.value
    )
    power_state_msg.locomotion_estimated_runtime = rospy.Time(
        state.power_state.locomotion_estimated_runtime.seconds,
        state.power_state.locomotion_estimated_runtime.nanos,
    )
    return power_state_msg


def GetDockStatesFromState(state: docking_pb2.DockState) -> DockState:
    """Maps dock state data from robot state proto to ROS DockState message

    Args:
        state: Robot State proto
    Returns:
        DockState message
    """
    dock_state_msg = DockState()
    dock_state_msg.status = state.status
    dock_state_msg.dock_type = state.dock_type
    dock_state_msg.dock_id = state.dock_id
    dock_state_msg.power_status = state.power_status
    return dock_state_msg


def GetBehaviorFaults(
    behavior_faults: typing.List[robot_state_pb2.BehaviorFault],
    spot_wrapper: SpotWrapper,
) -> typing.List[BehaviorFault]:
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


def GetBehaviorFaultsFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> BehaviorFaultState:
    """Maps behavior fault data from robot state proto to ROS BehaviorFaultState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BehaviorFaultState message
    """
    behavior_fault_state_msg = BehaviorFaultState()
    behavior_fault_state_msg.faults = GetBehaviorFaults(
        state.behavior_fault_state.faults, spot_wrapper
    )
    return behavior_fault_state_msg


def GetSystemFaults(
    system_faults: typing.List[robot_state_pb2.SystemFault], spot_wrapper: SpotWrapper
) -> typing.List[SystemFault]:
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


def GetSystemFaultsFromState(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> SystemFaultState:
    """Maps system fault data from robot state proto to ROS SystemFaultState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        SystemFaultState message
    """
    system_fault_state_msg = SystemFaultState()
    system_fault_state_msg.faults = GetSystemFaults(
        state.system_fault_state.faults, spot_wrapper
    )
    system_fault_state_msg.historical_faults = GetSystemFaults(
        state.system_fault_state.historical_faults, spot_wrapper
    )
    return system_fault_state_msg


def GetSpotCheckResultsMsg(
    data: spot_check_pb2.SpotCheckFeedbackResponse, resp: typing.Tuple[bool, str]
) -> SpotCheckResponse:
    """Build the SpotCheckReponse message from the SpotCheckFeedbackResponse"""
    ros_resp = SpotCheckResponse()
    ros_resp.success = resp[0]
    ros_resp.message = resp[1]

    ros_resp.camera_names = list(data.camera_results.keys())
    ros_resp.camera_results = list(data.camera_results.values())
    ros_resp.load_cell_names = list(data.load_cell_results.keys())
    ros_resp.load_cell_results = list(data.load_cell_results.values())
    ros_resp.kinematic_joint_names = list(data.kinematic_cal_results.keys())
    ros_resp.kinematic_cal_results = list(data.kinematic_cal_results.values())
    ros_resp.payload_result = data.payload_result
    ros_resp.leg_names = list(data.hip_range_of_motion_results.keys())
    ros_resp.hip_range_of_motion_results = list(
        data.hip_range_of_motion_results.values()
    )
    ros_resp.progress = data.progress
    ros_resp.last_cal_timestamp.secs = data.last_cal_timestamp.seconds
    ros_resp.last_cal_timestamp.nsecs = data.last_cal_timestamp.nanos

    return ros_resp
