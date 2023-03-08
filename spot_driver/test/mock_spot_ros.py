#!/usr/bin/env python3
import rospy
import typing

from std_srvs.srv import TriggerResponse, SetBool, SetBoolResponse
from spot_msgs.srv import PosedStandResponse, PosedStandRequest
from spot_msgs.srv import ClearBehaviorFaultResponse
from spot_msgs.srv import SetLocomotionResponse
from spot_msgs.srv import SetSwingHeightResponse
from spot_msgs.srv import SetVelocityResponse
from spot_msgs.msg import MobilityParams
from spot_msgs.msg import NavigateToGoal, NavigateToResult
from spot_msgs.msg import TrajectoryResult, TrajectoryGoal
from spot_msgs.msg import PoseBodyAction, PoseBodyGoal, PoseBodyResult
from spot_msgs.srv import ListGraphResponse
from spot_msgs.srv import DockResponse, GetDockStateResponse
from spot_msgs.srv import GripperAngleMoveResponse, GripperAngleMoveRequest
from spot_msgs.srv import ArmJointMovementResponse, ArmJointMovementRequest
from spot_msgs.srv import ArmForceTrajectoryResponse
from spot_msgs.srv import HandPoseResponse, HandPoseRequest
from spot_msgs.srv import SpotCheckRequest, SpotCheckResponse, SpotCheck

from bosdyn.api import image_pb2, robot_state_pb2, lease_pb2, geometry_pb2
from bosdyn.api.docking import docking_pb2
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.robot_command import RobotCommandBuilder
from google.protobuf import wrappers_pb2, timestamp_pb2, duration_pb2
from bosdyn.client.frame_helpers import (
    add_edge_to_tree,
    VISION_FRAME_NAME,
    BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
)

from spot_driver.spot_ros import SpotROS
from spot_driver.spot_wrapper import SpotWrapper


# Stubbed SpotWrapper class for testing
class TestSpotWrapper(SpotWrapper):
    def __init__(self):
        self._robot_state = robot_state_pb2.RobotState()
        self._metrics = robot_state_pb2.RobotMetrics()
        self._lease_list = lease_pb2.ListLeasesResponse()
        self._front_image = [image_pb2.ImageResponse()]
        self._side_image = [image_pb2.ImageResponse()]
        self._rear_image = [image_pb2.ImageResponse()]
        self._hand_image = [image_pb2.ImageResponse()]

        self._valid = True
        self._robot_params = {
            "is_standing": False,
            "is_sitting": True,
            "is_moving": False,
            "robot_id": None,
            "estop_timeout": 9.0,
        }
        self._async_tasks = AsyncTasks()
        self._mobility_params = RobotCommandBuilder.mobility_params()

    @property
    def time_skew(self) -> duration_pb2.Duration:
        robot_time_skew = duration_pb2.Duration(seconds=0, nanos=0)
        return robot_time_skew

    @property
    def robot_state(self) -> robot_state_pb2.RobotState:
        """Return latest  _robot_state data"""
        return self._robot_state

    @robot_state.setter
    def robot_state(self, robot_state: robot_state_pb2.RobotState):
        """Set the robot_state_task data"""
        self._robot_state = robot_state

    @property
    def metrics(self) -> robot_state_pb2.RobotMetrics:
        """Return latest _robot_metrics data"""
        return self._metrics

    @metrics.setter
    def metrics(self, robot_metrics: robot_state_pb2.RobotMetrics):
        """Set the _robot_metrics data"""
        self._metrics = robot_metrics

    @property
    def lease(self) -> lease_pb2.ListLeasesResponse:
        """Return latest _lease_list data"""
        return self._lease_list

    @lease.setter
    def lease(self, lease_list: lease_pb2.ListLeasesResponse):
        """Set the _lease_list data"""
        self._lease_list = lease_list

    @property
    def front_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest _front_image data"""
        return self._front_image

    @front_images.setter
    def front_images(self, front_image: typing.List[image_pb2.ImageResponse]):
        """Set the _front_image data"""
        self._front_image = front_image

    @property
    def side_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest _side_image data"""
        return self._side_image

    @side_images.setter
    def side_images(self, side_image: typing.List[image_pb2.ImageResponse]):
        """Set the _side_image data"""
        self._side_image = side_image

    @property
    def rear_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest _rear_image data"""
        return self._rear_image

    @rear_images.setter
    def rear_images(self, rear_image: typing.List[image_pb2.ImageResponse]):
        """Set the _rear_image data"""
        self._rear_image = rear_image

    @property
    def hand_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest _hand_image data"""
        return self._hand_image

    @hand_images.setter
    def hand_images(self, hand_image: typing.List[image_pb2.ImageResponse]):
        """Set the _hand_image data"""
        self._hand_image = hand_image

    def disconnect(self):
        pass


class TestSpotROS(SpotROS):
    def handle_claim(self, req):
        return TriggerResponse(success=True, message="Successfully called claim")

    def handle_release(self, req):
        return TriggerResponse(success=True, message="Successfully called release")

    def handle_locked_stop(self, req):
        return TriggerResponse(success=True, message="Successfully called locked_stop")

    def handle_stop(self, req):
        return TriggerResponse(success=True, message="Successfully called stop")

    def handle_self_right(self, req):
        return TriggerResponse(success=True, message="Successfully called self_right")

    def handle_sit(self, req):
        return TriggerResponse(success=True, message="Successfully called sit")

    def handle_stand(self, req):
        return TriggerResponse(success=True, message="Successfully called stand")

    def handle_posed_stand(self, req: PosedStandRequest):
        return PosedStandResponse(
            success=True, message="Successfully called posed_stand"
        )

    def handle_power_on(self, req) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called power_on")

    def handle_safe_power_off(self, req) -> TriggerResponse:
        return TriggerResponse(
            success=True, message="Successfully called safe_power_off"
        )

    def handle_estop_hard(self, req) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called estop_hard")

    def handle_estop_gentle(self, req) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called estop_gentle")

    def handle_estop_release(self, req) -> TriggerResponse:
        return TriggerResponse(
            success=True, message="Successfully called estop_release"
        )

    def handle_clear_behavior_fault(self, req) -> ClearBehaviorFaultResponse:
        return ClearBehaviorFaultResponse(
            success=True, message="Successfully called clear_behavior_fault"
        )

    def handle_stair_mode(self, req) -> SetBoolResponse:
        return SetBoolResponse(success=True, message="Successfully called stair_mode")

    def handle_locomotion_mode(self, req) -> SetLocomotionResponse:
        return SetLocomotionResponse(
            success=True, message="Successfully called locomotion_mode"
        )

    def handle_swing_height(self, req) -> SetSwingHeightResponse:
        return SetSwingHeightResponse(
            success=True, message="Successfully called swing_height"
        )

    def handle_velocity_limit(self, req) -> SetVelocityResponse:
        return SetVelocityResponse(
            success=True, message="Successfully called velocity_limit"
        )

    def handle_allow_motion(self, req: SetBool) -> typing.Tuple[bool, str]:
        return True, "Successfully called allow_motion"

    def handle_obstacle_params(self, req: MobilityParams) -> typing.Tuple[bool, str]:
        return True, "Successfully called obstacle_params"

    def handle_terrain_params(self, req: MobilityParams) -> typing.Tuple[bool, str]:
        return True, "Successfully called terrain_params"

    def handle_list_graph(self, upload_path) -> ListGraphResponse:
        return ListGraphResponse(waypoint_ids=["1", "2", "3"])

    def handle_roll_over_right(self, req) -> TriggerResponse:
        return TriggerResponse(
            success=True, message="Successfully called roll_over_right"
        )

    def handle_roll_over_left(self, req) -> TriggerResponse:
        return TriggerResponse(
            success=True, message="Successfully called roll_over_left"
        )

    def handle_dock(self, req) -> DockResponse:
        return DockResponse(success=True, message="Successfully called dock")

    def handle_undock(self, req) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called undock")

    def handle_get_docking_state(self, req) -> GetDockStateResponse:
        return GetDockStateResponse(dock_state=docking_pb2.DockState())

    def handle_arm_stow(self, srv_data) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called arm_stow")

    def handle_arm_unstow(self, srv_data) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called arm_unstow")

    def handle_gripper_open(self, srv_data) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called gripper_open")

    def handle_gripper_close(self, srv_data) -> TriggerResponse:
        return TriggerResponse(
            success=True, message="Successfully called gripper_close"
        )

    def handle_arm_carry(self, srv_data) -> TriggerResponse:
        return TriggerResponse(success=True, message="Successfully called arm_carry")

    def handle_gripper_angle_open(
        self, srv_data: GripperAngleMoveRequest
    ) -> GripperAngleMoveResponse:
        return GripperAngleMoveResponse(
            success=True, message="Successfully called gripper_angle_open"
        )

    def handle_arm_joint_move(
        self, srv_data: ArmJointMovementRequest
    ) -> ArmJointMovementResponse:
        return ArmJointMovementResponse(
            success=True, message="Successfully called arm_joint_move"
        )

    def handle_force_trajectory(self, srv_data) -> ArmForceTrajectoryResponse:
        return ArmForceTrajectoryResponse(
            success=True, message="Successfully called force_trajectory"
        )

    def handle_gripper_pose(self, srv_data: HandPoseRequest) -> HandPoseResponse:
        return HandPoseResponse(
            success=True, message="Successfully called gripper_pose"
        )

    def handle_navigate_to(self, msg: NavigateToGoal):
        self.navigate_as.set_succeeded(
            NavigateToResult(success=True, message="Successfully called navigate_to")
        )

    def handle_trajectory(self, msg: TrajectoryGoal):
        self.trajectory_server.set_succeeded(
            TrajectoryResult(success=True, message="Successfully called trajectory")
        )

    def handle_in_motion_or_idle_body_pose(self, goal: PoseBodyGoal):
        self.motion_or_idle_body_pose_as.set_succeeded(
            PoseBodyResult(
                success=True, message="Successfully called motion_or_idle_body_pose"
            )
        )

    def handle_posed_stand_action(self, goal: PoseBodyGoal):
        self.body_pose_as.set_succeeded(
            PoseBodyResult(success=True, message="Successfully called body_pose")
        )

    def handle_spot_check(self, req: SpotCheckRequest) -> SpotCheckResponse:
        return SpotCheckResponse(success=True, message="Successfully called spot_check")


# Run the mock SpotROS class as a node
class MockSpotROS:
    def __init__(self):
        self.spot_ros = TestSpotROS()
        self.spot_ros.node_name = "mock_spot_ros"
        self.spot_ros.spot_wrapper = TestSpotWrapper()

    def set_joint_states(self, state: robot_state_pb2.RobotState):
        # Test getJointStatesFromState with two joints and acquisition_timestamp
        state.kinematic_state.joint_states.add(
            name="fl.hx",
            position=wrappers_pb2.DoubleValue(value=1.0),
            velocity=wrappers_pb2.DoubleValue(value=2.0),
            acceleration=wrappers_pb2.DoubleValue(value=3.0),
            load=wrappers_pb2.DoubleValue(value=4.0),
        )
        state.kinematic_state.joint_states.add(
            name="fl.hy",
            position=wrappers_pb2.DoubleValue(value=5.0),
            velocity=wrappers_pb2.DoubleValue(value=6.0),
            acceleration=wrappers_pb2.DoubleValue(value=7.0),
            load=wrappers_pb2.DoubleValue(value=8.0),
        )
        state.kinematic_state.acquisition_timestamp.seconds = 30
        state.kinematic_state.acquisition_timestamp.nanos = 100
        return state

    def set_TF_states(self, state: robot_state_pb2.RobotState):
        # Test with vision frame transformation
        # Add mock edges to relate the body, odom and vision frames. Body is the root frame.
        body_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=-2, y=-3, z=-2),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        vision_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=2, y=3, z=2),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        none_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=0, y=0, z=0),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        edges = {}
        edges = add_edge_to_tree(
            edges, body_tform_example, BODY_FRAME_NAME, ODOM_FRAME_NAME
        )
        edges = add_edge_to_tree(
            edges, vision_tform_example, BODY_FRAME_NAME, VISION_FRAME_NAME
        )
        edges = add_edge_to_tree(edges, none_tform_example, "", BODY_FRAME_NAME)

        snapshot = geometry_pb2.FrameTreeSnapshot(child_to_parent_edge_map=edges)
        state.kinematic_state.transforms_snapshot.CopyFrom(snapshot)
        return state

    def set_twist_odom_states(self, state: robot_state_pb2.RobotState):
        state.kinematic_state.velocity_of_body_in_odom.linear.x = 1.0
        state.kinematic_state.velocity_of_body_in_odom.linear.y = 2.0
        state.kinematic_state.velocity_of_body_in_odom.linear.z = 3.0
        state.kinematic_state.velocity_of_body_in_odom.angular.x = 4.0
        state.kinematic_state.velocity_of_body_in_odom.angular.y = 5.0
        state.kinematic_state.velocity_of_body_in_odom.angular.z = 6.0
        return state

    def set_odom_states(self, state: robot_state_pb2.RobotState):
        # Add mock edges to relate the body, odom and vision frames. Body is the root frame.
        vision_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=2, y=3, z=2),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        body_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=-2, y=-3, z=-2),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        none_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=0, y=0, z=0),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        edges = {}
        edges = add_edge_to_tree(
            edges, body_tform_example, BODY_FRAME_NAME, ODOM_FRAME_NAME
        )
        edges = add_edge_to_tree(
            edges, vision_tform_example, BODY_FRAME_NAME, VISION_FRAME_NAME
        )
        edges = add_edge_to_tree(edges, none_tform_example, "", BODY_FRAME_NAME)

        snapshot = geometry_pb2.FrameTreeSnapshot(child_to_parent_edge_map=edges)
        state.kinematic_state.transforms_snapshot.CopyFrom(snapshot)
        return state

    def set_foot_states(self, state: robot_state_pb2.RobotState):
        # Test getFeetFromState with two feet
        state.foot_state.add(
            foot_position_rt_body=geometry_pb2.Vec3(x=1.0, y=2.0, z=3.0),
            contact=robot_state_pb2.FootState.CONTACT_MADE,
            terrain=robot_state_pb2.FootState.TerrainState(
                ground_mu_est=0.5,
                frame_name="frame1",
                foot_slip_distance_rt_frame=geometry_pb2.Vec3(x=1.0, y=2.0, z=3.0),
                foot_slip_velocity_rt_frame=geometry_pb2.Vec3(x=4.0, y=5.0, z=6.0),
                ground_contact_normal_rt_frame=geometry_pb2.Vec3(x=7.0, y=8.0, z=9.0),
                visual_surface_ground_penetration_mean=0.1,
                visual_surface_ground_penetration_std=0.02,
            ),
        )
        state.foot_state.add(
            foot_position_rt_body=geometry_pb2.Vec3(x=4.0, y=5.0, z=6.0),
            contact=robot_state_pb2.FootState.CONTACT_LOST,
            terrain=robot_state_pb2.FootState.TerrainState(
                ground_mu_est=0.6,
                frame_name="frame2",
                foot_slip_distance_rt_frame=geometry_pb2.Vec3(x=10.0, y=11.0, z=12.0),
                foot_slip_velocity_rt_frame=geometry_pb2.Vec3(x=13.0, y=14.0, z=15.0),
                ground_contact_normal_rt_frame=geometry_pb2.Vec3(
                    x=16.0, y=17.0, z=18.0
                ),
                visual_surface_ground_penetration_mean=0.2,
                visual_surface_ground_penetration_std=0.03,
            ),
        )

        return state

    def set_estop_states(self, state: robot_state_pb2.RobotState):
        # Test getEStopStateFromState, hardware estopped
        state.estop_states.add(
            timestamp=timestamp_pb2.Timestamp(seconds=30, nanos=10),
            name="estop1",
            state=robot_state_pb2.EStopState.STATE_ESTOPPED,
            type=robot_state_pb2.EStopState.TYPE_HARDWARE,
        )
        # Add software type estop state, not estopped
        state.estop_states.add(
            timestamp=timestamp_pb2.Timestamp(seconds=20, nanos=15),
            name="estop2",
            state=robot_state_pb2.EStopState.STATE_NOT_ESTOPPED,
            type=robot_state_pb2.EStopState.TYPE_SOFTWARE,
        )

        return state

    def set_wifi_states(self, state: robot_state_pb2.RobotState):
        initial_wifi_state = robot_state_pb2.WiFiState(
            current_mode=robot_state_pb2.WiFiState.MODE_ACCESS_POINT, essid="test_essid"
        )
        state.comms_states.add(wifi_state=initial_wifi_state)

        return state

    def set_battery_states(self, state: robot_state_pb2.RobotState):
        # Test with one battery state
        state.battery_states.add(
            timestamp=timestamp_pb2.Timestamp(seconds=1, nanos=2),
            identifier="battery1",
            charge_percentage=wrappers_pb2.DoubleValue(value=95.0),
            estimated_runtime=duration_pb2.Duration(seconds=100),
            current=wrappers_pb2.DoubleValue(value=10.0),
            voltage=wrappers_pb2.DoubleValue(value=9.0),
            temperatures=[25.0, 26.0, 27.0],
            status=robot_state_pb2.BatteryState.STATUS_DISCHARGING,
        )
        return state

    def set_power_states(self, state: robot_state_pb2.RobotState):
        state.power_state.motor_power_state = (
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_OFF
        )
        state.power_state.shore_power_state = (
            robot_state_pb2.PowerState.SHORE_POWER_STATE_ON
        )
        return state

    def set_system_fault_states(self, state: robot_state_pb2.RobotState):
        state.system_fault_state.faults.add(
            name="fault1",
            onset_timestamp=timestamp_pb2.Timestamp(seconds=1, nanos=2),
            duration=duration_pb2.Duration(seconds=3, nanos=4),
            code=42,
            uid=5,
            error_message="error message1",
            attributes=["imu", "power"],
            severity=robot_state_pb2.SystemFault.SEVERITY_WARN,
        )

        state.system_fault_state.historical_faults.add(
            name="fault2",
            onset_timestamp=timestamp_pb2.Timestamp(seconds=6, nanos=7),
            duration=duration_pb2.Duration(seconds=8, nanos=9),
            code=43,
            uid=10,
            error_message="error message2",
            attributes=["wifi", "vision"],
            severity=robot_state_pb2.SystemFault.SEVERITY_CRITICAL,
        )

        return state

    def set_behaviour_fault_states(self, state: robot_state_pb2.RobotState):
        state.behavior_fault_state.faults.add(
            behavior_fault_id=1,
            onset_timestamp=timestamp_pb2.Timestamp(seconds=1, nanos=2),
            cause=robot_state_pb2.BehaviorFault.CAUSE_FALL,
            status=robot_state_pb2.BehaviorFault.STATUS_UNCLEARABLE,
        )

        state.behavior_fault_state.faults.add(
            behavior_fault_id=3,
            onset_timestamp=timestamp_pb2.Timestamp(seconds=4, nanos=5),
            cause=robot_state_pb2.BehaviorFault.CAUSE_LEASE_TIMEOUT,
            status=robot_state_pb2.BehaviorFault.STATUS_CLEARABLE,
        )

        return state

    def set_robot_state(self):
        # Create a robot state message inside the spot_wrapper object
        self.spot_ros.spot_wrapper.robot_state = robot_state_pb2.RobotState()

        # Set the robot state message's joint state field
        self.set_joint_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_TF_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_twist_odom_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_odom_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_foot_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_estop_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_wifi_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_battery_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_power_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_system_fault_states(self.spot_ros.spot_wrapper.robot_state)
        self.set_behaviour_fault_states(self.spot_ros.spot_wrapper.robot_state)

    def set_robot_metrics(self):
        # Create a robot metrics message inside the spot_wrapper object
        self.spot_ros.spot_wrapper.metrics = robot_state_pb2.RobotMetrics()

        # Populate the Metrics message field with a timestamp and metrics
        self.spot_ros.spot_wrapper.metrics.timestamp.seconds = 1
        self.spot_ros.spot_wrapper.metrics.timestamp.nanos = 2
        self.spot_ros.spot_wrapper.metrics.metrics.add(
            label="distance", float_value=3.0
        )
        self.spot_ros.spot_wrapper.metrics.metrics.add(label="gait cycles", int_value=4)
        self.spot_ros.spot_wrapper.metrics.metrics.add(
            label="time moving", duration=duration_pb2.Duration(seconds=5, nanos=6)
        )
        self.spot_ros.spot_wrapper.metrics.metrics.add(
            label="electric power", duration=duration_pb2.Duration(seconds=7, nanos=8)
        )

    def set_robot_lease(self):
        # Create a robot lease message inside the spot_wrapper object
        list_lease_resp = lease_pb2.ListLeasesResponse()

        # Populate the lease message field with a timestamp and lease
        list_lease_resp.resources.add(
            resource="spot",
            lease=lease_pb2.Lease(
                resource="lease_id",
                epoch="epoch1",
                sequence=[1, 2, 3],
                client_names=["Adam", "Bob", "Charlie"],
            ),
            lease_owner=lease_pb2.LeaseOwner(client_name="Adam", user_name="Dylan"),
            stale_time=timestamp_pb2.Timestamp(seconds=4, nanos=5),
        )

        self.spot_ros.spot_wrapper.lease = list_lease_resp.resources

    def set_robot_front_camera_data(self):
        # Create a robot front camera data message inside the spot_wrapper object
        self.spot_ros.spot_wrapper.front_images = []

        # Populate the front camera data message field with a timestamp and image
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "frontleft_fisheye_image"
        self.spot_ros.spot_wrapper.front_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "frontright_fisheye_image"
        self.spot_ros.spot_wrapper.front_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 240
        image_response.shot.image.rows = 424
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
        image_response.shot.frame_name_image_sensor = "frontleft_depth"
        self.spot_ros.spot_wrapper.front_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 240
        image_response.shot.image.rows = 424
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
        image_response.shot.frame_name_image_sensor = "frontright_depth"
        self.spot_ros.spot_wrapper.front_images.append(image_response)

    def set_robot_rear_camera_data(self):
        # Create a robot rear camera data message inside the spot_wrapper object
        self.spot_ros.spot_wrapper.rear_images = []

        # Populate the rear camera data message field with a timestamp and image
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 640
        image_response.shot.image.rows = 480
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "back_fisheye_image"
        self.spot_ros.spot_wrapper.rear_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 424
        image_response.shot.image.rows = 240
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
        image_response.shot.frame_name_image_sensor = "back_depth"
        self.spot_ros.spot_wrapper.rear_images.append(image_response)

    def set_robot_side_camera_data(self):
        # Create a robot left camera data message inside the spot_wrapper object
        self.spot_ros.spot_wrapper.side_images = []

        # Populate the left camera data message field with a timestamp and image
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "left_fisheye_image"
        self.spot_ros.spot_wrapper.side_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 240
        image_response.shot.image.rows = 424
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
        image_response.shot.frame_name_image_sensor = "left_depth"
        self.spot_ros.spot_wrapper.side_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "right_fisheye_image"
        self.spot_ros.spot_wrapper.side_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 240
        image_response.shot.image.rows = 424
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
        image_response.shot.frame_name_image_sensor = "right_depth"
        self.spot_ros.spot_wrapper.side_images.append(image_response)

    def set_hand_camera_data(self):
        # Create a hand camera data message inside the spot_wrapper object
        self.spot_ros.spot_wrapper.hand_images = []

        # Populate the hand camera data message field with a timestamp and image
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.data = b"hand_image"
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "hand_image"
        self.spot_ros.spot_wrapper.hand_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 240
        image_response.shot.image.rows = 424
        image_response.shot.image.data = b"hand_depth"
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
        image_response.shot.frame_name_image_sensor = "hand_depth"
        self.spot_ros.spot_wrapper.hand_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.data = b"hand_color_image"
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
        image_response.shot.frame_name_image_sensor = "hand_color_image"
        self.spot_ros.spot_wrapper.hand_images.append(image_response)

        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 480
        image_response.shot.image.rows = 640
        image_response.shot.image.data = b"hand_depth_in_color_frame"
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = (
            image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
        )
        image_response.shot.frame_name_image_sensor = "hand_depth_in_color_frame"
        self.spot_ros.spot_wrapper.hand_images.append(image_response)

    def main(self):
        rospy.init_node(self.spot_ros.node_name, anonymous=True)
        # Initialize variables for transforms
        self.spot_ros.mode_parent_odom_tf = "vision"
        self.spot_ros.tf_name_kinematic_odom = "odom"
        self.spot_ros.tf_name_raw_kinematic = "odom"
        self.spot_ros.tf_name_vision_odom = "vision"
        self.spot_ros.tf_name_raw_vision = "vision"

        # Set up the ROS publishers, subscribers, services, and action servers
        self.spot_ros.initialize_publishers()
        self.spot_ros.initialize_subscribers()
        self.spot_ros.initialize_services()
        self.spot_ros.initialize_action_servers()

        # Manually set robot_state, metrics, lease data
        self.set_robot_state()
        self.set_robot_metrics()
        self.set_robot_lease()

        # Manually set robot images data
        self.set_robot_front_camera_data()
        self.set_robot_rear_camera_data()
        self.set_robot_side_camera_data()
        self.set_hand_camera_data()

        # Set running rate
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            # Call publish callbacks
            self.spot_ros.RobotStateCB("robot_state_test")
            self.spot_ros.MetricsCB("metrics_test")
            self.spot_ros.LeaseCB("lease_test")
            self.spot_ros.FrontImageCB("front_camera_test")
            self.spot_ros.RearImageCB("rear_camera_test")
            self.spot_ros.SideImageCB("side_camera_test")
            self.spot_ros.HandImageCB("hand_camera_test")
            rate.sleep()


if __name__ == "__main__":
    run_spot_ros = MockSpotROS()
    run_spot_ros.main()
