PKG = "ros_helpers"
NAME = "ros_helpers_test"
SUITE = "ros_helpers_test.TestSuiteROSHelpers"

import unittest
import sys
import rostest
import rospy

import spot_driver.ros_helpers as ros_helpers
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import Image, CameraInfo
from spot_msgs.msg import WiFiState
from spot_msgs.msg import FootState, FootStateArray
from bosdyn.api import image_pb2, geometry_pb2, robot_state_pb2
from bosdyn.api.docking import docking_pb2
from google.protobuf import wrappers_pb2, timestamp_pb2, duration_pb2
from bosdyn.client.frame_helpers import (
    add_edge_to_tree,
    VISION_FRAME_NAME,
    BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
)

from spot_driver.spot_wrapper import SpotWrapper


class TestSpotWrapper(SpotWrapper):
    def __init__(self):
        pass

    @property
    def time_skew(self) -> duration_pb2.Duration:
        robot_time_skew = duration_pb2.Duration(seconds=0, nanos=0)
        return robot_time_skew


class TestPopulateTransformStamped(unittest.TestCase):
    def test_populate_transform_stamped(self):
        # Test populateTransformStamped
        transform = Transform()
        transform.translation.x = 1.0
        transform.translation.y = 2.0
        transform.translation.z = 3.0
        transform.rotation.x = 4.0
        transform.rotation.y = 5.0
        transform.rotation.z = 6.0
        transform.rotation.w = 7.0

        current_time = rospy.Time.now()
        transform_stamped = ros_helpers.populateTransformStamped(
            current_time, "parent_frame", "child_frame", transform
        )
        self.assertEquals(transform_stamped.header.stamp, current_time)
        self.assertEquals(transform_stamped.header.frame_id, "parent_frame")
        self.assertEquals(transform_stamped.child_frame_id, "child_frame")
        self.assertEquals(transform_stamped.transform, transform)


class TestGetImageMsg(unittest.TestCase):
    @property
    def test_get_image_msg(self):
        spot_wrapper = TestSpotWrapper()

        # Test getImageMsg with a valid image
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.cols = 640
        image_response.shot.image.rows = 480
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8

        image, camera_info = ros_helpers.getImageMsg(image_response, spot_wrapper)
        self.assertEquals(image.cols, 640)
        self.assertEquals(image.rows, 480)
        self.assertEquals(image.pixel_format, 3)
        self.assertEquals(camera_info.cols, 640)
        self.assertEquals(camera_info.rows, 480)


class TestGetJointStatesFromState(unittest.TestCase):
    def test_get_joint_states_from_state(self):
        # Test getJointStatesFromState with two joints and acquisition_timestamp
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()
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

        joint_state = ros_helpers.GetJointStatesFromState(state, spot_wrapper)
        self.assertEquals(joint_state.name[0], "front_left_hip_x")
        self.assertEquals(joint_state.position[0], 1.0)
        self.assertEquals(joint_state.velocity[0], 2.0)
        self.assertEquals(joint_state.effort[0], 4.0)
        self.assertEquals(joint_state.name[1], "front_left_hip_y")
        self.assertEquals(joint_state.position[1], 5.0)
        self.assertEquals(joint_state.velocity[1], 6.0)
        self.assertEquals(joint_state.effort[1], 8.0)
        self.assertEquals(joint_state.header.stamp.secs, 30)
        self.assertEquals(joint_state.header.stamp.nsecs, 100)


class TestGetEStopStateFromState(unittest.TestCase):
    def test_get_estop_state_from_state(self):
        # Test getEStopStateFromState, hardware estopped
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()
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

        estop_state_array = ros_helpers.GetEStopStateFromState(state, spot_wrapper)

        self.assertEquals(estop_state_array.estop_states[0].name, "estop1")
        self.assertEquals(
            estop_state_array.estop_states[0].state,
            robot_state_pb2.EStopState.STATE_ESTOPPED,
        )
        self.assertEquals(
            estop_state_array.estop_states[0].type,
            robot_state_pb2.EStopState.TYPE_HARDWARE,
        )
        self.assertEquals(estop_state_array.estop_states[0].header.stamp.secs, 30)
        self.assertEquals(estop_state_array.estop_states[0].header.stamp.nsecs, 10)

        self.assertEquals(estop_state_array.estop_states[1].name, "estop2")
        self.assertEquals(
            estop_state_array.estop_states[1].state,
            robot_state_pb2.EStopState.STATE_NOT_ESTOPPED,
        )
        self.assertEquals(
            estop_state_array.estop_states[1].type,
            robot_state_pb2.EStopState.TYPE_SOFTWARE,
        )
        self.assertEquals(estop_state_array.estop_states[1].header.stamp.secs, 20)
        self.assertEquals(estop_state_array.estop_states[1].header.stamp.nsecs, 15)


class TestGetFeetFromState(unittest.TestCase):
    def test_get_feet_from_state(self):
        # Test getFeetFromState with two feet
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()
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

        foot_state_array = ros_helpers.GetFeetFromState(state, spot_wrapper)
        self.assertEquals(foot_state_array.states[0].foot_position_rt_body.y, 2.0)
        self.assertEquals(foot_state_array.states[0].foot_position_rt_body.z, 3.0)
        self.assertEquals(foot_state_array.states[0].foot_position_rt_body.x, 1.0)
        self.assertEquals(
            foot_state_array.states[0].contact, robot_state_pb2.FootState.CONTACT_MADE
        )
        self.assertEquals(foot_state_array.states[0].terrain.ground_mu_est, 0.5)
        self.assertEquals(foot_state_array.states[0].terrain.frame_name, "frame1")
        self.assertEquals(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.x, 1.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.y, 2.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.z, 3.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.x, 4.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.y, 5.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.z, 6.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.x, 7.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.y, 8.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.z, 9.0
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.visual_surface_ground_penetration_mean,
            0.1,
        )
        self.assertEquals(
            foot_state_array.states[0].terrain.visual_surface_ground_penetration_std,
            0.02,
        )

        self.assertEquals(foot_state_array.states[1].foot_position_rt_body.y, 5.0)
        self.assertEquals(foot_state_array.states[1].foot_position_rt_body.z, 6.0)
        self.assertEquals(foot_state_array.states[1].foot_position_rt_body.x, 4.0)
        self.assertEquals(
            foot_state_array.states[1].contact, robot_state_pb2.FootState.CONTACT_LOST
        )
        self.assertEquals(foot_state_array.states[1].terrain.ground_mu_est, 0.6)
        self.assertEquals(foot_state_array.states[1].terrain.frame_name, "frame2")
        self.assertEquals(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.x, 10.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.y, 11.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.z, 12.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.x, 13.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.y, 14.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.z, 15.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.x, 16.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.y, 17.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.z, 18.0
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.visual_surface_ground_penetration_mean,
            0.2,
        )
        self.assertEquals(
            foot_state_array.states[1].terrain.visual_surface_ground_penetration_std,
            0.03,
        )


class TestGetOdomTwistFromState(unittest.TestCase):
    def test_get_odom_twist_from_state(self):
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()
        state.kinematic_state.velocity_of_body_in_odom.linear.x = 1.0
        state.kinematic_state.velocity_of_body_in_odom.linear.y = 2.0
        state.kinematic_state.velocity_of_body_in_odom.linear.z = 3.0
        state.kinematic_state.velocity_of_body_in_odom.angular.x = 4.0
        state.kinematic_state.velocity_of_body_in_odom.angular.y = 5.0
        state.kinematic_state.velocity_of_body_in_odom.angular.z = 6.0

        twist_odom_msg = ros_helpers.GetOdomTwistFromState(state, spot_wrapper)
        self.assertEquals(twist_odom_msg.twist.twist.linear.x, 1.0)
        self.assertEquals(twist_odom_msg.twist.twist.linear.y, 2.0)
        self.assertEquals(twist_odom_msg.twist.twist.linear.z, 3.0)
        self.assertEquals(twist_odom_msg.twist.twist.angular.x, 4.0)
        self.assertEquals(twist_odom_msg.twist.twist.angular.y, 5.0)
        self.assertEquals(twist_odom_msg.twist.twist.angular.z, 6.0)


class TestGetOdomFromState(unittest.TestCase):
    def test_get_odom_from_state(self):
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()

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

        # Test with vision frame transformation
        odometry_msg = ros_helpers.GetOdomFromState(state, spot_wrapper, True)
        self.assertEquals(
            odometry_msg.pose.pose.position.x, -vision_tform_example.position.x
        )
        self.assertEquals(
            odometry_msg.pose.pose.position.y, -vision_tform_example.position.y
        )
        self.assertEquals(
            odometry_msg.pose.pose.position.z, -vision_tform_example.position.z
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.x, vision_tform_example.rotation.x
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.y, vision_tform_example.rotation.y
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.z, vision_tform_example.rotation.z
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.w, vision_tform_example.rotation.w
        )

        # Test with odom frame transformation
        odometry_msg = ros_helpers.GetOdomFromState(state, spot_wrapper, False)
        self.assertEquals(
            odometry_msg.pose.pose.position.x, -body_tform_example.position.x
        )
        self.assertEquals(
            odometry_msg.pose.pose.position.y, -body_tform_example.position.y
        )
        self.assertEquals(
            odometry_msg.pose.pose.position.z, -body_tform_example.position.z
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.x, body_tform_example.rotation.x
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.y, body_tform_example.rotation.y
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.z, body_tform_example.rotation.z
        )
        self.assertEquals(
            odometry_msg.pose.pose.orientation.w, body_tform_example.rotation.w
        )


class TestGetWifiFromState(unittest.TestCase):
    def test_get_wifi_from_state(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()
        initial_wifi_state = robot_state_pb2.WiFiState(
            current_mode=robot_state_pb2.WiFiState.MODE_ACCESS_POINT, essid="test_essid"
        )
        state.comms_states.add(wifi_state=initial_wifi_state)

        wifi_state = ros_helpers.GetWifiFromState(state, spot_wrapper)
        self.assertEquals(
            wifi_state.current_mode, robot_state_pb2.WiFiState.MODE_ACCESS_POINT
        )
        self.assertEquals(wifi_state.essid, "test_essid")


class TestGenerateFeetTF(unittest.TestCase):
    def test_generate_feet_tf(self):
        foot_state_msg = FootStateArray(
            [
                FootState(foot_position_rt_body=geometry_pb2.Vec3(x=1.0, y=2.0, z=3.0)),
                FootState(foot_position_rt_body=geometry_pb2.Vec3(x=4.0, y=5.0, z=6.0)),
                FootState(foot_position_rt_body=geometry_pb2.Vec3(x=7.0, y=8.0, z=9.0)),
                FootState(
                    foot_position_rt_body=geometry_pb2.Vec3(x=10.0, y=11.0, z=12.0)
                ),
            ]
        )
        tf_message = ros_helpers.GenerateFeetTF(foot_state_msg)
        self.assertEquals(len(tf_message.transforms), 4)
        self.assertEquals(tf_message.transforms[0].transform.translation.x, 1.0)
        self.assertEquals(tf_message.transforms[0].transform.translation.y, 2.0)
        self.assertEquals(tf_message.transforms[0].transform.translation.z, 3.0)
        self.assertEquals(tf_message.transforms[0].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[0].child_frame_id, "front_left_foot")
        self.assertEquals(tf_message.transforms[1].transform.translation.x, 4.0)
        self.assertEquals(tf_message.transforms[1].transform.translation.y, 5.0)
        self.assertEquals(tf_message.transforms[1].transform.translation.z, 6.0)
        self.assertEquals(tf_message.transforms[1].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[1].child_frame_id, "front_right_foot")
        self.assertEquals(tf_message.transforms[2].transform.translation.x, 7.0)
        self.assertEquals(tf_message.transforms[2].transform.translation.y, 8.0)
        self.assertEquals(tf_message.transforms[2].transform.translation.z, 9.0)
        self.assertEquals(tf_message.transforms[2].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[2].child_frame_id, "rear_left_foot")
        self.assertEquals(tf_message.transforms[3].transform.translation.x, 10.0)
        self.assertEquals(tf_message.transforms[3].transform.translation.y, 11.0)
        self.assertEquals(tf_message.transforms[3].transform.translation.z, 12.0)
        self.assertEquals(tf_message.transforms[3].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[3].child_frame_id, "rear_right_foot")


class TestGetTFFromState(unittest.TestCase):
    def test_get_tf_from_state_body(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()
        inverse_target_frame = "body"

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
        tf_message = ros_helpers.GetTFFromState(
            state, spot_wrapper, inverse_target_frame
        )

        self.assertEquals(len(tf_message.transforms), 2)
        self.assertEquals(tf_message.transforms[0].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[0].child_frame_id, "odom")
        self.assertEquals(tf_message.transforms[0].transform.translation.x, -2.0)
        self.assertEquals(tf_message.transforms[0].transform.translation.y, -3.0)
        self.assertEquals(tf_message.transforms[0].transform.translation.z, -2.0)

        self.assertEquals(tf_message.transforms[1].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[1].child_frame_id, "vision")
        self.assertEquals(tf_message.transforms[1].transform.translation.x, 2.0)
        self.assertEquals(tf_message.transforms[1].transform.translation.y, 3.0)
        self.assertEquals(tf_message.transforms[1].transform.translation.z, 2.0)

    def test_get_tf_from_state_vision(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()
        inverse_target_frame = "vision"

        # Test with vision frame transformation
        # Add mock edges to relate the body, odom and vision frames. Body is the root frame.
        body_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=-2, y=-3, z=-4),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        vision_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=4, y=5, z=6),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        special_tform_example = geometry_pb2.SE3Pose(
            position=geometry_pb2.Vec3(x=7, y=8, z=9),
            rotation=geometry_pb2.Quaternion(x=0, y=0, z=0, w=1),
        )
        edges = {}
        edges = add_edge_to_tree(
            edges, body_tform_example, BODY_FRAME_NAME, ODOM_FRAME_NAME
        )
        edges = add_edge_to_tree(
            edges, vision_tform_example, BODY_FRAME_NAME, VISION_FRAME_NAME
        )
        edges = add_edge_to_tree(
            edges, special_tform_example, "special_frame", BODY_FRAME_NAME
        )

        snapshot = geometry_pb2.FrameTreeSnapshot(child_to_parent_edge_map=edges)
        state.kinematic_state.transforms_snapshot.CopyFrom(snapshot)
        tf_message = ros_helpers.GetTFFromState(
            state, spot_wrapper, inverse_target_frame
        )
        self.assertEquals(len(tf_message.transforms), 3)

        self.assertEquals(tf_message.transforms[0].header.frame_id, "body")
        self.assertEquals(tf_message.transforms[0].child_frame_id, "odom")
        self.assertEquals(tf_message.transforms[0].transform.translation.x, -2.0)
        self.assertEquals(tf_message.transforms[0].transform.translation.y, -3.0)
        self.assertEquals(tf_message.transforms[0].transform.translation.z, -4.0)

        self.assertEquals(tf_message.transforms[1].header.frame_id, "vision")
        self.assertEquals(tf_message.transforms[1].child_frame_id, "body")
        self.assertEquals(tf_message.transforms[1].transform.translation.x, -4.0)
        self.assertEquals(tf_message.transforms[1].transform.translation.y, -5.0)
        self.assertEquals(tf_message.transforms[1].transform.translation.z, -6.0)

        self.assertEquals(tf_message.transforms[2].header.frame_id, "special_frame")
        self.assertEquals(tf_message.transforms[2].child_frame_id, "body")
        self.assertEquals(tf_message.transforms[2].transform.translation.x, 7.0)
        self.assertEquals(tf_message.transforms[2].transform.translation.y, 8.0)
        self.assertEquals(tf_message.transforms[2].transform.translation.z, 9.0)


class TestGetBatteryStatesFromState(unittest.TestCase):
    def test_get_battery_states_from_state_zero(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()

        # Test with no battery states
        battery_states = ros_helpers.GetBatteryStatesFromState(state, spot_wrapper)
        self.assertEquals(len(battery_states.battery_states), 0)

    def test_get_battery_states_from_state_one(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()
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
        battery_states = ros_helpers.GetBatteryStatesFromState(state, spot_wrapper)
        self.assertEquals(len(battery_states.battery_states), 1)
        self.assertEquals(battery_states.battery_states[0].header.stamp.secs, 1)
        self.assertEquals(battery_states.battery_states[0].header.stamp.nsecs, 2)
        self.assertEquals(battery_states.battery_states[0].identifier, "battery1")
        self.assertEquals(battery_states.battery_states[0].charge_percentage, 95.0)
        self.assertEquals(battery_states.battery_states[0].estimated_runtime.secs, 100)
        self.assertEquals(battery_states.battery_states[0].current, 10.0)
        self.assertEquals(battery_states.battery_states[0].voltage, 9.0)
        self.assertEquals(
            battery_states.battery_states[0].temperatures, [25.0, 26.0, 27.0]
        )
        self.assertEquals(
            battery_states.battery_states[0].status,
            robot_state_pb2.BatteryState.STATUS_DISCHARGING,
        )

    def test_get_battery_states_from_state_two(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()
        # Test with two battery states
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
        state.battery_states.add(
            timestamp=timestamp_pb2.Timestamp(seconds=3, nanos=4),
            identifier="battery2",
            charge_percentage=wrappers_pb2.DoubleValue(value=5.0),
            estimated_runtime=duration_pb2.Duration(seconds=10),
            current=wrappers_pb2.DoubleValue(value=5.0),
            voltage=wrappers_pb2.DoubleValue(value=10.0),
            temperatures=[35.0, 36.0, 37.0],
            status=robot_state_pb2.BatteryState.STATUS_CHARGING,
        )
        battery_states = ros_helpers.GetBatteryStatesFromState(state, spot_wrapper)
        self.assertEquals(len(battery_states.battery_states), 2)
        self.assertEquals(battery_states.battery_states[0].header.stamp.secs, 1)
        self.assertEquals(battery_states.battery_states[0].header.stamp.nsecs, 2)
        self.assertEquals(battery_states.battery_states[0].identifier, "battery1")
        self.assertEquals(battery_states.battery_states[0].charge_percentage, 95.0)
        self.assertEquals(battery_states.battery_states[0].estimated_runtime.secs, 100)
        self.assertEquals(battery_states.battery_states[0].current, 10.0)
        self.assertEquals(battery_states.battery_states[0].voltage, 9.0)
        self.assertEquals(
            battery_states.battery_states[0].temperatures, [25.0, 26.0, 27.0]
        )
        self.assertEquals(
            battery_states.battery_states[0].status,
            robot_state_pb2.BatteryState.STATUS_DISCHARGING,
        )
        self.assertEquals(battery_states.battery_states[1].header.stamp.secs, 3)
        self.assertEquals(battery_states.battery_states[1].header.stamp.nsecs, 4)
        self.assertEquals(battery_states.battery_states[1].identifier, "battery2")
        self.assertEquals(battery_states.battery_states[1].charge_percentage, 5.0)
        self.assertEquals(battery_states.battery_states[1].estimated_runtime.secs, 10)
        self.assertEquals(battery_states.battery_states[1].current, 5.0)
        self.assertEquals(battery_states.battery_states[1].voltage, 10.0)
        self.assertEquals(
            battery_states.battery_states[1].temperatures, [35.0, 36.0, 37.0]
        )
        self.assertEquals(
            battery_states.battery_states[1].status,
            robot_state_pb2.BatteryState.STATUS_CHARGING,
        )


class TestGetPowerStatesFromState(unittest.TestCase):
    def test_get_power_states_from_state_off(self):
        state = robot_state_pb2.RobotState()
        state.power_state.motor_power_state = (
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_OFF
        )
        state.power_state.shore_power_state = (
            robot_state_pb2.PowerState.SHORE_POWER_STATE_ON
        )
        spot_wrapper = TestSpotWrapper()
        power_state_msg = ros_helpers.GetPowerStatesFromState(state, spot_wrapper)
        self.assertEquals(
            power_state_msg.motor_power_state,
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_OFF,
        )
        self.assertEquals(
            power_state_msg.shore_power_state,
            robot_state_pb2.PowerState.SHORE_POWER_STATE_ON,
        )

    def test_get_power_states_from_state_motor_on(self):
        state = robot_state_pb2.RobotState()
        state.power_state.motor_power_state = (
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_ON
        )
        state.power_state.shore_power_state = (
            robot_state_pb2.PowerState.SHORE_POWER_STATE_OFF
        )
        spot_wrapper = TestSpotWrapper()
        power_state_msg = ros_helpers.GetPowerStatesFromState(state, spot_wrapper)
        self.assertEquals(
            power_state_msg.motor_power_state,
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_ON,
        )
        self.assertEquals(
            power_state_msg.shore_power_state,
            robot_state_pb2.PowerState.SHORE_POWER_STATE_OFF,
        )


class TestGetDockStatesFromState(unittest.TestCase):
    def test_get_dock_states_from_state(self):
        state = docking_pb2.DockState()
        state.status = docking_pb2.DockState.DOCK_STATUS_DOCKED
        state.dock_type = docking_pb2.DOCK_TYPE_SPOT_DOCK
        state.dock_id = 3
        state.power_status = docking_pb2.DockState.LINK_STATUS_CONNECTED

        dock_state_msg = ros_helpers.GetDockStatesFromState(state)
        self.assertEquals(
            dock_state_msg.status, docking_pb2.DockState.DOCK_STATUS_DOCKED
        )
        self.assertEquals(dock_state_msg.dock_type, docking_pb2.DOCK_TYPE_SPOT_DOCK)
        self.assertEquals(dock_state_msg.dock_id, 3)
        self.assertEquals(
            dock_state_msg.power_status, docking_pb2.DockState.LINK_STATUS_CONNECTED
        )


class TestGetBehaviorFaults(unittest.TestCase):
    def test_get_behavior_faults(self):
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.BehaviorFaultState()
        state.faults.add(
            behavior_fault_id=1,
            onset_timestamp=timestamp_pb2.Timestamp(seconds=1, nanos=2),
            cause=robot_state_pb2.BehaviorFault.CAUSE_FALL,
            status=robot_state_pb2.BehaviorFault.STATUS_UNCLEARABLE,
        )

        state.faults.add(
            behavior_fault_id=3,
            onset_timestamp=timestamp_pb2.Timestamp(seconds=4, nanos=5),
            cause=robot_state_pb2.BehaviorFault.CAUSE_LEASE_TIMEOUT,
            status=robot_state_pb2.BehaviorFault.STATUS_CLEARABLE,
        )

        behavior_faults = ros_helpers.GetBehaviorFaults(state.faults, spot_wrapper)
        self.assertEquals(behavior_faults[0].behavior_fault_id, 1)
        self.assertEquals(behavior_faults[0].header.stamp.secs, 1)
        self.assertEquals(behavior_faults[0].header.stamp.nsecs, 2)
        self.assertEquals(
            behavior_faults[0].cause, robot_state_pb2.BehaviorFault.CAUSE_FALL
        )
        self.assertEquals(
            behavior_faults[0].status, robot_state_pb2.BehaviorFault.STATUS_UNCLEARABLE
        )

        self.assertEquals(behavior_faults[1].behavior_fault_id, 3)
        self.assertEquals(behavior_faults[1].header.stamp.secs, 4)
        self.assertEquals(behavior_faults[1].header.stamp.nsecs, 5)
        self.assertEquals(
            behavior_faults[1].cause, robot_state_pb2.BehaviorFault.CAUSE_LEASE_TIMEOUT
        )
        self.assertEquals(
            behavior_faults[1].status, robot_state_pb2.BehaviorFault.STATUS_CLEARABLE
        )


class TestGetBehaviorFaultsFromState(unittest.TestCase):
    def test_get_behvaior_faults_from_state(self):
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()
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

        behavior_faults = ros_helpers.GetBehaviorFaultsFromState(state, spot_wrapper)
        self.assertEquals(behavior_faults.faults[0].behavior_fault_id, 1)
        self.assertEquals(behavior_faults.faults[0].header.stamp.secs, 1)
        self.assertEquals(behavior_faults.faults[0].header.stamp.nsecs, 2)
        self.assertEquals(
            behavior_faults.faults[0].cause, robot_state_pb2.BehaviorFault.CAUSE_FALL
        )
        self.assertEquals(
            behavior_faults.faults[0].status,
            robot_state_pb2.BehaviorFault.STATUS_UNCLEARABLE,
        )

        self.assertEquals(behavior_faults.faults[1].behavior_fault_id, 3)
        self.assertEquals(behavior_faults.faults[1].header.stamp.secs, 4)
        self.assertEquals(behavior_faults.faults[1].header.stamp.nsecs, 5)
        self.assertEquals(
            behavior_faults.faults[1].cause,
            robot_state_pb2.BehaviorFault.CAUSE_LEASE_TIMEOUT,
        )
        self.assertEquals(
            behavior_faults.faults[1].status,
            robot_state_pb2.BehaviorFault.STATUS_CLEARABLE,
        )


class TestGetSystemFaults(unittest.TestCase):
    def test_get_system_faults(self):
        spot_wrapper = TestSpotWrapper()
        system_fault_state = robot_state_pb2.SystemFaultState()
        system_fault_state.faults.add(
            name="fault1",
            onset_timestamp=timestamp_pb2.Timestamp(seconds=1, nanos=2),
            duration=duration_pb2.Duration(seconds=3, nanos=4),
            code=42,
            uid=5,
            error_message="error message1",
            attributes=["imu", "power"],
            severity=robot_state_pb2.SystemFault.SEVERITY_WARN,
        )

        system_fault_state.faults.add(
            name="fault2",
            onset_timestamp=timestamp_pb2.Timestamp(seconds=6, nanos=7),
            duration=duration_pb2.Duration(seconds=8, nanos=9),
            code=43,
            uid=10,
            error_message="error message2",
            attributes=["wifi", "vision"],
            severity=robot_state_pb2.SystemFault.SEVERITY_CRITICAL,
        )

        system_faults = ros_helpers.GetSystemFaults(
            system_fault_state.faults, spot_wrapper
        )
        self.assertEquals(system_faults[0].name, "fault1")
        self.assertEquals(system_faults[0].header.stamp.secs, 1)
        self.assertEquals(system_faults[0].header.stamp.nsecs, 2)
        self.assertEquals(system_faults[0].duration.secs, 3)
        self.assertEquals(system_faults[0].duration.nsecs, 4)
        self.assertEquals(system_faults[0].code, 42)
        self.assertEquals(system_faults[0].uid, 5)
        self.assertEquals(system_faults[0].error_message, "error message1")
        self.assertEquals(system_faults[0].attributes, ["imu", "power"])
        self.assertEquals(
            system_faults[0].severity, robot_state_pb2.SystemFault.SEVERITY_WARN
        )

        self.assertEquals(system_faults[1].name, "fault2")
        self.assertEquals(system_faults[1].header.stamp.secs, 6)
        self.assertEquals(system_faults[1].header.stamp.nsecs, 7)
        self.assertEquals(system_faults[1].duration.secs, 8)
        self.assertEquals(system_faults[1].duration.nsecs, 9)
        self.assertEquals(system_faults[1].code, 43)
        self.assertEquals(system_faults[1].uid, 10)
        self.assertEquals(system_faults[1].error_message, "error message2")
        self.assertEquals(system_faults[1].attributes, ["wifi", "vision"])
        self.assertEquals(
            system_faults[1].severity, robot_state_pb2.SystemFault.SEVERITY_CRITICAL
        )


class TestGetSystemFaultsFromState(unittest.TestCase):
    def test_get_system_faults_from_state(self):
        spot_wrapper = TestSpotWrapper()
        state = robot_state_pb2.RobotState()
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

        system_faults = ros_helpers.GetSystemFaultsFromState(state, spot_wrapper)
        self.assertEquals(system_faults.faults[0].name, "fault1")
        self.assertEquals(system_faults.faults[0].header.stamp.secs, 1)
        self.assertEquals(system_faults.faults[0].header.stamp.nsecs, 2)
        self.assertEquals(system_faults.faults[0].duration.secs, 3)
        self.assertEquals(system_faults.faults[0].duration.nsecs, 4)
        self.assertEquals(system_faults.faults[0].code, 42)
        self.assertEquals(system_faults.faults[0].uid, 5)
        self.assertEquals(system_faults.faults[0].error_message, "error message1")
        self.assertEquals(system_faults.faults[0].attributes, ["imu", "power"])
        self.assertEquals(
            system_faults.faults[0].severity, robot_state_pb2.SystemFault.SEVERITY_WARN
        )

        self.assertEquals(system_faults.historical_faults[0].name, "fault2")
        self.assertEquals(system_faults.historical_faults[0].header.stamp.secs, 6)
        self.assertEquals(system_faults.historical_faults[0].header.stamp.nsecs, 7)
        self.assertEquals(system_faults.historical_faults[0].duration.secs, 8)
        self.assertEquals(system_faults.historical_faults[0].duration.nsecs, 9)
        self.assertEquals(system_faults.historical_faults[0].code, 43)
        self.assertEquals(system_faults.historical_faults[0].uid, 10)
        self.assertEquals(
            system_faults.historical_faults[0].error_message, "error message2"
        )
        self.assertEquals(
            system_faults.historical_faults[0].attributes, ["wifi", "vision"]
        )
        self.assertEquals(
            system_faults.historical_faults[0].severity,
            robot_state_pb2.SystemFault.SEVERITY_CRITICAL,
        )


class TestSuiteROSHelpers(unittest.TestSuite):
    def __init__(self) -> None:
        super(TestSuiteROSHelpers, self).__init__()
        rospy.init_node("ros_helpers test")
        self.loader = unittest.TestLoader()
        self.addTest(self.loader.loadTestsFromTestCase(TestPopulateTransformStamped))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetImageMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetJointStatesFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetEStopStateFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetFeetFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetOdomTwistFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetOdomFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetWifiFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGenerateFeetTF))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetTFFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetBatteryStatesFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetPowerStatesFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetDockStatesFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetBehaviorFaults))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetBehaviorFaultsFromState))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetSystemFaults))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetSystemFaultsFromState))


if __name__ == "__main__":
    print("Starting tests!")
    rostest.rosrun(PKG, NAME, TestSuiteROSHelpers)
