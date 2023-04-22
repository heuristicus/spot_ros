#!/usr/bin/env python3
PKG = "ros_helpers"
NAME = "ros_helpers_test"
SUITE = "ros_helpers_test.TestSuiteROSHelpers"

import logging
import unittest
import rospy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import (
    FrameTreeSnapshot,
    AprilTagProperties,
    ImageProperties,
    ImageSource,
    WorldObject,
    WorldObjectArray,
)

from google.protobuf import wrappers_pb2, timestamp_pb2, duration_pb2
from bosdyn.api import (
    image_pb2,
    geometry_pb2,
    robot_state_pb2,
    point_cloud_pb2,
    world_object_pb2,
)
from bosdyn.api.docking import docking_pb2
from bosdyn.client.spot_check import spot_check_pb2
from bosdyn.client.frame_helpers import (
    add_edge_to_tree,
    VISION_FRAME_NAME,
    BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
)

import spot_driver.ros_helpers as ros_helpers
from spot_wrapper.wrapper import SpotWrapper


class TestSpotWrapper(SpotWrapper):
    def __init__(self):
        self._logger = logging.getLogger("spot_wrapper")
        self._frame_prefix = ""

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

        current_time = rospy.Time(5, 6)
        transform_stamped = ros_helpers.populateTransformStamped(
            current_time, "parent_frame", "child_frame", transform
        )
        self.assertEqual(transform_stamped.header.stamp, current_time)
        self.assertEqual(transform_stamped.header.frame_id, "parent_frame")
        self.assertEqual(transform_stamped.child_frame_id, "child_frame")
        self.assertEqual(transform_stamped.transform, transform)


class TestGetImageMsg(unittest.TestCase):
    @property
    def test_get_image_msg(self):
        spot_wrapper = TestSpotWrapper()

        # Test GetImageMsg with a valid image
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.data = b"test"
        image_response.shot.image.cols = 640
        image_response.shot.image.rows = 480
        image_response.shot.image.format = image_pb2.Image.FORMAT_JPEG
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
        image_response.shot.frame_name_image_sensor = "frontleft_fisheye_image"

        image, camera_info = ros_helpers.GetImageMsg(image_response, spot_wrapper)
        self.assertEqual(image.cols, 640)
        self.assertEqual(image.rows, 480)
        self.assertEqual(image.pixel_format, 3)
        self.assertEqual(image.encoding, "rgb8")
        self.assertEqual(image.is_bigendian, True)
        self.assertEqual(image.step, 3 * 640)
        self.assertEqual(image.data, image_response.shot.image.data)
        self.assertEqual(camera_info.header.frame_id, "frontleft_fisheye_image")
        self.assertEqual(camera_info.cols, 640)
        self.assertEqual(camera_info.rows, 480)


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
        self.assertEqual(joint_state.name[0], "front_left_hip_x")
        self.assertEqual(joint_state.position[0], 1.0)
        self.assertEqual(joint_state.velocity[0], 2.0)
        self.assertEqual(joint_state.effort[0], 4.0)
        self.assertEqual(joint_state.name[1], "front_left_hip_y")
        self.assertEqual(joint_state.position[1], 5.0)
        self.assertEqual(joint_state.velocity[1], 6.0)
        self.assertEqual(joint_state.effort[1], 8.0)
        self.assertEqual(joint_state.header.stamp.secs, 30)
        self.assertEqual(joint_state.header.stamp.nsecs, 100)


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

        self.assertEqual(estop_state_array.estop_states[0].name, "estop1")
        self.assertEqual(
            estop_state_array.estop_states[0].state,
            robot_state_pb2.EStopState.STATE_ESTOPPED,
        )
        self.assertEqual(
            estop_state_array.estop_states[0].type,
            robot_state_pb2.EStopState.TYPE_HARDWARE,
        )
        self.assertEqual(estop_state_array.estop_states[0].header.stamp.secs, 30)
        self.assertEqual(estop_state_array.estop_states[0].header.stamp.nsecs, 10)

        self.assertEqual(estop_state_array.estop_states[1].name, "estop2")
        self.assertEqual(
            estop_state_array.estop_states[1].state,
            robot_state_pb2.EStopState.STATE_NOT_ESTOPPED,
        )
        self.assertEqual(
            estop_state_array.estop_states[1].type,
            robot_state_pb2.EStopState.TYPE_SOFTWARE,
        )
        self.assertEqual(estop_state_array.estop_states[1].header.stamp.secs, 20)
        self.assertEqual(estop_state_array.estop_states[1].header.stamp.nsecs, 15)


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
        self.assertEqual(foot_state_array.states[0].foot_position_rt_body.y, 2.0)
        self.assertEqual(foot_state_array.states[0].foot_position_rt_body.z, 3.0)
        self.assertEqual(foot_state_array.states[0].foot_position_rt_body.x, 1.0)
        self.assertEqual(
            foot_state_array.states[0].contact, robot_state_pb2.FootState.CONTACT_MADE
        )
        self.assertEqual(foot_state_array.states[0].terrain.ground_mu_est, 0.5)
        self.assertEqual(foot_state_array.states[0].terrain.frame_name, "frame1")
        self.assertEqual(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.x, 1.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.y, 2.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.z, 3.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.x, 4.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.y, 5.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.z, 6.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.x, 7.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.y, 8.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.z, 9.0
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.visual_surface_ground_penetration_mean,
            0.1,
        )
        self.assertEqual(
            foot_state_array.states[0].terrain.visual_surface_ground_penetration_std,
            0.02,
        )

        self.assertEqual(foot_state_array.states[1].foot_position_rt_body.y, 5.0)
        self.assertEqual(foot_state_array.states[1].foot_position_rt_body.z, 6.0)
        self.assertEqual(foot_state_array.states[1].foot_position_rt_body.x, 4.0)
        self.assertEqual(
            foot_state_array.states[1].contact, robot_state_pb2.FootState.CONTACT_LOST
        )
        self.assertEqual(foot_state_array.states[1].terrain.ground_mu_est, 0.6)
        self.assertEqual(foot_state_array.states[1].terrain.frame_name, "frame2")
        self.assertEqual(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.x, 10.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.y, 11.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.z, 12.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.x, 13.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.y, 14.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.z, 15.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.x, 16.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.y, 17.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.z, 18.0
        )
        self.assertEqual(
            foot_state_array.states[1].terrain.visual_surface_ground_penetration_mean,
            0.2,
        )
        self.assertEqual(
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
        self.assertEqual(twist_odom_msg.twist.twist.linear.x, 1.0)
        self.assertEqual(twist_odom_msg.twist.twist.linear.y, 2.0)
        self.assertEqual(twist_odom_msg.twist.twist.linear.z, 3.0)
        self.assertEqual(twist_odom_msg.twist.twist.angular.x, 4.0)
        self.assertEqual(twist_odom_msg.twist.twist.angular.y, 5.0)
        self.assertEqual(twist_odom_msg.twist.twist.angular.z, 6.0)


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
        self.assertEqual(
            odometry_msg.pose.pose.position.x, -vision_tform_example.position.x
        )
        self.assertEqual(
            odometry_msg.pose.pose.position.y, -vision_tform_example.position.y
        )
        self.assertEqual(
            odometry_msg.pose.pose.position.z, -vision_tform_example.position.z
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.x, vision_tform_example.rotation.x
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.y, vision_tform_example.rotation.y
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.z, vision_tform_example.rotation.z
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.w, vision_tform_example.rotation.w
        )

        # Test with odom frame transformation
        odometry_msg = ros_helpers.GetOdomFromState(state, spot_wrapper, False)
        self.assertEqual(
            odometry_msg.pose.pose.position.x, -body_tform_example.position.x
        )
        self.assertEqual(
            odometry_msg.pose.pose.position.y, -body_tform_example.position.y
        )
        self.assertEqual(
            odometry_msg.pose.pose.position.z, -body_tform_example.position.z
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.x, body_tform_example.rotation.x
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.y, body_tform_example.rotation.y
        )
        self.assertEqual(
            odometry_msg.pose.pose.orientation.z, body_tform_example.rotation.z
        )
        self.assertEqual(
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
        self.assertEqual(
            wifi_state.current_mode, robot_state_pb2.WiFiState.MODE_ACCESS_POINT
        )
        self.assertEqual(wifi_state.essid, "test_essid")


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
        tf_message = ros_helpers.GenerateFeetTF(foot_state_msg, rospy.Time(4, 5))
        self.assertEqual(len(tf_message.transforms), 4)
        self.assertEqual(tf_message.transforms[0].transform.translation.x, 1.0)
        self.assertEqual(tf_message.transforms[0].transform.translation.y, 2.0)
        self.assertEqual(tf_message.transforms[0].transform.translation.z, 3.0)
        self.assertEqual(tf_message.transforms[0].header.frame_id, "body")
        self.assertEqual(tf_message.transforms[0].child_frame_id, "front_left_foot")
        self.assertEqual(tf_message.transforms[1].transform.translation.x, 4.0)
        self.assertEqual(tf_message.transforms[1].transform.translation.y, 5.0)
        self.assertEqual(tf_message.transforms[1].transform.translation.z, 6.0)
        self.assertEqual(tf_message.transforms[1].header.frame_id, "body")
        self.assertEqual(tf_message.transforms[1].child_frame_id, "front_right_foot")
        self.assertEqual(tf_message.transforms[2].transform.translation.x, 7.0)
        self.assertEqual(tf_message.transforms[2].transform.translation.y, 8.0)
        self.assertEqual(tf_message.transforms[2].transform.translation.z, 9.0)
        self.assertEqual(tf_message.transforms[2].header.frame_id, "body")
        self.assertEqual(tf_message.transforms[2].child_frame_id, "rear_left_foot")
        self.assertEqual(tf_message.transforms[3].transform.translation.x, 10.0)
        self.assertEqual(tf_message.transforms[3].transform.translation.y, 11.0)
        self.assertEqual(tf_message.transforms[3].transform.translation.z, 12.0)
        self.assertEqual(tf_message.transforms[3].header.frame_id, "body")
        self.assertEqual(tf_message.transforms[3].child_frame_id, "rear_right_foot")


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

        transforms = sorted(tf_message.transforms, key=lambda x: x.child_frame_id)

        self.assertEqual(len(transforms), 2)
        self.assertEqual(transforms[0].header.frame_id, "body")
        self.assertEqual(transforms[0].child_frame_id, "odom")
        self.assertEqual(transforms[0].transform.translation.x, -2.0)
        self.assertEqual(transforms[0].transform.translation.y, -3.0)
        self.assertEqual(transforms[0].transform.translation.z, -2.0)

        self.assertEqual(transforms[1].header.frame_id, "body")
        self.assertEqual(transforms[1].child_frame_id, "vision")
        self.assertEqual(transforms[1].transform.translation.x, 2.0)
        self.assertEqual(transforms[1].transform.translation.y, 3.0)
        self.assertEqual(transforms[1].transform.translation.z, 2.0)

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

        transforms = list(
            reversed(sorted(tf_message.transforms, key=lambda x: x.header.frame_id))
        )

        self.assertEqual(len(transforms), 3)
        self.assertEqual(transforms[0].header.frame_id, "vision")
        self.assertEqual(transforms[0].child_frame_id, "body")
        self.assertEqual(transforms[0].transform.translation.x, -4.0)
        self.assertEqual(transforms[0].transform.translation.y, -5.0)
        self.assertEqual(transforms[0].transform.translation.z, -6.0)

        self.assertEqual(transforms[1].header.frame_id, "special_frame")
        self.assertEqual(transforms[1].child_frame_id, "body")
        self.assertEqual(transforms[1].transform.translation.x, 7.0)
        self.assertEqual(transforms[1].transform.translation.y, 8.0)
        self.assertEqual(transforms[1].transform.translation.z, 9.0)

        self.assertEqual(transforms[2].header.frame_id, "body")
        self.assertEqual(transforms[2].child_frame_id, "odom")
        self.assertEqual(transforms[2].transform.translation.x, -2.0)
        self.assertEqual(transforms[2].transform.translation.y, -3.0)
        self.assertEqual(transforms[2].transform.translation.z, -4.0)


class TestGetBatteryStatesFromState(unittest.TestCase):
    def test_get_battery_states_from_state_zero(self):
        state = robot_state_pb2.RobotState()
        spot_wrapper = TestSpotWrapper()

        # Test with no battery states
        battery_states = ros_helpers.GetBatteryStatesFromState(state, spot_wrapper)
        self.assertEqual(len(battery_states.battery_states), 0)

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
        self.assertEqual(len(battery_states.battery_states), 1)
        self.assertEqual(battery_states.battery_states[0].header.stamp.secs, 1)
        self.assertEqual(battery_states.battery_states[0].header.stamp.nsecs, 2)
        self.assertEqual(battery_states.battery_states[0].identifier, "battery1")
        self.assertEqual(battery_states.battery_states[0].charge_percentage, 95.0)
        self.assertEqual(battery_states.battery_states[0].estimated_runtime.secs, 100)
        self.assertEqual(battery_states.battery_states[0].current, 10.0)
        self.assertEqual(battery_states.battery_states[0].voltage, 9.0)
        self.assertEqual(
            battery_states.battery_states[0].temperatures, [25.0, 26.0, 27.0]
        )
        self.assertEqual(
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
        self.assertEqual(len(battery_states.battery_states), 2)
        self.assertEqual(battery_states.battery_states[0].header.stamp.secs, 1)
        self.assertEqual(battery_states.battery_states[0].header.stamp.nsecs, 2)
        self.assertEqual(battery_states.battery_states[0].identifier, "battery1")
        self.assertEqual(battery_states.battery_states[0].charge_percentage, 95.0)
        self.assertEqual(battery_states.battery_states[0].estimated_runtime.secs, 100)
        self.assertEqual(battery_states.battery_states[0].current, 10.0)
        self.assertEqual(battery_states.battery_states[0].voltage, 9.0)
        self.assertEqual(
            battery_states.battery_states[0].temperatures, [25.0, 26.0, 27.0]
        )
        self.assertEqual(
            battery_states.battery_states[0].status,
            robot_state_pb2.BatteryState.STATUS_DISCHARGING,
        )
        self.assertEqual(battery_states.battery_states[1].header.stamp.secs, 3)
        self.assertEqual(battery_states.battery_states[1].header.stamp.nsecs, 4)
        self.assertEqual(battery_states.battery_states[1].identifier, "battery2")
        self.assertEqual(battery_states.battery_states[1].charge_percentage, 5.0)
        self.assertEqual(battery_states.battery_states[1].estimated_runtime.secs, 10)
        self.assertEqual(battery_states.battery_states[1].current, 5.0)
        self.assertEqual(battery_states.battery_states[1].voltage, 10.0)
        self.assertEqual(
            battery_states.battery_states[1].temperatures, [35.0, 36.0, 37.0]
        )
        self.assertEqual(
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
        self.assertEqual(
            power_state_msg.motor_power_state,
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_OFF,
        )
        self.assertEqual(
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
        self.assertEqual(
            power_state_msg.motor_power_state,
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_ON,
        )
        self.assertEqual(
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
        self.assertEqual(
            dock_state_msg.status, docking_pb2.DockState.DOCK_STATUS_DOCKED
        )
        self.assertEqual(dock_state_msg.dock_type, docking_pb2.DOCK_TYPE_SPOT_DOCK)
        self.assertEqual(dock_state_msg.dock_id, 3)
        self.assertEqual(
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
        self.assertEqual(behavior_faults[0].behavior_fault_id, 1)
        self.assertEqual(behavior_faults[0].header.stamp.secs, 1)
        self.assertEqual(behavior_faults[0].header.stamp.nsecs, 2)
        self.assertEqual(
            behavior_faults[0].cause, robot_state_pb2.BehaviorFault.CAUSE_FALL
        )
        self.assertEqual(
            behavior_faults[0].status, robot_state_pb2.BehaviorFault.STATUS_UNCLEARABLE
        )

        self.assertEqual(behavior_faults[1].behavior_fault_id, 3)
        self.assertEqual(behavior_faults[1].header.stamp.secs, 4)
        self.assertEqual(behavior_faults[1].header.stamp.nsecs, 5)
        self.assertEqual(
            behavior_faults[1].cause, robot_state_pb2.BehaviorFault.CAUSE_LEASE_TIMEOUT
        )
        self.assertEqual(
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
        self.assertEqual(behavior_faults.faults[0].behavior_fault_id, 1)
        self.assertEqual(behavior_faults.faults[0].header.stamp.secs, 1)
        self.assertEqual(behavior_faults.faults[0].header.stamp.nsecs, 2)
        self.assertEqual(
            behavior_faults.faults[0].cause, robot_state_pb2.BehaviorFault.CAUSE_FALL
        )
        self.assertEqual(
            behavior_faults.faults[0].status,
            robot_state_pb2.BehaviorFault.STATUS_UNCLEARABLE,
        )

        self.assertEqual(behavior_faults.faults[1].behavior_fault_id, 3)
        self.assertEqual(behavior_faults.faults[1].header.stamp.secs, 4)
        self.assertEqual(behavior_faults.faults[1].header.stamp.nsecs, 5)
        self.assertEqual(
            behavior_faults.faults[1].cause,
            robot_state_pb2.BehaviorFault.CAUSE_LEASE_TIMEOUT,
        )
        self.assertEqual(
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
        self.assertEqual(system_faults[0].name, "fault1")
        self.assertEqual(system_faults[0].header.stamp.secs, 1)
        self.assertEqual(system_faults[0].header.stamp.nsecs, 2)
        self.assertEqual(system_faults[0].duration.secs, 3)
        self.assertEqual(system_faults[0].duration.nsecs, 4)
        self.assertEqual(system_faults[0].code, 42)
        self.assertEqual(system_faults[0].uid, 5)
        self.assertEqual(system_faults[0].error_message, "error message1")
        self.assertEqual(system_faults[0].attributes, ["imu", "power"])
        self.assertEqual(
            system_faults[0].severity, robot_state_pb2.SystemFault.SEVERITY_WARN
        )

        self.assertEqual(system_faults[1].name, "fault2")
        self.assertEqual(system_faults[1].header.stamp.secs, 6)
        self.assertEqual(system_faults[1].header.stamp.nsecs, 7)
        self.assertEqual(system_faults[1].duration.secs, 8)
        self.assertEqual(system_faults[1].duration.nsecs, 9)
        self.assertEqual(system_faults[1].code, 43)
        self.assertEqual(system_faults[1].uid, 10)
        self.assertEqual(system_faults[1].error_message, "error message2")
        self.assertEqual(system_faults[1].attributes, ["wifi", "vision"])
        self.assertEqual(
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
        self.assertEqual(system_faults.faults[0].name, "fault1")
        self.assertEqual(system_faults.faults[0].header.stamp.secs, 1)
        self.assertEqual(system_faults.faults[0].header.stamp.nsecs, 2)
        self.assertEqual(system_faults.faults[0].duration.secs, 3)
        self.assertEqual(system_faults.faults[0].duration.nsecs, 4)
        self.assertEqual(system_faults.faults[0].code, 42)
        self.assertEqual(system_faults.faults[0].uid, 5)
        self.assertEqual(system_faults.faults[0].error_message, "error message1")
        self.assertEqual(system_faults.faults[0].attributes, ["imu", "power"])
        self.assertEqual(
            system_faults.faults[0].severity, robot_state_pb2.SystemFault.SEVERITY_WARN
        )

        self.assertEqual(system_faults.historical_faults[0].name, "fault2")
        self.assertEqual(system_faults.historical_faults[0].header.stamp.secs, 6)
        self.assertEqual(system_faults.historical_faults[0].header.stamp.nsecs, 7)
        self.assertEqual(system_faults.historical_faults[0].duration.secs, 8)
        self.assertEqual(system_faults.historical_faults[0].duration.nsecs, 9)
        self.assertEqual(system_faults.historical_faults[0].code, 43)
        self.assertEqual(system_faults.historical_faults[0].uid, 10)
        self.assertEqual(
            system_faults.historical_faults[0].error_message, "error message2"
        )
        self.assertEqual(
            system_faults.historical_faults[0].attributes, ["wifi", "vision"]
        )
        self.assertEqual(
            system_faults.historical_faults[0].severity,
            robot_state_pb2.SystemFault.SEVERITY_CRITICAL,
        )


class TestGetSpotCheckResultsMsg(unittest.TestCase):
    def test_get_spot_check_result_msg_success(self):
        """Test that the SpotCheckMsg is correctly populated."""
        camera_results = {
            "frontright": spot_check_pb2.DepthPlaneSpotCheckResult(
                status=spot_check_pb2.DepthPlaneSpotCheckResult.STATUS_OK,
                severity_score=11.0,
            ),
            "frontleft": spot_check_pb2.DepthPlaneSpotCheckResult(
                status=spot_check_pb2.DepthPlaneSpotCheckResult.STATUS_OK,
                severity_score=9.0,
            ),
            "left": spot_check_pb2.DepthPlaneSpotCheckResult(
                status=spot_check_pb2.DepthPlaneSpotCheckResult.STATUS_OK,
                severity_score=8.0,
            ),
            "right": spot_check_pb2.DepthPlaneSpotCheckResult(
                status=spot_check_pb2.DepthPlaneSpotCheckResult.STATUS_OK,
                severity_score=7.0,
            ),
            "back": spot_check_pb2.DepthPlaneSpotCheckResult(
                status=spot_check_pb2.DepthPlaneSpotCheckResult.STATUS_OK,
                severity_score=6.0,
            ),
        }
        load_cell_results = {
            "fl.hxa": spot_check_pb2.LoadCellSpotCheckResult(
                error=spot_check_pb2.LoadCellSpotCheckResult.ERROR_NONE,
                zero=0.1,
                old_zero=0.2,
            ),
            "fl.hya": spot_check_pb2.LoadCellSpotCheckResult(
                error=spot_check_pb2.LoadCellSpotCheckResult.ERROR_NONE,
                zero=0.3,
                old_zero=0.4,
            ),
        }
        kinematic_cal_results = {
            "fl.hx": spot_check_pb2.JointKinematicCheckResult(
                error=spot_check_pb2.JointKinematicCheckResult.ERROR_NONE,
                offset=0.1,
                old_offset=0.2,
                health_score=0.9,
            ),
            "fl.hy": spot_check_pb2.JointKinematicCheckResult(
                error=spot_check_pb2.JointKinematicCheckResult.ERROR_NONE,
                offset=0.05,
                old_offset=0.3,
                health_score=0.8,
            ),
        }
        payload_result = spot_check_pb2.PayloadCheckResult(
            error=spot_check_pb2.PayloadCheckResult.ERROR_NONE, extra_payload=5.1
        )
        hip_range_of_motion_results = {
            "fl.hx": spot_check_pb2.HipRangeOfMotionResult(
                error=spot_check_pb2.HipRangeOfMotionResult.ERROR_NONE,
                hx=[0.1, 0.2, 0.3],
                hy=[0.4, 0.5, 0.6],
            )
        }
        progress = 1.0
        last_cal_timestamp = timestamp_pb2.Timestamp(seconds=1, nanos=2)

        spot_check_data = spot_check_pb2.SpotCheckFeedbackResponse(
            camera_results=camera_results,
            load_cell_results=load_cell_results,
            kinematic_cal_results=kinematic_cal_results,
            payload_result=payload_result,
            hip_range_of_motion_results=hip_range_of_motion_results,
            last_cal_timestamp=last_cal_timestamp,
            progress=progress,
        )

        resp = [True, "Spot check test run successfully"]

        spot_check_msg = ros_helpers.GetSpotCheckResultsMsg(spot_check_data, resp)

        # Check that the spot check message is correctly populated
        self.assertEqual(spot_check_msg.progress, 1.0)
        self.assertEqual(spot_check_msg.last_cal_timestamp.secs, 1)
        self.assertEqual(spot_check_msg.last_cal_timestamp.nsecs, 2)

        for i in range(len(spot_check_msg.camera_results)):
            self.assertEqual(
                spot_check_msg.camera_results[i],
                camera_results[spot_check_msg.camera_names[i]],
                f"Camera result for {spot_check_msg.camera_names[i]} does not match",
            )

        for i in range(len(spot_check_msg.load_cell_results)):
            self.assertEqual(
                spot_check_msg.load_cell_results[i],
                load_cell_results[spot_check_msg.load_cell_names[i]],
                f"Load cell result for {spot_check_msg.load_cell_names[i]} does not match",
            )

        for i in range(len(spot_check_msg.kinematic_cal_results)):
            self.assertEqual(
                spot_check_msg.kinematic_cal_results[i],
                kinematic_cal_results[spot_check_msg.kinematic_joint_names[i]],
                f"Kinematic cal result for {spot_check_msg.kinematic_joint_names[i]} does not match",
            )

        for i in range(len(spot_check_msg.hip_range_of_motion_results)):
            self.assertEqual(
                spot_check_msg.hip_range_of_motion_results[i],
                hip_range_of_motion_results[spot_check_msg.leg_names[i]],
                f"Hip range of motion result for {spot_check_msg.leg_names[i]} does not match",
            )

        self.assertEqual(spot_check_msg.payload_result, payload_result)

        # Check that the response is correctly populated
        self.assertEqual(spot_check_msg.success, True)
        self.assertEqual(spot_check_msg.message, "Spot check test run successfully")


class TestGetPointCloudMsg(unittest.TestCase):
    def test_point_cloud_msg(self):
        # Create TestSpotWrapper
        spot_wrapper = TestSpotWrapper()

        # Create PointCloudResponse data
        point_cloud_data = point_cloud_pb2.PointCloudResponse()
        point_cloud_data.status = point_cloud_pb2.PointCloudResponse.STATUS_OK
        point_cloud = point_cloud_pb2.PointCloud(
            source=point_cloud_pb2.PointCloudSource(
                name="test_point_cloud",
                frame_name_sensor="eap",
                acquisition_time=timestamp_pb2.Timestamp(seconds=1, nanos=2),
                transforms_snapshot=None,
            ),
            num_points=3,
            encoding=point_cloud_pb2.PointCloud.ENCODING_XYZ_32F,
            encoding_parameters=None,
            data=b"\x00\x00\x80?\x00\x00\x00@\x00\x00\x80@\x00\x00\x80?\x00\x00\x00@\x00\x00\x80@\x00\x00\x80?\x00\x00\x00@\x00\x00\x80@",
        )
        point_cloud_data.point_cloud.CopyFrom(point_cloud)

        # Create a point cloud message
        point_cloud_msg = ros_helpers.GetPointCloudMsg(point_cloud_data, spot_wrapper)

        # Check that the point cloud message is correctly populated
        self.assertEqual(point_cloud_msg.header.frame_id, "eap")
        self.assertEqual(point_cloud_msg.header.stamp.secs, 1)
        self.assertEqual(point_cloud_msg.header.stamp.nsecs, 2)
        self.assertEqual(point_cloud_msg.height, 1)
        self.assertEqual(point_cloud_msg.width, 3)
        self.assertEqual(point_cloud_msg.fields[0].name, "x")
        self.assertEqual(point_cloud_msg.fields[0].offset, 0)
        self.assertEqual(point_cloud_msg.fields[0].datatype, 7)
        self.assertEqual(point_cloud_msg.fields[0].count, 1)
        self.assertEqual(point_cloud_msg.fields[1].name, "y")
        self.assertEqual(point_cloud_msg.fields[1].offset, 4)
        self.assertEqual(point_cloud_msg.fields[1].datatype, 7)
        self.assertEqual(point_cloud_msg.fields[1].count, 1)
        self.assertEqual(point_cloud_msg.fields[2].name, "z")
        self.assertEqual(point_cloud_msg.fields[2].offset, 8)
        self.assertEqual(point_cloud_msg.fields[2].datatype, 7)
        self.assertEqual(point_cloud_msg.fields[2].count, 1)
        self.assertEqual(point_cloud_msg.is_bigendian, False)
        self.assertEqual(point_cloud_msg.point_step, 12)
        self.assertEqual(point_cloud_msg.row_step, 36)
        self.assertEqual(
            point_cloud_msg.data,
            b"\x00\x00\x80?\x00\x00\x00@\x00\x00\x80@\x00\x00\x80?\x00\x00\x00@\x00\x00\x80@\x00\x00\x80?\x00\x00\x00@\x00\x00\x80@",
        )
        self.assertEqual(point_cloud_msg.is_dense, True)


class TestGetFrameTreeSnapshotMsg(unittest.TestCase):
    def test_get_frame_tree_snapshot_msg(self):
        # Create FrameTreeSnapshot data
        data = geometry_pb2.FrameTreeSnapshot()
        child_parent_map = {
            "sensor": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                parent_frame_name="body",
                parent_tform_child=geometry_pb2.SE3Pose(
                    position=geometry_pb2.Vec3(x=1.0, y=2.0, z=3.0),
                    rotation=geometry_pb2.Quaternion(x=7.0, y=8.0, z=9.0, w=1.0),
                ),
            ),
            "eap": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                parent_frame_name="body",
                parent_tform_child=geometry_pb2.SE3Pose(
                    position=geometry_pb2.Vec3(x=4.0, y=5.0, z=6.0),
                    rotation=geometry_pb2.Quaternion(x=10.0, y=11.0, z=12.0, w=13.0),
                ),
            ),
        }
        data.child_to_parent_edge_map["eap"].CopyFrom(child_parent_map["eap"])
        data.child_to_parent_edge_map["sensor"].CopyFrom(child_parent_map["sensor"])

        # Create a frame tree snapshot message
        frame_tree_snapshot_msg: FrameTreeSnapshot = (
            ros_helpers.GetFrameTreeSnapshotMsg(data)
        )

        # Check that the frame tree snapshot message is correctly populated
        self.assertEqual(frame_tree_snapshot_msg.child_edges[0], "sensor")
        self.assertEqual(frame_tree_snapshot_msg.child_edges[1], "eap")
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_frame_name, "body"
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.position.x, 1.0
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.position.y, 2.0
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.position.z, 3.0
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.orientation.x,
            7.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.orientation.y,
            8.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.orientation.z,
            9.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[0].parent_tform_child.orientation.w,
            1.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_frame_name, "body"
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.position.x, 4.0
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.position.y, 5.0
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.position.z, 6.0
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.orientation.x,
            10.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.orientation.y,
            11.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.orientation.z,
            12.0,
        )
        self.assertEqual(
            frame_tree_snapshot_msg.parent_edges[1].parent_tform_child.orientation.w,
            13.0,
        )


class TestGetAprilTagPropertiesMsg(unittest.TestCase):
    def test_get_apriltag_properties_msg(self):
        # Create AprilTagProperties data
        data = world_object_pb2.AprilTagProperties()
        data.tag_id = 1
        data.dimensions.x = 2.0
        data.dimensions.y = 3.0
        data.frame_name_fiducial = "AprilTag20"
        data.fiducial_pose_status = world_object_pb2.AprilTagProperties.STATUS_OK
        data.frame_name_fiducial_filtered = "AprilTag20_filtered"
        data.fiducial_filtered_pose_status = (
            world_object_pb2.AprilTagProperties.STATUS_OK
        )
        data.frame_name_camera = "frontleft_fisheye_image"
        data.detection_covariance.CopyFrom(
            geometry_pb2.SE3Covariance(
                matrix=geometry_pb2.Matrix(
                    rows=6,
                    cols=6,
                    values=[float(i) for i in range(36)],
                )
            )
        )
        data.detection_covariance_reference_frame = "frontleft_fisheye_image"

        # Create an AprilTagProperties message
        apriltag_properties_msg: AprilTagProperties = (
            ros_helpers.GetAprilTagPropertiesMsg(data)
        )

        # Check that the AprilTagProperties message is correctly populated
        self.assertEqual(apriltag_properties_msg.tag_id, 1)
        self.assertEqual(apriltag_properties_msg.x, 2.0)
        self.assertEqual(apriltag_properties_msg.y, 3.0)
        self.assertEqual(apriltag_properties_msg.frame_name_fiducial, "AprilTag20")
        self.assertEqual(
            apriltag_properties_msg.fiducial_pose_status,
            AprilTagProperties.STATUS_OK,
        )
        self.assertEqual(
            apriltag_properties_msg.frame_name_fiducial_filtered,
            "AprilTag20_filtered",
        )
        self.assertEqual(
            apriltag_properties_msg.fiducial_filtered_pose_status,
            AprilTagProperties.STATUS_OK,
        )
        self.assertEqual(
            apriltag_properties_msg.frame_name_camera,
            "frontleft_fisheye_image",
        )
        self.assertEqual(
            apriltag_properties_msg.detection_covariance.covariance,
            [float(i) for i in range(36)],
        )
        self.assertEqual(
            apriltag_properties_msg.detection_covariance_reference_frame,
            "frontleft_fisheye_image",
        )


class TestGetImagePropertiesMsg(unittest.TestCase):
    def test_get_image_properties_msg(self):
        # Create TestSpotWrapper
        spot_wrapper = TestSpotWrapper()

        # Create ImageProperties data
        data = world_object_pb2.ImageProperties()
        data.camera_source = "frontleft"
        data.coordinates.CopyFrom(
            geometry_pb2.Polygon(
                vertexes=[
                    geometry_pb2.Vec2(x=1.0, y=2.0),
                    geometry_pb2.Vec2(x=3.0, y=4.0),
                ]
            )
        )
        data.image_source.CopyFrom(
            image_pb2.ImageSource(
                name="frontleft_fisheye_image",
                cols=640,
                rows=480,
                depth_scale=1.0,
                pinhole=image_pb2.ImageSource.PinholeModel(
                    intrinsics=image_pb2.ImageSource.PinholeModel.CameraIntrinsics(
                        focal_length=geometry_pb2.Vec2(x=1.0, y=2.0),
                        principal_point=geometry_pb2.Vec2(x=3.0, y=4.0),
                        skew=geometry_pb2.Vec2(x=5.0, y=6.0),
                    )
                ),
                image_type=image_pb2.ImageSource.IMAGE_TYPE_VISUAL,
                pixel_formats=[image_pb2.Image.PIXEL_FORMAT_RGB_U8],
                image_formats=[image_pb2.Image.FORMAT_RAW],
            )
        )

        data.image_capture.CopyFrom(
            image_pb2.ImageCapture(
                acquisition_time=timestamp_pb2.Timestamp(seconds=1, nanos=2),
                transforms_snapshot=geometry_pb2.FrameTreeSnapshot(),
                frame_name_image_sensor="frontleft_fisheye_image",
                image=image_pb2.Image(
                    cols=640,
                    rows=480,
                    data=b"image_data",
                    format=image_pb2.Image.FORMAT_RAW,
                    pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                ),
                capture_params=image_pb2.CaptureParameters(
                    exposure_duration=duration_pb2.Duration(seconds=1, nanos=2),
                    gain=3.0,
                ),
            )
        )
        data.frame_name_image_coordinates = "frontleft"

        # Create an ImageProperties message
        image_properties_msg: ImageProperties = ros_helpers.GetImagePropertiesMsg(
            data, spot_wrapper
        )

        # Check that the ImageProperties message is correctly populated
        self.assertEqual(image_properties_msg.camera_source, "frontleft")
        self.assertEqual(image_properties_msg.image_data_coordinates.points[0].x, 1.0)
        self.assertEqual(image_properties_msg.image_data_coordinates.points[0].y, 2.0)
        self.assertEqual(image_properties_msg.image_data_coordinates.points[1].x, 3.0)
        self.assertEqual(image_properties_msg.image_data_coordinates.points[1].y, 4.0)
        self.assertEqual(
            image_properties_msg.image_source.name, "frontleft_fisheye_image"
        )
        self.assertEqual(image_properties_msg.image_source.cols, 640)
        self.assertEqual(image_properties_msg.image_source.rows, 480)
        self.assertEqual(image_properties_msg.image_source.depth_scale, 1.0)
        self.assertEqual(image_properties_msg.image_source.focal_length_x, 1.0)
        self.assertEqual(image_properties_msg.image_source.focal_length_y, 2.0)
        self.assertEqual(image_properties_msg.image_source.principal_point_x, 3.0)
        self.assertEqual(image_properties_msg.image_source.principal_point_y, 4.0)
        self.assertEqual(image_properties_msg.image_source.skew_x, 5.0)
        self.assertEqual(image_properties_msg.image_source.skew_y, 6.0)
        self.assertEqual(
            image_properties_msg.image_source.image_type,
            ImageSource.IMAGE_TYPE_VISUAL,
        )
        self.assertEqual(
            image_properties_msg.image_source.pixel_formats[0],
            ImageSource.PIXEL_FORMAT_RGB_U8,
        )
        self.assertEqual(
            image_properties_msg.image_source.image_formats[0],
            ImageSource.FORMAT_RAW,
        )

        self.assertEqual(image_properties_msg.image_capture.acquisition_time.secs, 1)
        self.assertEqual(image_properties_msg.image_capture.acquisition_time.nsecs, 2)
        self.assertEqual(
            image_properties_msg.image_capture.frame_name_image_sensor,
            "frontleft_fisheye_image",
        )
        self.assertEqual(image_properties_msg.image_capture.image.height, 480)
        self.assertEqual(image_properties_msg.image_capture.image.width, 640)
        self.assertEqual(image_properties_msg.image_capture.image.data, b"image_data")
        self.assertEqual(
            image_properties_msg.image_capture.image.encoding,
            "rgb8",
        )
        self.assertEqual(
            image_properties_msg.image_capture.capture_exposure_duration.secs, 1.0
        )
        self.assertEqual(
            image_properties_msg.image_capture.capture_exposure_duration.nsecs, 2.0
        )
        self.assertEqual(image_properties_msg.image_capture.capture_sensor_gain, 3.0)

        self.assertEqual(image_properties_msg.frame_name_image_coordinates, "frontleft")


class TestGetWorldObjectsMsg(unittest.TestCase):
    def setUp(self):
        # Create world_object_pb2.ListWorldObjectResponse test data, similar to real data
        self.world_object = world_object_pb2.WorldObject(
            id=1,
            name="world_obj_apriltag_350",
            acquisition_time=timestamp_pb2.Timestamp(
                seconds=1678806362, nanos=176319408
            ),
            transforms_snapshot=geometry_pb2.FrameTreeSnapshot(
                child_to_parent_edge_map={
                    "vision": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                        parent_frame_name="body",
                        parent_tform_child=geometry_pb2.SE3Pose(
                            position=geometry_pb2.Vec3(x=-8.0, y=-24.0, z=-0.5),
                            rotation=geometry_pb2.Quaternion(
                                x=0.0, y=0.0, z=-1.0, w=1.0
                            ),
                        ),
                    ),
                    "odom": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                        parent_frame_name="body",
                        parent_tform_child=geometry_pb2.SE3Pose(
                            position=geometry_pb2.Vec3(x=-7.0, y=-25.0, z=0.0),
                            rotation=geometry_pb2.Quaternion(
                                x=-1.5, y=0.0, z=0.8, w=0.5
                            ),
                        ),
                    ),
                    "filtered_fiducial_350": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                        parent_frame_name="vision",
                        parent_tform_child=geometry_pb2.SE3Pose(
                            position=geometry_pb2.Vec3(x=-11.0, y=-24.0, z=0.45),
                            rotation=geometry_pb2.Quaternion(
                                x=0.47, y=0.50, z=0.51, w=-0.46
                            ),
                        ),
                    ),
                    "fiducial_350": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                        parent_frame_name="vision",
                        parent_tform_child=geometry_pb2.SE3Pose(
                            position=geometry_pb2.Vec3(x=0.27, y=-0.2, z=0.61),
                            rotation=geometry_pb2.Quaternion(
                                x=0.73, y=-0.66, z=0.0, w=-0.11
                            ),
                        ),
                    ),
                    "body": geometry_pb2.FrameTreeSnapshot.ParentEdge(
                        parent_frame_name="",
                        parent_tform_child=geometry_pb2.SE3Pose(
                            rotation=geometry_pb2.Quaternion(
                                x=0.0, y=0.0, z=0.0, w=1.0
                            ),
                        ),
                    ),
                }
            ),
            apriltag_properties=world_object_pb2.AprilTagProperties(
                tag_id=350,
                dimensions=geometry_pb2.Vec2(x=0.16, y=0.16),
                frame_name_fiducial="fiducial_350",
                fiducial_pose_status=world_object_pb2.AprilTagProperties.STATUS_OK,
                frame_name_fiducial_filtered="filtered_fiducial_350",
                fiducial_filtered_pose_status=world_object_pb2.AprilTagProperties.STATUS_OK,
                frame_name_camera="left",
                detection_covariance=geometry_pb2.SE3Covariance(
                    matrix=geometry_pb2.Matrix(
                        rows=6,
                        cols=6,
                        values=[
                            0.0,
                            0.0,
                            -3.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            1.1,
                            -9.5,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            -9.7,
                            0.0,
                            0.0,
                            -3.0,
                            1.1,
                            -9.7,
                            8.4,
                            -4.4,
                            0.0,
                            0.0,
                            -9.5,
                            0.0,
                            0.0,
                            -4.4,
                            0.0,
                            0.0,
                            8.4,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                    )
                ),
                detection_covariance_reference_frame="vision",
            ),
            image_properties=world_object_pb2.ImageProperties(
                camera_source="left",
                coordinates=geometry_pb2.Polygon(
                    vertexes=[
                        geometry_pb2.Vec2(x=19.3, y=285.3),
                        geometry_pb2.Vec2(x=26.2, y=336.1),
                        geometry_pb2.Vec2(x=79.5, y=323.8),
                        geometry_pb2.Vec2(x=74.5, y=247.8),
                    ]
                ),
            ),
            dock_properties=world_object_pb2.DockProperties(
                dock_id=50,
                type=docking_pb2.DOCK_TYPE_SPOT_DOCK,
                frame_name_dock="dock",
                unavailable=False,
                from_prior=True,
            ),
            ray_properties=world_object_pb2.RayProperties(
                frame="body",
                ray=geometry_pb2.Ray(
                    origin=geometry_pb2.Vec3(x=1.0, y=2.0, z=3.0),
                    direction=geometry_pb2.Vec3(x=4.0, y=5.0, z=6.0),
                ),
            ),
            bounding_box_properties=world_object_pb2.BoundingBoxProperties(
                frame="body",
                size_ewrt_frame=geometry_pb2.Vec3(x=1.0, y=2.0, z=3.0),
            ),
        )
        return super().setUp()

    def test_get_world_objects_msg(self):
        # Create a TestSpotWrapper
        spot_wrapper = TestSpotWrapper()

        data = world_object_pb2.ListWorldObjectResponse(
            world_objects=[self.world_object]
        )

        # Create a WorldObjects message
        world_objects_msg: WorldObjectArray = ros_helpers.GetWorldObjectsMsg(
            data, spot_wrapper
        )

        self.assertEqual(len(world_objects_msg.world_objects), 1)

        # Check that the WorldObject is correct according to the above test data
        msg_world_obj: WorldObject = world_objects_msg.world_objects[0]
        self.assertEqual(msg_world_obj.id, 1)
        self.assertEqual(msg_world_obj.name, "world_obj_apriltag_350")

        # Check that the AprilTagProperties are correct
        msg_apriltag_props: AprilTagProperties = msg_world_obj.apriltag_properties
        self.assertEqual(msg_apriltag_props.tag_id, 350)
        self.assertEqual(msg_apriltag_props.x, 0.16)
        self.assertEqual(msg_apriltag_props.y, 0.16)
        self.assertEqual(msg_apriltag_props.frame_name_fiducial, "fiducial_350")
        self.assertEqual(
            msg_apriltag_props.fiducial_pose_status, AprilTagProperties.STATUS_OK
        )
        self.assertEqual(
            msg_apriltag_props.frame_name_fiducial_filtered, "filtered_fiducial_350"
        )
        self.assertEqual(
            msg_apriltag_props.fiducial_filtered_pose_status,
            AprilTagProperties.STATUS_OK,
        )
        self.assertEqual(msg_apriltag_props.frame_name_camera, "left")
        self.assertEqual(len(msg_apriltag_props.detection_covariance.covariance), 36)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[0], 0.0)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[2], -3.0)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[9], -9.5)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[14], -9.7)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[18], 1.1)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[21], -4.4)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[24], -9.5)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[27], -4.4)
        self.assertEqual(msg_apriltag_props.detection_covariance.covariance[30], 8.4)
        self.assertEqual(
            msg_apriltag_props.detection_covariance_reference_frame, "vision"
        )

        # Check that the ImageProperties are correct
        msg_image_props: ImageProperties = msg_world_obj.image_properties
        self.assertEqual(msg_image_props.camera_source, "left")
        self.assertEqual(msg_image_props.image_data_coordinates.points[0].x, 19.3)
        self.assertEqual(msg_image_props.image_data_coordinates.points[0].y, 285.3)
        self.assertEqual(msg_image_props.image_data_coordinates.points[1].x, 26.2)
        self.assertEqual(msg_image_props.image_data_coordinates.points[1].y, 336.1)
        self.assertEqual(msg_image_props.image_data_coordinates.points[2].x, 79.5)
        self.assertEqual(msg_image_props.image_data_coordinates.points[2].y, 323.8)
        self.assertEqual(msg_image_props.image_data_coordinates.points[3].x, 74.5)
        self.assertEqual(msg_image_props.image_data_coordinates.points[3].y, 247.8)

        # Check that the DockProperties are correct
        self.assertEqual(msg_world_obj.dock_id, 50)
        self.assertEqual(msg_world_obj.dock_type, docking_pb2.DOCK_TYPE_SPOT_DOCK)
        self.assertEqual(msg_world_obj.frame_name_dock, "dock")
        self.assertEqual(msg_world_obj.dock_unavailable, False)
        self.assertEqual(msg_world_obj.from_prior_detection, True)

        # Check that the RayProperties are correct
        self.assertEqual(msg_world_obj.ray_frame, "body")
        self.assertEqual(msg_world_obj.ray_origin.x, 1.0)
        self.assertEqual(msg_world_obj.ray_origin.y, 2.0)
        self.assertEqual(msg_world_obj.ray_origin.z, 3.0)
        self.assertEqual(msg_world_obj.ray_direction.x, 4.0)
        self.assertEqual(msg_world_obj.ray_direction.y, 5.0)
        self.assertEqual(msg_world_obj.ray_direction.z, 6.0)

        # Check that the PoseProperties are correct
        self.assertEqual(msg_world_obj.bounding_box_frame, "body")
        self.assertEqual(msg_world_obj.bounding_box_size_ewrt_frame.x, 1.0)
        self.assertEqual(msg_world_obj.bounding_box_size_ewrt_frame.y, 2.0)
        self.assertEqual(msg_world_obj.bounding_box_size_ewrt_frame.z, 3.0)

    def test_get_tf_world_objects(self):
        # Test the GetTFFromWorldObjects function

        # Create a TestSpotWrapper
        spot_wrapper = TestSpotWrapper()

        # Create a WorldObject
        world_objects = [self.world_object]

        # Get the TF message
        tf_msg: TFMessage = ros_helpers.GetTFFromWorldObjects(
            world_objects, spot_wrapper, "vision"
        )

        # Check that the TF message is correct
        transforms = sorted(tf_msg.transforms, key=lambda x: x.child_frame_id)

        self.assertEqual(len(transforms), 2)
        self.assertEqual(transforms[1].header.frame_id, "vision")
        self.assertEqual(transforms[1].child_frame_id, "filtered_fiducial_350")
        self.assertAlmostEqual(transforms[1].transform.translation.x, -44.0, places=1)
        self.assertAlmostEqual(transforms[1].transform.translation.y, -96.0, places=1)
        self.assertAlmostEqual(transforms[1].transform.translation.z, 1.8, places=1)
        self.assertAlmostEqual(transforms[1].transform.rotation.x, 0.94, places=1)
        self.assertAlmostEqual(transforms[1].transform.rotation.y, 1.0, places=1)
        self.assertAlmostEqual(transforms[1].transform.rotation.z, 1.02, places=1)
        self.assertAlmostEqual(transforms[1].transform.rotation.w, -0.92, places=1)

        self.assertEqual(transforms[0].header.frame_id, "vision")
        self.assertEqual(transforms[0].child_frame_id, "fiducial_350")
        self.assertAlmostEqual(transforms[0].transform.translation.x, 1.08, places=1)
        self.assertAlmostEqual(transforms[0].transform.translation.y, -0.8, places=1)
        self.assertAlmostEqual(transforms[0].transform.translation.z, 2.44, places=1)
        self.assertAlmostEqual(transforms[0].transform.rotation.x, 1.46, places=1)
        self.assertAlmostEqual(transforms[0].transform.rotation.y, -1.3, places=1)
        self.assertAlmostEqual(transforms[0].transform.rotation.z, 0.0, places=1)
        self.assertAlmostEqual(transforms[0].transform.rotation.w, -0.22, places=1)

    def test_get_tf_world_objects_no_world_objects(self):
        # Test the GetTFFromWorldObjects function with no WorldObjects

        # Create a TestSpotWrapper
        spot_wrapper = TestSpotWrapper()

        # Create a WorldObject
        world_objects = []

        # Get the TF message
        tf_msg: TFMessage = ros_helpers.GetTFFromWorldObjects(
            world_objects, spot_wrapper, "vision"
        )

        # Check that the TF message is correct
        self.assertEqual(len(tf_msg.transforms), 0)


class TestGetBodyImageMsgs(unittest.TestCase):
    def setUp(self):
        # Make a test ImageResponse
        image_response = image_pb2.ImageResponse()
        image_response.shot.image.data = b"test"
        image_response.shot.image.cols = 640
        image_response.shot.image.rows = 480
        image_response.shot.image.format = image_pb2.Image.FORMAT_RAW
        image_response.shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
        image_response.shot.frame_name_image_sensor = "frontleft_fisheye_image"

        self.image_response = image_response

    def test_get_image_msgs(self):
        # Create TestSpotWrapper
        spot_wrapper = TestSpotWrapper()

        # Get the ImageMsgs
        image_msg, camera_info = ros_helpers.bosdyn_data_to_image_and_camera_info_msgs(
            self.image_response, spot_wrapper
        )

        # Check that the ImageMsg is correct
        self.assertEqual(image_msg.header.frame_id, "frontleft_fisheye_image")
        self.assertEqual(image_msg.height, 480)
        self.assertEqual(image_msg.width, 640)
        self.assertEqual(image_msg.encoding, "rgb8")
        self.assertEqual(image_msg.is_bigendian, True)
        self.assertEqual(image_msg.step, 1920)
        self.assertEqual(image_msg.data, b"test")

        # Check that the CameraInfoMsg is correct
        self.assertEqual(camera_info.header.frame_id, "frontleft_fisheye_image")
        self.assertEqual(camera_info.height, 480)
        self.assertEqual(camera_info.width, 640)


class TestSuiteROSHelpers(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteROSHelpers, self).__init__()
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
        self.addTest(self.loader.loadTestsFromTestCase(TestGetSpotCheckResultsMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetPointCloudMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetFrameTreeSnapshotMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetAprilTagPropertiesMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetImagePropertiesMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetWorldObjectsMsg))
        self.addTest(self.loader.loadTestsFromTestCase(TestGetBodyImageMsgs))


if __name__ == "__main__":
    print("Starting tests!")
    import rosunit

    rosunit.unitrun(PKG, NAME, TestPopulateTransformStamped)
    rosunit.unitrun(PKG, NAME, TestGetImageMsg)
    rosunit.unitrun(PKG, NAME, TestGetJointStatesFromState)
    rosunit.unitrun(PKG, NAME, TestGetEStopStateFromState)
    rosunit.unitrun(PKG, NAME, TestGetFeetFromState)
    rosunit.unitrun(PKG, NAME, TestGetOdomTwistFromState)
    rosunit.unitrun(PKG, NAME, TestGetOdomFromState)
    rosunit.unitrun(PKG, NAME, TestGetWifiFromState)
    rosunit.unitrun(PKG, NAME, TestGenerateFeetTF)
    rosunit.unitrun(PKG, NAME, TestGetTFFromState)
    rosunit.unitrun(PKG, NAME, TestGetBatteryStatesFromState)
    rosunit.unitrun(PKG, NAME, TestGetPowerStatesFromState)
    rosunit.unitrun(PKG, NAME, TestGetDockStatesFromState)
    rosunit.unitrun(PKG, NAME, TestGetBehaviorFaults)
    rosunit.unitrun(PKG, NAME, TestGetBehaviorFaultsFromState)
    rosunit.unitrun(PKG, NAME, TestGetSystemFaults)
    rosunit.unitrun(PKG, NAME, TestGetSystemFaultsFromState)
    rosunit.unitrun(PKG, NAME, TestGetSpotCheckResultsMsg)
    rosunit.unitrun(PKG, NAME, TestGetPointCloudMsg)
    rosunit.unitrun(PKG, NAME, TestGetFrameTreeSnapshotMsg)
    rosunit.unitrun(PKG, NAME, TestGetAprilTagPropertiesMsg)
    rosunit.unitrun(PKG, NAME, TestGetImagePropertiesMsg)
    rosunit.unitrun(PKG, NAME, TestGetWorldObjectsMsg)
    rosunit.unitrun(PKG, NAME, TestGetBodyImageMsgs)

    print("Tests complete!")
