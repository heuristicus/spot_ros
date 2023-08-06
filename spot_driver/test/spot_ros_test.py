#!/usr/bin/env python3
PKG = "spot_ros"
NAME = "spot_ros_test"
SUITE = "spot_ros_test.TestSuiteSpotROS"

import time
import typing
import sys
import unittest

import rospy
import actionlib
from rosservice import get_service_class_by_name

from std_msgs.msg import Duration
from std_srvs.srv import TriggerResponse
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState, PointCloud2
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray
from spot_msgs.msg import FootStateArray
from spot_msgs.msg import EStopStateArray
from spot_msgs.msg import WorldObjectArray, WorldObject
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFaultState
from spot_msgs.msg import SystemFaultState
from spot_msgs.msg import BatteryStateArray
from spot_msgs.msg import NavigateToAction, NavigateToGoal
from spot_msgs.msg import NavigateRouteAction, NavigateRouteGoal
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_msgs.msg import PoseBodyAction, PoseBodyGoal
from spot_msgs.msg import DockAction, DockGoal

from spot_msgs.srv import PosedStandRequest
from spot_msgs.srv import NavigateInitRequest, NavigateInitResponse
from spot_msgs.srv import SpotCheckRequest, SpotCheckResponse, SpotCheck
from spot_msgs.srv import ListGraphResponse
from spot_msgs.srv import (
    DockResponse,
    GetDockStateResponse,
    DockRequest,
)
from spot_msgs.srv import GripperAngleMoveResponse
from spot_msgs.srv import ArmForceTrajectoryResponse
from spot_msgs.srv import ArmJointMovementResponse
from spot_msgs.srv import HandPoseResponse
from spot_msgs.srv import Grasp3dResponse
from spot_msgs.srv import DownloadGraphResponse
from spot_msgs.srv import GraphCloseLoopsResponse

from bosdyn.api import robot_state_pb2, geometry_pb2


class TestRobotStateCB(unittest.TestCase):
    def setUp(self):
        self.data = {}

    def joint_states_cb(self, joint_state: JointState):
        self.data["joint_state"] = joint_state

    def tf_cb(self, tf: TFMessage):
        # Differentiating between foot and body TFs
        if not len(tf.transforms):
            return

        if "foot" in tf.transforms[0].child_frame_id:
            self.data["foot_TF"] = tf
        elif "fiducial" in tf.transforms[0].child_frame_id:
            self.data["fiducial_TF"] = tf
        else:
            self.data["TF"] = tf

    def twist_odom_cb(self, twist_odom: TwistWithCovarianceStamped):
        self.data["twist_odom"] = twist_odom

    def odom_cb(self, odom: Odometry):
        self.data["odom"] = odom

    def foot_cb(self, foot: FootStateArray):
        self.data["foot"] = foot

    def estop_cb(self, estop: EStopStateArray):
        self.data["estop"] = estop

    def wifi_cb(self, wifi: WiFiState):
        self.data["wifi"] = wifi

    def battery_cb(self, battery: BatteryStateArray):
        self.data["battery"] = battery

    def power_cb(self, power: PowerState):
        self.data["power"] = power

    def system_fault_cb(self, system_fault: SystemFaultState):
        self.data["system_fault"] = system_fault

    def behaviour_fault_cb(self, behaviour_fault: BehaviorFaultState):
        self.data["behaviour_fault"] = behaviour_fault

    def check_joint_states(self, joint_state: JointState):
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

    def check_foot_TF_states(self, tf_message: TFMessage):
        self.assertEqual(len(tf_message.transforms), 2)
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

    def check_TF_states(self, tf_message: TFMessage):
        transforms = sorted(tf_message.transforms, key=lambda x: x.child_frame_id)

        self.assertEqual(len(transforms), 2)
        self.assertEqual(transforms[0].header.frame_id, "vision")
        self.assertEqual(transforms[0].child_frame_id, "body")
        self.assertEqual(transforms[0].transform.translation.x, -2.0)
        self.assertEqual(transforms[0].transform.translation.y, -3.0)
        self.assertEqual(transforms[0].transform.translation.z, -2.0)

        self.assertEqual(transforms[1].header.frame_id, "body")
        self.assertEqual(transforms[1].child_frame_id, "odom")
        self.assertEqual(transforms[1].transform.translation.x, -2.0)
        self.assertEqual(transforms[1].transform.translation.y, -3.0)
        self.assertEqual(transforms[1].transform.translation.z, -2.0)

    def check_twist_odom_states(self, twist_odom_msg: TwistWithCovarianceStamped):
        self.assertEqual(twist_odom_msg.twist.twist.linear.x, 1.0)
        self.assertEqual(twist_odom_msg.twist.twist.linear.y, 2.0)
        self.assertEqual(twist_odom_msg.twist.twist.linear.z, 3.0)
        self.assertEqual(twist_odom_msg.twist.twist.angular.x, 4.0)
        self.assertEqual(twist_odom_msg.twist.twist.angular.y, 5.0)
        self.assertEqual(twist_odom_msg.twist.twist.angular.z, 6.0)

    def check_odom_states(self, odometry_msg: Odometry):
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

    def check_foot_states(self, foot_state_array: FootStateArray):
        self.assertAlmostEqual(
            foot_state_array.states[0].foot_position_rt_body.y, 2.0, places=3
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].foot_position_rt_body.z, 3.0, places=3
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].foot_position_rt_body.x, 1.0, places=3
        )
        self.assertEqual(
            foot_state_array.states[0].contact, robot_state_pb2.FootState.CONTACT_MADE
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.ground_mu_est, 0.5, places=3
        )
        self.assertEqual(foot_state_array.states[0].terrain.frame_name, "frame1")
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.x,
            1.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.y,
            2.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.foot_slip_distance_rt_frame.z,
            3.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.x,
            4.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.y,
            5.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.foot_slip_velocity_rt_frame.z,
            6.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.x,
            7.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.y,
            8.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.ground_contact_normal_rt_frame.z,
            9.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.visual_surface_ground_penetration_mean,
            0.1,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[0].terrain.visual_surface_ground_penetration_std,
            0.02,
            places=3,
        )

        self.assertAlmostEqual(
            foot_state_array.states[1].foot_position_rt_body.y, 5.0, places=3
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].foot_position_rt_body.z, 6.0, places=3
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].foot_position_rt_body.x, 4.0, places=3
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].contact, robot_state_pb2.FootState.CONTACT_LOST
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.ground_mu_est, 0.6, places=3
        )
        self.assertEqual(foot_state_array.states[1].terrain.frame_name, "frame2")
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.x,
            10.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.y,
            11.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.foot_slip_distance_rt_frame.z,
            12.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.x,
            13.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.y,
            14.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.foot_slip_velocity_rt_frame.z,
            15.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.x,
            16.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.y,
            17.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.ground_contact_normal_rt_frame.z,
            18.0,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.visual_surface_ground_penetration_mean,
            0.2,
            places=3,
        )
        self.assertAlmostEqual(
            foot_state_array.states[1].terrain.visual_surface_ground_penetration_std,
            0.03,
            places=3,
        )

    def check_estop_states(self, estop_state_array: EStopStateArray):
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

    def check_wifi_states(self, wifi_state: WiFiState):
        self.assertEqual(
            wifi_state.current_mode, robot_state_pb2.WiFiState.MODE_ACCESS_POINT
        )
        self.assertEqual(wifi_state.essid, "test_essid")

    def check_battery_states(self, battery_states: BatteryStateArray):
        self.assertEqual(len(battery_states.battery_states), 1)
        self.assertEqual(battery_states.battery_states[0].header.stamp.secs, 1)
        self.assertEqual(battery_states.battery_states[0].header.stamp.nsecs, 2)
        self.assertEqual(battery_states.battery_states[0].identifier, "battery1")
        self.assertEqual(battery_states.battery_states[0].charge_percentage, 95.0)
        self.assertEqual(battery_states.battery_states[0].estimated_runtime.secs, 100)
        self.assertEqual(battery_states.battery_states[0].current, 10.0)
        self.assertEqual(battery_states.battery_states[0].voltage, 9.0)
        self.assertAlmostEqual(
            battery_states.battery_states[0].temperatures[0], 25.0, places=3
        )
        self.assertAlmostEqual(
            battery_states.battery_states[0].temperatures[1], 26.0, places=3
        )
        self.assertAlmostEqual(
            battery_states.battery_states[0].temperatures[2], 27.0, places=3
        )
        self.assertEqual(
            battery_states.battery_states[0].status,
            robot_state_pb2.BatteryState.STATUS_DISCHARGING,
        )

    def check_power_states(self, power_state_msg: PowerState):
        self.assertEqual(
            power_state_msg.motor_power_state,
            robot_state_pb2.PowerState.MOTOR_POWER_STATE_OFF,
        )
        self.assertEqual(
            power_state_msg.shore_power_state,
            robot_state_pb2.PowerState.SHORE_POWER_STATE_ON,
        )

    def check_system_fault_states(self, system_faults: SystemFaultState):
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

    def check_behaviour_fault_states(self, behavior_faults: BehaviorFaultState):
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

    def test_robot_state_cb(self):
        # Set up a subscriber to listen to joint_states, TF, twist_odom,
        # odom, foot, estop, wifi, battery, power, system fault, behaviour fault
        self.joint_states = rospy.Subscriber(
            "joint_states", JointState, self.joint_states_cb
        )
        self.tf = rospy.Subscriber("tf", TFMessage, self.tf_cb)
        self.twist_odom = rospy.Subscriber(
            "/spot/odometry/twist", TwistWithCovarianceStamped, self.twist_odom_cb
        )
        self.odom = rospy.Subscriber("/spot/odometry", Odometry, self.odom_cb)
        self.feet = rospy.Subscriber("/spot/status/feet", FootStateArray, self.foot_cb)
        self.estop = rospy.Subscriber(
            "/spot/status/estop", EStopStateArray, self.estop_cb
        )
        self.wifi = rospy.Subscriber("/spot/status/wifi", WiFiState, self.wifi_cb)
        self.battery = rospy.Subscriber(
            "/spot/status/battery_states", BatteryStateArray, self.battery_cb
        )
        self.power = rospy.Subscriber(
            "/spot/status/power_state", PowerState, self.power_cb
        )
        self.system_fault = rospy.Subscriber(
            "/spot/status/system_faults", SystemFaultState, self.system_fault_cb
        )
        self.behaviour_fault = rospy.Subscriber(
            "/spot/status/behavior_faults", BehaviorFaultState, self.behaviour_fault_cb
        )

        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            time.sleep(1)
            counter += 1

        # Check if the data is not empty
        self.assertTrue("joint_state" in self.data, "Joint state is empty")
        self.assertTrue("TF" in self.data, "TF is empty")
        self.assertTrue("twist_odom" in self.data, "Twist odom is empty")
        self.assertTrue("odom" in self.data, "Odom is empty")
        self.assertTrue("foot" in self.data, "Foot is empty")
        self.assertTrue("estop" in self.data, "Estop is empty")
        self.assertTrue("wifi" in self.data, "Wifi is empty")
        self.assertTrue("battery" in self.data, "Battery is empty")
        self.assertTrue("power" in self.data, "Power is empty")
        self.assertTrue("system_fault" in self.data, "System fault is empty")
        self.assertTrue("behaviour_fault" in self.data, "Behaviour fault is empty")

        # Check contents of received data
        self.check_joint_states(self.data["joint_state"])
        self.check_foot_TF_states(self.data["foot_TF"])
        self.check_TF_states(self.data["TF"])
        self.check_twist_odom_states(self.data["twist_odom"])
        self.check_odom_states(self.data["odom"])
        self.check_foot_states(self.data["foot"])
        self.check_estop_states(self.data["estop"])
        self.check_wifi_states(self.data["wifi"])
        self.check_battery_states(self.data["battery"])
        self.check_power_states(self.data["power"])
        self.check_system_fault_states(self.data["system_fault"])
        self.check_behaviour_fault_states(self.data["behaviour_fault"])


class TestMetricsCB(unittest.TestCase):
    def setUp(self):
        self.data = {}

    def metrics_cb(self, data):
        self.data["metrics"] = data

    def check_metrics_data(self, metrics: Metrics):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(metrics.header.stamp.secs, 1)
        self.assertEqual(metrics.header.stamp.nsecs, 2)
        self.assertAlmostEqual(metrics.distance, 3.0, 2)
        self.assertEqual(metrics.gait_cycles, 4)
        self.assertEqual(metrics.time_moving.secs, 5)
        self.assertEqual(metrics.time_moving.nsecs, 6)
        self.assertEqual(metrics.electric_power.secs, 7)
        self.assertEqual(metrics.electric_power.nsecs, 8)

    def test_metrics_cb(self):
        self.metrics = rospy.Subscriber(
            "/spot/status/metrics", Metrics, self.metrics_cb
        )

        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            time.sleep(1)
            counter += 1

        self.assertTrue("metrics" in self.data, "Metrics is empty")
        self.check_metrics_data(self.data["metrics"])


class TestLeaseCB(unittest.TestCase):
    def setUp(self):
        self.data = {}

    def lease_cb(self, data):
        self.data["lease"] = data

    def check_lease_data(self, lease: LeaseArray):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(lease.resources[0].resource, "spot")
        self.assertEqual(lease.resources[0].lease.resource, "lease_id")
        self.assertEqual(lease.resources[0].lease.epoch, "epoch1")
        self.assertEqual(lease.resources[0].lease.sequence[0], 1)
        self.assertEqual(lease.resources[0].lease.sequence[1], 2)
        self.assertEqual(lease.resources[0].lease.sequence[2], 3)
        self.assertEqual(lease.resources[0].lease_owner.client_name, "Adam")
        self.assertEqual(lease.resources[0].lease_owner.user_name, "Dylan")

    def test_lease_cb(self):
        self.lease = rospy.Subscriber("/spot/status/leases", LeaseArray, self.lease_cb)

        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            time.sleep(1)
            counter += 1

        self.assertTrue("lease" in self.data, "Lease is empty")
        self.check_lease_data(self.data["lease"])


class TestHandImageCB(unittest.TestCase):
    def setUp(self):
        self.data = {}

    def hand_image_mono_cb(self, data):
        self.data["hand_image_mono"] = data

    def hand_image_color_cb(self, data):
        self.data["hand_color_image"] = data

    def hand_depth_cb(self, data):
        self.data["hand_depth"] = data

    def hand_depth_in_hand_color_cb(self, data):
        self.data["hand_depth_in_hand_color"] = data

    def hand_image_mono_info_cb(self, data):
        self.data["hand_image_mono_info"] = data

    def hand_image_color_info_cb(self, data):
        self.data["hand_image_color_info"] = data

    def hand_depth_info_cb(self, data):
        self.data["hand_depth_info"] = data

    def hand_depth_in_color_info_cb(self, data):
        self.data["hand_depth_in_color_info"] = data

    def check_hand_image_data_mono(self, hand_image: Image):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image.header.frame_id, "hand_image")
        self.assertEqual(hand_image.height, 640)
        self.assertEqual(hand_image.width, 480)
        self.assertEqual(hand_image.encoding, "mono8")
        self.assertEqual(hand_image.is_bigendian, True)
        self.assertEqual(hand_image.step, 480)
        self.assertEqual(hand_image.data, b"hand_image")

    def check_hand_image_data_depth(self, hand_image: Image):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image.header.frame_id, "hand_depth")
        self.assertEqual(hand_image.height, 424)
        self.assertEqual(hand_image.width, 240)
        self.assertEqual(hand_image.encoding, "16UC1")
        self.assertEqual(hand_image.is_bigendian, False)
        self.assertEqual(hand_image.step, 2 * 240)
        self.assertEqual(hand_image.data, b"hand_depth")

    def check_hand_image_data_color(self, hand_image: Image):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image.header.frame_id, "hand_color_image")
        self.assertEqual(hand_image.height, 640)
        self.assertEqual(hand_image.width, 480)
        self.assertEqual(hand_image.encoding, "rgb8")
        self.assertEqual(hand_image.is_bigendian, True)
        self.assertEqual(hand_image.step, 3 * 480)
        self.assertEqual(hand_image.data, b"hand_color_image")

    def check_hand_image_data_depth_in_color(self, hand_image: Image):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image.header.frame_id, "hand_depth_in_color_frame")
        self.assertEqual(hand_image.height, 640)
        self.assertEqual(hand_image.width, 480)
        self.assertEqual(hand_image.encoding, "mono8")
        self.assertEqual(hand_image.is_bigendian, True)
        self.assertEqual(hand_image.step, 480)
        self.assertEqual(hand_image.data, b"hand_depth_in_color_frame")

    def check_hand_image_info_mono(self, hand_image_info: CameraInfo):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image_info.header.frame_id, "hand_image")
        self.assertEqual(hand_image_info.height, 640)
        self.assertEqual(hand_image_info.width, 480)

    def check_hand_image_info_color(self, hand_image_info: CameraInfo):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image_info.header.frame_id, "hand_color_image")
        self.assertEqual(hand_image_info.height, 640)
        self.assertEqual(hand_image_info.width, 480)

    def check_hand_image_info_depth(self, hand_image_info: CameraInfo):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image_info.header.frame_id, "hand_depth")
        self.assertEqual(hand_image_info.height, 424)
        self.assertEqual(hand_image_info.width, 240)

    def check_hand_image_info_depth_in_color(self, hand_image_info: CameraInfo):
        # Check against the data provided in mock_spot_ros.py
        self.assertEqual(hand_image_info.header.frame_id, "hand_depth_in_color_frame")
        self.assertEqual(hand_image_info.height, 640)
        self.assertEqual(hand_image_info.width, 480)

    def test_hand_image_cb(self):
        self.hand_image_mono = rospy.Subscriber(
            "/spot/camera/hand_mono/image", Image, self.hand_image_mono_cb
        )
        self.hand_image_color = rospy.Subscriber(
            "/spot/camera/hand_color/image", Image, self.hand_image_color_cb
        )
        self.hand_depth = rospy.Subscriber(
            "/spot/depth/hand/image", Image, self.hand_depth_cb
        )
        self.hand_depth_in_hand_color = rospy.Subscriber(
            "/spot/depth/hand/depth_in_color", Image, self.hand_depth_in_hand_color_cb
        )

        self.hand_image_mono_info = rospy.Subscriber(
            "/spot/camera/hand_mono/camera_info",
            CameraInfo,
            self.hand_image_mono_info_cb,
        )
        self.hand_image_color_info = rospy.Subscriber(
            "/spot/camera/hand_color/camera_info",
            CameraInfo,
            self.hand_image_color_info_cb,
        )

        self.hand_depth_info = rospy.Subscriber(
            "/spot/depth/hand/camera_info", CameraInfo, self.hand_depth_info_cb
        )
        self.hand_depth_in_color_info = rospy.Subscriber(
            "/spot/camera/hand/depth_in_color/camera_info",
            CameraInfo,
            self.hand_depth_in_color_info_cb,
        )

        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            time.sleep(1)
            counter += 1

        # Check that we got all the data
        self.assertTrue("hand_image_mono" in self.data, "Hand image is empty")
        self.assertTrue("hand_color_image" in self.data, "Hand image color is empty")
        self.assertTrue("hand_depth" in self.data, "Hand depth is empty")
        self.assertTrue(
            "hand_depth_in_hand_color" in self.data, "Hand depth in hand color is empty"
        )

        self.assertTrue(
            "hand_image_mono_info" in self.data, "Hand image mono info is empty"
        )
        self.assertTrue(
            "hand_image_color_info" in self.data, "Hand image color info is empty"
        )
        self.assertTrue("hand_depth_info" in self.data, "Hand depth info is empty")
        self.assertTrue(
            "hand_depth_in_color_info" in self.data, "Hand depth in color info is empty"
        )

        # Check that the data is valid
        self.check_hand_image_data_mono(self.data["hand_image_mono"])
        self.check_hand_image_data_color(self.data["hand_color_image"])
        self.check_hand_image_data_depth(self.data["hand_depth"])
        self.check_hand_image_data_depth_in_color(self.data["hand_depth_in_hand_color"])

        self.check_hand_image_info_mono(self.data["hand_image_mono_info"])
        self.check_hand_image_info_color(self.data["hand_image_color_info"])
        self.check_hand_image_info_depth(self.data["hand_depth_info"])
        self.check_hand_image_info_depth_in_color(self.data["hand_depth_in_color_info"])


class TestPointCloudCB(unittest.TestCase):
    def setUp(self):
        self.data = {}

    def point_cloud_cb(self, msg: PointCloud2):
        self.data["point_cloud"] = msg

    def check_point_cloud(self, point_cloud_msg: PointCloud2):
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

    def test_point_cloud_cb(self):
        self.point_cloud = rospy.Subscriber(
            "/spot/lidar/points", PointCloud2, self.point_cloud_cb
        )

        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            time.sleep(1)
            counter += 1

        # Check that we got all the data
        self.assertTrue("point_cloud" in self.data, "Point cloud is empty")

        # Check that the data is valid
        self.check_point_cloud(self.data["point_cloud"])


class TestWorldObjectCB(unittest.TestCase):
    def setUp(self):
        self.data = {}

    def world_object_cb(self, msg: WorldObjectArray):
        self.data["world_object"] = msg

    def check_world_object(self, world_object_msg: WorldObjectArray):
        # Check that the world object message is correctly populated
        world_objects: typing.List[WorldObject] = sorted(
            world_object_msg.world_objects, key=lambda x: x.id
        )

        self.assertEqual(world_objects[0].id, 1)
        self.assertEqual(world_objects[0].name, "world_obj_apriltag_350")
        self.assertEqual(
            sorted(world_objects[0].frame_tree_snapshot.child_edges),
            sorted(["vision", "fiducial_350", "filtered_fiducial_350", "body", "odom"]),
        )

    def test_world_object_cb(self):
        self.world_object = rospy.Subscriber(
            "/spot/world_objects", WorldObjectArray, self.world_object_cb
        )

        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            time.sleep(1)
            counter += 1

        # Check that we got all the data
        self.assertTrue("world_object" in self.data, "World object is empty")

        # Check that the data is valid
        self.check_world_object(self.data["world_object"])


class TestServiceHandlers(unittest.TestCase):
    def call_service(self, service_name, *args, **kwargs):
        # Call a service and wait for it to be available
        try:
            rospy.wait_for_service(service_name)
            service_type = get_service_class_by_name(service_name)
            proxy = rospy.ServiceProxy(service_name, service_type)
            return proxy(*args, **kwargs)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def test_claim_service(self):
        # Test that the claim service works
        resp: TriggerResponse = self.call_service("/spot/claim")

        self.assertTrue(resp.success, "Claim service failed")
        self.assertEqual(resp.message, "Successfully called claim")

    def test_release_service(self):
        # Test that the release service works
        resp: TriggerResponse = self.call_service("/spot/release")

        self.assertTrue(resp.success, "Release service failed")
        self.assertEqual(resp.message, "Successfully called release")

    def test_locked_stop(self):
        # Test that the locked stop service works
        resp: TriggerResponse = self.call_service("/spot/locked_stop")

        self.assertTrue(resp.success, "Locked stop service failed")
        self.assertEqual(resp.message, "Successfully called locked_stop")

    def test_stop(self):
        # Test that the stop service works
        resp: TriggerResponse = self.call_service("/spot/stop")

        self.assertTrue(resp.success, "Stop service failed")
        self.assertEqual(resp.message, "Successfully called stop")

    def test_self_right(self):
        # Test that the self right service works
        resp: TriggerResponse = self.call_service("/spot/self_right")

        self.assertTrue(resp.success, "Self right service failed")
        self.assertEqual(resp.message, "Successfully called self_right")

    def test_sit(self):
        # Test that the sit service works
        resp: TriggerResponse = self.call_service("/spot/sit")

        self.assertTrue(resp.success, "Sit service failed")
        self.assertEqual(resp.message, "Successfully called sit")

    def test_stand(self):
        # Test that the stand service works
        resp: TriggerResponse = self.call_service("/spot/stand")

        self.assertTrue(resp.success, "Stand service failed")
        self.assertEqual(resp.message, "Successfully called stand")

    def test_posed_stand(self):
        # Test that the posed stand service works
        resp = self.call_service("/spot/posed_stand", PosedStandRequest())

        self.assertTrue(resp.success, "Posed stand service failed")
        self.assertEqual(resp.message, "Successfully called posed_stand")

    def test_power_on(self):
        # Test that the power on service works
        resp: TriggerResponse = self.call_service("/spot/power_on")

        self.assertTrue(resp.success, "Power on service failed")
        self.assertEqual(resp.message, "Successfully called power_on")

    def test_safe_power_off(self):
        # Test that the safe power off service works
        resp: TriggerResponse = self.call_service("/spot/power_off")

        self.assertTrue(resp.success, "Safe power off service failed")
        self.assertEqual(resp.message, "Successfully called safe_power_off")

    def test_estop_hard(self):
        # Test that the estop hard service works
        resp: TriggerResponse = self.call_service("/spot/estop/hard")

        self.assertTrue(resp.success, "Estop hard service failed")
        self.assertEqual(resp.message, "Successfully called estop_hard")

    def test_estop_gentle(self):
        # Test that the estop soft service works
        resp: TriggerResponse = self.call_service("/spot/estop/gentle")

        self.assertTrue(resp.success, "Estop gentle service failed")
        self.assertEqual(resp.message, "Successfully called estop_gentle")

    def test_estop_release(self):
        # Test that the estop release service works
        resp: TriggerResponse = self.call_service("/spot/estop/release")

        self.assertTrue(resp.success, "Estop release service failed")
        self.assertEqual(resp.message, "Successfully called estop_release")

    def test_clear_behavior_fault(self):
        # Test that the clear behavior fault service works
        resp: TriggerResponse = self.call_service("/spot/clear_behavior_fault")

        self.assertTrue(resp.success, "Clear behavior fault service failed")
        self.assertEqual(resp.message, "Successfully called clear_behavior_fault")

    def test_stair_mode(self):
        # Test that the stair mode service works
        resp: TriggerResponse = self.call_service("/spot/stair_mode", True)

        self.assertTrue(resp.success, "Stair mode service failed")
        self.assertEqual(resp.message, "Successfully called stair_mode")

    def test_locomotion_mode(self):
        # Test that the locomotion mode service works
        resp: TriggerResponse = self.call_service("/spot/locomotion_mode")

        self.assertTrue(resp.success, "Locomotion mode service failed")
        self.assertEqual(resp.message, "Successfully called locomotion_mode")

    def test_swing_height(self):
        # Test that the swing height service works
        resp: TriggerResponse = self.call_service("/spot/swing_height")

        self.assertTrue(resp.success, "Swing height service failed")
        self.assertEqual(resp.message, "Successfully called swing_height")

    def test_velocity_limit(self):
        # Test that the vel limit service works
        resp: TriggerResponse = self.call_service("/spot/velocity_limit")

        self.assertTrue(resp.success, "Velocity limit service failed")
        self.assertEqual(resp.message, "Successfully called velocity_limit")

    def test_allow_motion(self):
        # Test that the allow motion service works
        resp: TriggerResponse = self.call_service("/spot/allow_motion", True)

        self.assertTrue(resp.success, "Allow motion service failed")
        self.assertEqual(resp.message, "Successfully called allow_motion")

    def test_obstacle_params(self):
        # Test that the obstacle params service works
        resp: TriggerResponse = self.call_service("/spot/obstacle_params")

        self.assertTrue(resp.success, "Obstacle params service failed")
        self.assertEqual(resp.message, "Successfully called obstacle_params")

    def test_terrain_params(self):
        # Test that the terrain params service works
        resp: TriggerResponse = self.call_service("/spot/terrain_params")

        self.assertTrue(resp.success, "Terrain params service failed")
        self.assertEqual(resp.message, "Successfully called terrain_params")

    def test_list_graph(self):
        # Test that the list graph service works
        resp: ListGraphResponse = self.call_service("/spot/list_graph")

        self.assertEqual(
            resp.waypoint_ids, ["1", "2", "3"], "List graph service failed"
        )

    def test_roll_over_right(self):
        # Test that the roll over right service works
        resp: TriggerResponse = self.call_service("/spot/roll_over_right")

        self.assertTrue(resp.success, "Roll over right service failed")
        self.assertEqual(resp.message, "Successfully called roll_over_right")

    def test_roll_over_left(self):
        # Test that the roll over left service works
        resp: TriggerResponse = self.call_service("/spot/roll_over_left")

        self.assertTrue(resp.success, "Roll over left service failed")
        self.assertEqual(resp.message, "Successfully called roll_over_left")

    def test_dock(self):
        # Test that the dock service works
        resp: DockResponse = self.call_service("/spot/dock", DockRequest(dock_id=1))

        self.assertTrue(resp.success, "Dock service failed")
        self.assertEqual(resp.message, "Successfully called dock")

    def test_undock(self):
        # Test that the undock service works
        resp: TriggerResponse = self.call_service("/spot/undock")

        self.assertTrue(resp.success, "Undock service failed")
        self.assertEqual(resp.message, "Successfully called undock")

    def test_get_docking_state(self):
        # Test that the get docking state service works
        resp: GetDockStateResponse = self.call_service("/spot/docking_state")

        self.assertTrue(resp.dock_state, "Get docking state service failed")

    def test_arm_stow(self):
        # Test that the arm stow service works
        resp: TriggerResponse = self.call_service("/spot/arm_stow")

        self.assertTrue(resp.success, "Arm stow service failed")
        self.assertEqual(resp.message, "Successfully called arm_stow")

    def test_arm_unstow(self):
        # Test that the arm unstow service works
        resp: TriggerResponse = self.call_service("/spot/arm_unstow")

        self.assertTrue(resp.success, "Arm unstow service failed")
        self.assertEqual(resp.message, "Successfully called arm_unstow")

    def test_gripper_open(self):
        # Test that the gripper open service works
        resp: TriggerResponse = self.call_service("/spot/gripper_open")

        self.assertTrue(resp.success, "Gripper open service failed")
        self.assertEqual(resp.message, "Successfully called gripper_open")

    def test_gripper_close(self):
        # Test that the gripper close service works
        resp: TriggerResponse = self.call_service("/spot/gripper_close")

        self.assertTrue(resp.success, "Gripper close service failed")
        self.assertEqual(resp.message, "Successfully called gripper_close")

    def test_arm_carry(self):
        # Test that the arm carry service works
        resp: TriggerResponse = self.call_service("/spot/arm_carry")

        self.assertTrue(resp.success, "Arm carry service failed")
        self.assertEqual(resp.message, "Successfully called arm_carry")

    def test_gripper_angle_open(self):
        # Test that the gripper angle opens service works
        resp: GripperAngleMoveResponse = self.call_service("/spot/gripper_angle_open")

        self.assertTrue(resp.success, "Gripper angle opens service failed")
        self.assertEqual(resp.message, "Successfully called gripper_angle_open")

    def test_arm_joint_move(self):
        # Test that the arm joint move service works
        resp: ArmJointMovementResponse = self.call_service("/spot/arm_joint_move")

        self.assertTrue(resp.success, "Arm joint move service failed")
        self.assertEqual(resp.message, "Successfully called arm_joint_move")

    def test_force_trajectory(self):
        # Test that the force trajectory service works
        resp: ArmForceTrajectoryResponse = self.call_service("/spot/force_trajectory")

        self.assertTrue(resp.success, "Force trajectory service failed")
        self.assertEqual(resp.message, "Successfully called force_trajectory")

    def test_gripper_pose(self):
        # Test that the hand pose service works
        resp: HandPoseResponse = self.call_service("/spot/gripper_pose")

        self.assertTrue(resp.success, "Hand pose service failed")
        self.assertEqual(resp.message, "Successfully called gripper_pose")

    def test_grasp_3d(self):
        # Test that the grasp 3d service works
        resp: Grasp3dResponse = self.call_service("/spot/grasp_3d")

        self.assertTrue(resp.success, "Grasp 3d service failed")
        self.assertEqual(resp.message, "Successfully called grasp_3d")

    def test_spot_check(self):
        # Test that the Spot Check service works
        resp: SpotCheckResponse = self.call_service("/spot/spot_check")

        self.assertTrue(resp.success, "Spot Check service failed")
        self.assertEqual(resp.message, "Successfully called spot_check")

    def test_download_graph(self):
        resp: DownloadGraphResponse = self.call_service("/spot/download_graph")

        self.assertEqual(resp.waypoint_ids, ["1", "2", "3"])

    def test_graph_close_loops(self):
        resp: GraphCloseLoopsResponse = self.call_service("/spot/graph_close_loops")

        self.assertTrue(resp.success, "Graph close loops service failed")
        self.assertEqual(resp.message, "Successfully called graph_close_loops")

    def test_optimize_graph_anchoring(self):
        resp: TriggerResponse = self.call_service("/spot/optimize_graph_anchoring")

        self.assertTrue(resp.success, "Optimize graph anchoring service failed")
        self.assertEqual(resp.message, "Successfully called graph_optimize_anchoring")

    def test_navigate_init(self):
        resp: NavigateInitResponse = self.call_service("/spot/navigate_init")

        self.assertTrue(resp.success, "Navigate init service failed")
        self.assertEqual(resp.message, "Successfully called navigate_init")


class TestActionHandlers(unittest.TestCase):
    def test_navigate_to_action(self):
        self.navigate_to_action_client = actionlib.SimpleActionClient(
            "/spot/navigate_to", NavigateToAction
        )
        self.navigate_to_action_client.wait_for_server()

        goal = NavigateToGoal()
        goal.navigate_to = "1"

        self.navigate_to_action_client.send_goal(goal)
        self.navigate_to_action_client.wait_for_result()

        result = self.navigate_to_action_client.get_result()

        self.assertEqual(result.message, "Successfully called navigate_to")
        self.assertTrue(result.success, "Navigate to action failed")

    def test_navigate_route_action(self):
        self.navigate_route_action_client = actionlib.SimpleActionClient(
            "/spot/navigate_route", NavigateRouteAction
        )
        self.navigate_route_action_client.wait_for_server()

        goal = NavigateRouteGoal()
        goal.waypoint_ids = ["waypoint1", "waypoint2", "waypoint1"]

        self.navigate_route_action_client.send_goal(goal)
        self.navigate_route_action_client.wait_for_result()

        result = self.navigate_route_action_client.get_result()

        self.assertEqual(result.message, "Successfully called navigate_route")
        self.assertTrue(result.success, "Navigate_route action failed")

    def test_trajectory_action(self):
        self.trajectory_action_client = actionlib.SimpleActionClient(
            "/spot/trajectory", TrajectoryAction
        )
        self.trajectory_action_client.wait_for_server()

        goal = TrajectoryGoal()
        goal.target_pose = PoseStamped(pose=Pose(position=Point(x=1, y=1, z=1)))
        goal.duration = Duration(rospy.Duration(10))

        self.trajectory_action_client.send_goal(goal)
        self.trajectory_action_client.wait_for_result()

        result = self.trajectory_action_client.get_result()

        self.assertEqual(result.message, "Successfully called trajectory")
        self.assertTrue(result.success, "Trajectory action failed")

    def test_motion_or_idle_body_pose_action(self):
        self.motion_or_idle_body_pose_action_client = actionlib.SimpleActionClient(
            "/spot/motion_or_idle_body_pose", PoseBodyAction
        )
        self.motion_or_idle_body_pose_action_client.wait_for_server()

        goal = PoseBodyGoal()
        goal.body_pose = Pose(position=Point(x=1, y=1, z=1))

        self.motion_or_idle_body_pose_action_client.send_goal(goal)
        self.motion_or_idle_body_pose_action_client.wait_for_result()

        result = self.motion_or_idle_body_pose_action_client.get_result()

        self.assertEqual(result.message, "Successfully called motion_or_idle_body_pose")
        self.assertTrue(result.success, "Motion or idle body pose action failed")

    def test_body_pose(self):
        self.body_pose_action_client = actionlib.SimpleActionClient(
            "/spot/body_pose", PoseBodyAction
        )
        self.body_pose_action_client.wait_for_server()

        goal = PoseBodyGoal()
        goal.body_pose = Pose(position=Point(x=1, y=1, z=1))

        self.body_pose_action_client.send_goal(goal)
        self.body_pose_action_client.wait_for_result()

        result = self.body_pose_action_client.get_result()

        self.assertEqual(result.message, "Successfully called body_pose")
        self.assertTrue(result.success, "Body pose action failed")

    def test_dock_action(self):
        self.dock_action_client = actionlib.SimpleActionClient("/spot/dock", DockAction)
        self.dock_action_client.wait_for_server()

        goal = DockGoal()
        goal.undock = False
        goal.dock_id = 101

        self.dock_action_client.send_goal(goal)
        self.dock_action_client.wait_for_result()

        result = self.dock_action_client.get_result()

        self.assertEqual(result.message, "Successfully called dock")
        self.assertTrue(result.success, "Dock action failed")


# Test suite for SpotROS
class TestSuiteSpotROS(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteSpotROS, self).__init__()
        self.loader = unittest.TestLoader()
        self.addTest(self.loader.loadTestsFromTestCase(TestRobotStateCB))
        self.addTest(self.loader.loadTestsFromTestCase(TestMetricsCB))
        self.addTest(self.loader.loadTestsFromTestCase(TestLeaseCB))
        self.addTest(self.loader.loadTestsFromTestCase(TestHandImageCB))
        self.addTest(self.loader.loadTestsFromTestCase(TestPointCloudCB))
        self.addTest(self.loader.loadTestsFromTestCase(TestWorldObjectCB))
        self.addTest(self.loader.loadTestsFromTestCase(TestServiceHandlers))
        self.addTest(self.loader.loadTestsFromTestCase(TestActionHandlers))


if __name__ == "__main__":
    print("Starting tests!")
    import rosunit

    rospy.init_node(NAME, anonymous=True)
    rosunit.unitrun(PKG, NAME, TestRobotStateCB)
    rosunit.unitrun(PKG, NAME, TestMetricsCB)
    rosunit.unitrun(PKG, NAME, TestLeaseCB)
    rosunit.unitrun(PKG, NAME, TestHandImageCB)
    rosunit.unitrun(PKG, NAME, TestPointCloudCB)
    rosunit.unitrun(PKG, NAME, TestWorldObjectCB)
    rosunit.unitrun(PKG, NAME, TestServiceHandlers)
    rosunit.unitrun(PKG, NAME, TestActionHandlers)
    print("Tests complete!")
