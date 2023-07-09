import copy

import rospy
import math
import time
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry


from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import Quaternion, SE2VelocityLimit
from bosdyn.client import math_helpers
from google.protobuf.wrappers_pb2 import DoubleValue
import actionlib
import functools
import math
import bosdyn.geometry
import tf2_ros
import tf2_geometry_msgs

from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray, LeaseResource
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray
from spot_msgs.msg import DockAction, DockGoal, DockResult
from spot_msgs.msg import PoseBodyAction, PoseBodyGoal, PoseBodyResult
from spot_msgs.msg import Feedback
from spot_msgs.msg import MobilityParams, ObstacleParams, TerrainParams
from spot_msgs.msg import NavigateToAction, NavigateToResult, NavigateToFeedback
from spot_msgs.msg import TrajectoryAction, TrajectoryResult, TrajectoryFeedback
from spot_msgs.srv import ListGraph, ListGraphResponse
from spot_msgs.srv import SetLocomotion, SetLocomotionResponse
from spot_msgs.srv import SetTerrainParams, SetTerrainParamsResponse
from spot_msgs.srv import SetObstacleParams, SetObstacleParamsResponse
from spot_msgs.srv import ClearBehaviorFault, ClearBehaviorFaultResponse
from spot_msgs.srv import SetVelocity, SetVelocityResponse
from spot_msgs.srv import Dock, DockResponse, GetDockState, GetDockStateResponse
from spot_msgs.srv import PosedStand, PosedStandResponse
from spot_msgs.srv import SetSwingHeight, SetSwingHeightResponse
from spot_msgs.srv import (
    ArmJointMovement,
    ArmJointMovementResponse,
    ArmJointMovementRequest,
)
from spot_msgs.srv import (
    GripperAngleMove,
    GripperAngleMoveResponse,
    GripperAngleMoveRequest,
)
from spot_msgs.srv import (
    ArmForceTrajectory,
    ArmForceTrajectoryResponse,
    ArmForceTrajectoryRequest,
)
from spot_msgs.srv import HandPose, HandPoseResponse, HandPoseRequest
from spot_msgs.srv import Grasp3d, Grasp3dRequest, Grasp3dResponse

from .ros_helpers import *
from spot_wrapper.wrapper import SpotWrapper

import actionlib
import logging
import threading


class RateLimitedCall:
    """
    Wrap a function with this class to limit how frequently it can be called within a loop
    """

    def __init__(self, fn, rate_limit):
        """

        Args:
            fn: Function to call
            rate_limit: The function will not be called faster than this rate
        """
        self.fn = fn
        self.min_time_between_calls = 1.0 / rate_limit
        self.last_call = 0

    def __call__(self):
        now_sec = time.time()
        if (now_sec - self.last_call) > self.min_time_between_calls:
            self.fn()
            self.last_call = now_sec


class SpotROS:
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self):
        self.spot_wrapper = None
        self.last_tf_msg = TFMessage()

        self.callbacks = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["robot_state"] = self.RobotStateCB
        self.callbacks["metrics"] = self.MetricsCB
        self.callbacks["lease"] = self.LeaseCB
        self.callbacks["front_image"] = self.FrontImageCB
        self.callbacks["side_image"] = self.SideImageCB
        self.callbacks["rear_image"] = self.RearImageCB
        self.callbacks["hand_image"] = self.HandImageCB
        self.callbacks["lidar_points"] = self.PointCloudCB
        self.active_camera_tasks = []
        self.camera_pub_to_async_task_mapping = {}

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
            tf_msg = GetTFFromState(state, self.spot_wrapper, self.mode_parent_odom_tf, self.publish_odom_tf)
            to_remove = []
            if len(tf_msg.transforms) > 0:
                for transform in tf_msg.transforms:
                    for last_tf in self.last_tf_msg.transforms:
                        if transform == last_tf:
                            to_remove.append(transform)

                if to_remove:
                    # Do it this way to preserve the original tf message received. If we store the message we have
                    # destroyed then if there are two duplicates in a row we will not remove the second set.
                    deduplicated_tf = copy.deepcopy(tf_msg)
                    for repeat_tf in to_remove:
                        deduplicated_tf.transforms.remove(repeat_tf)
                    publish_tf = deduplicated_tf
                else:
                    publish_tf = tf_msg

                self.tf_pub.publish(publish_tf)
            self.last_tf_msg = tf_msg

            # Odom Twist #
            twist_odom_msg = GetOdomTwistFromState(state, self.spot_wrapper)
            self.odom_twist_pub.publish(twist_odom_msg)

            # Odom #
            use_vision = self.mode_parent_odom_tf == "vision"
            odom_msg = GetOdomFromState(
                state,
                self.spot_wrapper,
                use_vision=use_vision,
            )
            self.odom_pub.publish(odom_msg)

            odom_corrected_msg = get_corrected_odom(odom_msg)
            self.odom_corrected_pub.publish(odom_corrected_msg)

            # Feet #
            foot_array_msg = GetFeetFromState(state, self.spot_wrapper)
            self.tf_pub.publish(generate_feet_tf(foot_array_msg))
            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = GetEStopStateFromState(state, self.spot_wrapper)
            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = GetWifiFromState(state, self.spot_wrapper)
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = GetBatteryStatesFromState(
                state, self.spot_wrapper
            )
            self.is_charging = (
                battery_states_array_msg.battery_states[0].status
                == BatteryState.STATUS_CHARGING
            )
            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = GetPowerStatesFromState(state, self.spot_wrapper)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = GetSystemFaultsFromState(state, self.spot_wrapper)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = getBehaviorFaultsFromState(
                state, self.spot_wrapper
            )
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
                    metrics_msg.time_moving = rospy.Time(
                        metric.duration.seconds, metric.duration.nanos
                    )
                if metric.label == "electric power":
                    metrics_msg.electric_power = rospy.Time(
                        metric.duration.seconds, metric.duration.nanos
                    )

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
            image_msg0, camera_info_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.frontleft_image_pub.publish(image_msg0)
            self.frontleft_image_info_pub.publish(camera_info_msg0)
            image_msg1, camera_info_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.frontright_image_pub.publish(image_msg1)
            self.frontright_image_info_pub.publish(camera_info_msg1)
            image_msg2, camera_info_msg2 = getImageMsg(data[2], self.spot_wrapper)
            self.frontleft_depth_pub.publish(image_msg2)
            self.frontleft_depth_info_pub.publish(camera_info_msg2)
            image_msg3, camera_info_msg3 = getImageMsg(data[3], self.spot_wrapper)
            self.frontright_depth_pub.publish(image_msg3)
            self.frontright_depth_info_pub.publish(camera_info_msg3)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.side_images
        if data:
            image_msg0, camera_info_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.left_image_pub.publish(image_msg0)
            self.left_image_info_pub.publish(camera_info_msg0)
            image_msg1, camera_info_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.right_image_pub.publish(image_msg1)
            self.right_image_info_pub.publish(camera_info_msg1)
            image_msg2, camera_info_msg2 = getImageMsg(data[2], self.spot_wrapper)
            self.left_depth_pub.publish(image_msg2)
            self.left_depth_info_pub.publish(camera_info_msg2)
            image_msg3, camera_info_msg3 = getImageMsg(data[3], self.spot_wrapper)
            self.right_depth_pub.publish(image_msg3)
            self.right_depth_info_pub.publish(camera_info_msg3)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.rear_images
        if data:
            mage_msg0, camera_info_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.back_image_pub.publish(mage_msg0)
            self.back_image_info_pub.publish(camera_info_msg0)
            mage_msg1, camera_info_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.back_depth_pub.publish(mage_msg1)
            self.back_depth_info_pub.publish(camera_info_msg1)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])

    def HandImageCB(self, results):
        """Callback for when the Spot Wrapper gets new hand image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.hand_images
        if data:
            mage_msg0, camera_info_msg0 = getImageMsg(data[0], self.spot_wrapper)
            self.hand_image_mono_pub.publish(mage_msg0)
            self.hand_image_mono_info_pub.publish(camera_info_msg0)
            mage_msg1, camera_info_msg1 = getImageMsg(data[1], self.spot_wrapper)
            self.hand_depth_pub.publish(mage_msg1)
            self.hand_depth_info_pub.publish(camera_info_msg1)
            image_msg2, camera_info_msg2 = getImageMsg(data[2], self.spot_wrapper)
            self.hand_image_color_pub.publish(image_msg2)
            self.hand_image_color_info_pub.publish(camera_info_msg2)
            image_msg3, camera_info_msg3 = getImageMsg(data[3], self.spot_wrapper)
            self.hand_depth_in_hand_color_pub.publish(image_msg3)
            self.hand_depth_in_color_info_pub.publish(camera_info_msg3)

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

    def PointCloudCB(self, results):
        """Callback for when the Spot Wrapper gets new point cloud data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        data = self.spot_wrapper.point_clouds
        if data:
            point_cloud_msg = GetPointCloudMsg(data[0], self.spot_wrapper)
            self.point_cloud_pub.publish(point_cloud_msg)

            self.populate_lidar_static_transforms(data[0])

    def handle_claim(self, req):
        """ROS service handler for the claim service"""
        resp = self.spot_wrapper.claim()
        return TriggerResponse(resp[0], resp[1])

    def handle_release(self, req):
        """ROS service handler for the release service"""
        resp = self.spot_wrapper.release()
        return TriggerResponse(resp[0], resp[1])

    def handle_locked_stop(self, req):
        """Stop the current motion of the robot and disallow any further motion until the allow motion service is
        called"""
        self.allow_motion = False
        return self.handle_stop(req)

    def handle_stop(self, req):
        """ROS service handler for the stop service. Interrupts the currently active motion"""
        resp = self.spot_wrapper.stop()
        message = "Spot stop service was called"
        if self.navigate_as.is_active():
            self.navigate_as.set_preempted(
                NavigateToResult(success=False, message=message)
            )
        if self.trajectory_server.is_active():
            self.trajectory_server.set_preempted(
                TrajectoryResult(success=False, message=message)
            )
        if self.body_pose_as.is_active():
            self.body_pose_as.set_preempted(
                PoseBodyResult(success=False, message=message)
            )
        return TriggerResponse(resp[0], resp[1])

    def handle_self_right(self, req):
        """ROS service handler for the self-right service"""
        if not self.robot_allowed_to_move(autonomous_command=False):
            return TriggerResponse(False, "Robot motion is not allowed")

        resp = self.spot_wrapper.self_right()
        return TriggerResponse(resp[0], resp[1])

    def handle_sit(self, req):
        """ROS service handler for the sit service"""
        if not self.robot_allowed_to_move(autonomous_command=False):
            return TriggerResponse(False, "Robot motion is not allowed")

        resp = self.spot_wrapper.sit()
        return TriggerResponse(resp[0], resp[1])

    def handle_stand(self, req):
        """ROS service handler for the stand service"""
        if not self.robot_allowed_to_move(autonomous_command=False):
            return TriggerResponse(False, "Robot motion is not allowed")
        resp = self.spot_wrapper.stand()
        return TriggerResponse(resp[0], resp[1])

    def handle_posed_stand(self, req):
        """
        Handle a service call for the posed stand

        Args:
            req: PosedStandRequest

        Returns: PosedStandResponse
        """
        success, message = self._posed_stand(
            req.body_height, req.body_yaw, req.body_pitch, req.body_roll
        )
        return PosedStandResponse(success, message)

    def handle_posed_stand_action(self, action):
        """
        Handle a call to the posed stand actionserver

        If no value is provided, this is equivalent to the basic stand commmand

        Args:
            action: PoseBodyGoal

        """
        success, message = self._posed_stand(
            action.body_height, action.yaw, action.pitch, action.roll
        )
        result = PoseBodyResult(success, message)
        rospy.sleep(2)  # Only return after the body has had a chance to move
        if success:
            self.body_pose_as.set_succeeded(result)
        else:
            self.body_pose_as.set_aborted(result)

    def _posed_stand(self, body_height, yaw, pitch, roll):
        """
        Make the robot do a posed stand with specified body height and orientation

        By empirical observation, the limit on body height is [-0.16, 0.11], and RPY are probably limited to 30
        degrees. Roll values are likely affected by the payload configuration as well. If the payload is
        misconfigured a high roll value could cause it to hit the legs

        Args:
            body_height: Height of the body relative to the default height
            yaw: Yaw to apply (in degrees)
            pitch: Pitch to apply (in degrees)
            roll: Roll to apply (in degrees)

        Returns:

        """
        if not self.robot_allowed_to_move(autonomous_command=False):
            return False, "Robot motion is not allowed"

        resp = self.spot_wrapper.stand(
            body_height=body_height,
            body_yaw=math.radians(yaw),
            body_pitch=math.radians(pitch),
            body_roll=math.radians(roll),
        )

        return resp[0], resp[1]

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
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors"""
        resp = self.spot_wrapper.assertEStop(False)
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_disengage(self, req):
        """ROS service handler to disengage the eStop on the robot."""
        resp = self.spot_wrapper.disengageEStop()
        return TriggerResponse(resp[0], resp[1])

    def handle_clear_behavior_fault(self, req):
        """ROS service handler for clearing behavior faults"""
        resp = self.spot_wrapper.clear_behavior_fault(req.id)
        return ClearBehaviorFaultResponse(resp[0], resp[1])

    def handle_stair_mode(self, req):
        """ROS service handler to set a stair mode to the robot."""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = req.data
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetBoolResponse(True, "Success")
        except Exception as e:
            return SetBoolResponse(False, "Error:{}".format(e))

    def handle_locomotion_mode(self, req):
        """ROS service handler to set locomotion mode"""
        if req.locomotion_mode in [0, 9] or req.locomotion_mode > 10:
            msg = "Attempted to set locomotion mode to {}, which is an invalid value.".format(
                req.locomotion_mode
            )
            rospy.logerr(msg)
            return SetLocomotionResponse(False, msg)
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = req.locomotion_mode
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetLocomotionResponse(True, "Success")
        except Exception as e:
            return SetLocomotionResponse(False, "Error:{}".format(e))

    def handle_swing_height(self, req):
        """ROS service handler to set the step swing height"""
        if req.swing_height == 0 or req.swing_height > 3:
            msg = "Attempted to set step swing height to {}, which is an invalid value.".format(
                req.swing_height
            )
            rospy.logerr(msg)
            return SetSwingHeightResponse(False, msg)
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.swing_height = req.swing_height
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetSwingHeightResponse(True, "Success")
        except Exception as e:
            return SetSwingHeightResponse(False, "Error:{}".format(e))

    def handle_vel_limit(self, req):
        """
        Handle a velocity_limit service call.

        Args:
            req: SetVelocityRequest containing requested velocity limit

        Returns: SetVelocityResponse
        """
        success, message = self.set_velocity_limits(
            req.velocity_limit.linear.x,
            req.velocity_limit.linear.y,
            req.velocity_limit.angular.z,
        )
        return SetVelocityResponse(success, message)

    def set_velocity_limits(self, max_linear_x, max_linear_y, max_angular_z):
        """
        Modify the mobility params to have a limit on the robot's velocity during trajectory commands.
        Velocities sent to cmd_vel ignore these values

        Passing 0 to any of the values will use spot's internal limits

        Args:
            max_linear_x: Maximum forwards/backwards velocity
            max_linear_y: Maximum lateral velocity
            max_angular_z: Maximum rotation velocity

        Returns: (bool, str) boolean indicating whether the call was successful, along with a message

        """
        if any(
            map(lambda x: 0 < x < 0.15, [max_linear_x, max_linear_y, max_angular_z])
        ):
            return (
                False,
                "Error: One of the values provided to velocity limits was below 0.15. Values in the range ("
                "0,0.15) can cause unexpected behaviour of the trajectory command.",
            )
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(
                SE2VelocityLimit(
                    max_vel=math_helpers.SE2Velocity(
                        max_linear_x, max_linear_y, max_angular_z
                    ).to_proto(),
                    min_vel=math_helpers.SE2Velocity(
                        -max_linear_x, -max_linear_y, -max_angular_z
                    ).to_proto(),
                )
            )
            self.spot_wrapper.set_mobility_params(mobility_params)
            return True, "Success"
        except Exception as e:
            return False, "Error:{}".format(e)

    def _transform_pose_to_body_frame(self, pose):
        """
        Transform a pose to the body frame

        Args:
            pose: PoseStamped to transform

        Raises: tf2_ros.LookupException if the transform lookup fails

        Returns: Transformed pose in body frame if given pose is not in the body frame, or the original pose if it is
        in the body frame
        """
        if pose.header.frame_id == "body":
            return pose

        body_to_fixed = self.tf_buffer.lookup_transform(
            "body", pose.header.frame_id, rospy.Time()
        )

        pose_in_body = tf2_geometry_msgs.do_transform_pose(pose, body_to_fixed)
        pose_in_body.header.frame_id = "body"

        return pose_in_body

    def robot_allowed_to_move(self, autonomous_command=True):
        """
        Check if the robot is allowed to move. This means checking both that autonomy is enabled, which can only be
        set when the driver is started, and also that motion is allowed, the state of which can change while the
        driver is running

        Args:
            autonomous_command: If true, indicates that this function should also check if autonomy is enabled

        Returns: True if the robot is allowed to move, false otherwise

        """
        if not self.allow_motion:
            rospy.logwarn(
                "Spot is not currently allowed to move. Use the allow_motion service to allow the robot to "
                "move."
            )
        autonomy_ok = True
        if autonomous_command:
            if not self.autonomy_enabled:
                rospy.logwarn(
                    "Spot is not allowed to be autonomous because this instance of the driver was started "
                    "with it disabled. Set autonomy_enabled to true in the launch file to enable it."
                )
                autonomy_ok = False

            if self.is_charging:
                rospy.logwarn(
                    "Spot cannot be autonomous because it is connected to shore power."
                )
                autonomy_ok = False

        return self.allow_motion and autonomy_ok

    def handle_allow_motion(self, req):
        """
        Handle a call to set whether motion is allowed or not

        When motion is not allowed, any service call or topic which can move the robot will return without doing
        anything

        Returns: (bool, str) True if successful, along with a message

        """
        self.allow_motion = req.data
        rospy.loginfo(
            "Robot motion is now {}".format(
                "allowed" if self.allow_motion else "disallowed"
            )
        )
        if not self.allow_motion:
            # Always send a stop command if disallowing motion, in case the robot is moving when it is sent
            self.spot_wrapper.stop()
        return True, "Spot motion was {}".format("enabled" if req.data else "disabled")

    def handle_obstacle_params(self, req):
        """
        Handle a call to set the obstacle params part of mobility params. The previous values are always overwritten.

        Args:
            req:

        Returns: (bool, str) True if successful, along with a message

        """
        mobility_params = self.spot_wrapper.get_mobility_params()
        obstacle_params = spot_command_pb2.ObstacleParams()
        # Currently only the obstacle setting that we allow is the padding. The previous value is always overwritten
        # Clamp to the range [0, 0.5] as on the controller
        if req.obstacle_params.obstacle_avoidance_padding < 0:
            rospy.logwarn(
                "Received padding value of {}, clamping to 0".format(
                    req.obstacle_params.obstacle_avoidance_padding
                )
            )
            req.obstacle_params.obstacle_avoidance_padding = 0
        if req.obstacle_params.obstacle_avoidance_padding > 0.5:
            rospy.logwarn(
                "Received padding value of {}, clamping to 0.5".format(
                    req.obstacle_params.obstacle_avoidance_padding
                )
            )
            req.obstacle_params.obstacle_avoidance_padding = 0.5

        disable_notallowed = ""
        if any(
            [
                req.obstacle_params.disable_vision_foot_obstacle_avoidance,
                req.obstacle_params.disable_vision_foot_constraint_avoidance,
                req.obstacle_params.disable_vision_body_obstacle_avoidance,
                req.obstacle_params.disable_vision_foot_obstacle_body_assist,
                req.obstacle_params.disable_vision_negative_obstacles,
            ]
        ):
            disable_notallowed = " Disabling any of the obstacle avoidance components is not currently allowed."
            rospy.logerr(
                "At least one of the disable settings on obstacle params was true."
                + disable_notallowed
            )

        obstacle_params.obstacle_avoidance_padding = (
            req.obstacle_params.obstacle_avoidance_padding
        )

        mobility_params.obstacle_params.CopyFrom(obstacle_params)
        self.spot_wrapper.set_mobility_params(mobility_params)
        return True, "Successfully set obstacle params" + disable_notallowed

    def handle_terrain_params(self, req):
        """
        Handle a call to set the terrain params part of mobility params. The previous values are always overwritten

        Args:
            req:

        Returns: (bool, str) True if successful, along with a message

        """
        mobility_params = self.spot_wrapper.get_mobility_params()

        # We always overwrite the previous settings of these values. Reject if not within recommended limits (as on
        # the controller)
        if 0.2 <= req.terrain_params.ground_mu_hint <= 0.8:
            # For some reason assignment to ground_mu_hint is not allowed once the terrain params are initialised
            # Must initialise with the protobuf type DoubleValue for initialisation to work
            terrain_params = spot_command_pb2.TerrainParams(
                ground_mu_hint=DoubleValue(value=req.terrain_params.ground_mu_hint)
            )
        else:
            return (
                False,
                "Failed to set terrain params, ground_mu_hint of {} is not in the range [0.4, 0.8]".format(
                    req.terrain_params.ground_mu_hint
                ),
            )

        if req.terrain_params.grated_surfaces_mode in [1, 2, 3]:
            terrain_params.grated_surfaces_mode = (
                req.terrain_params.grated_surfaces_mode
            )
        else:
            return (
                False,
                "Failed to set terrain params, grated_surfaces_mode {} was not one of [1, 2, 3]".format(
                    req.terrain_params.grated_surfaces_mode
                ),
            )

        mobility_params.terrain_params.CopyFrom(terrain_params)
        self.spot_wrapper.set_mobility_params(mobility_params)
        return True, "Successfully set terrain params"

    def trajectory_callback(self, msg):
        """
        Handle a callback from the trajectory topic requesting to go to a location

        The trajectory will time out after 5 seconds

        Args:
            msg: PoseStamped containing desired pose

        Returns:
        """
        if not self.robot_allowed_to_move():
            rospy.logerr(
                "Trajectory topic received a message but the robot is not allowed to move."
            )
            return

        try:
            self._send_trajectory_command(
                self._transform_pose_to_body_frame(msg), rospy.Duration(5)
            )
        except tf2_ros.LookupException as e:
            rospy.logerr(str(e))

    def handle_trajectory(self, req):
        """ROS actionserver execution handler to handle receiving a request to move to a location"""
        if not self.robot_allowed_to_move():
            rospy.logerr(
                "Trajectory service was called but robot is not allowed to move"
            )
            self.trajectory_server.set_aborted(
                TrajectoryResult(False, "Robot is not allowed to move.")
            )
            return

        mobility_params = self.spot_wrapper.get_mobility_params()
        if (
            mobility_params.vel_limit.max_vel.linear.x == 0
            or mobility_params.vel_limit.max_vel.linear.y == 0
            or mobility_params.vel_limit.max_vel.angular == 0
        ):
            rospy.logerr(
                "Spot will not move as one or more of its velocity limits are set to 0. "
            )
            self.trajectory_server.set_aborted(
                TrajectoryResult(False, "Velocity limits are set to 0.")
            )
            return

        target_pose = req.target_pose
        if req.target_pose.header.frame_id != "body":
            rospy.logwarn("Pose given was not in the body frame, will transform")
            try:
                target_pose = self._transform_pose_to_body_frame(target_pose)
            except tf2_ros.LookupException as e:
                self.trajectory_server.set_aborted(
                    TrajectoryResult(False, "Could not transform pose into body frame")
                )
                return
        if req.duration.data.to_sec() <= 0:
            self.trajectory_server.set_aborted(
                TrajectoryResult(False, "duration must be larger than 0")
            )
            return

        cmd_duration = rospy.Duration(req.duration.data.secs, req.duration.data.nsecs)
        resp = self._send_trajectory_command(
            target_pose, cmd_duration, req.precise_positioning
        )

        def timeout_cb(trajectory_server, _):
            trajectory_server.publish_feedback(
                TrajectoryFeedback("Failed to reach goal, timed out")
            )
            trajectory_server.set_aborted(
                TrajectoryResult(False, "Failed to reach goal, timed out")
            )

        # Abort the actionserver if cmd_duration is exceeded - the driver stops but does not provide feedback to
        # indicate this so we monitor it ourselves
        cmd_timeout = rospy.Timer(
            cmd_duration,
            functools.partial(timeout_cb, self.trajectory_server),
            oneshot=True,
        )

        # Sleep to allow some feedback to come through from the trajectory command
        rospy.sleep(0.25)
        if self.spot_wrapper._trajectory_status_unknown:
            rospy.logerr(
                "Sent trajectory request to spot but received unknown feedback. Resending command. This will "
                "only be attempted once"
            )
            # If we receive an unknown result from the trajectory, something went wrong internally (not
            # catastrophically). We need to resend the command, because getting status unknown happens right when
            # the command is sent. It's unclear right now why this happens
            resp = self._send_trajectory_command(
                target_pose, cmd_duration, req.precise_positioning
            )
            cmd_timeout.shutdown()
            cmd_timeout = rospy.Timer(
                cmd_duration,
                functools.partial(timeout_cb, self.trajectory_server),
                oneshot=True,
            )

        # The trajectory command is non-blocking but we need to keep this function up in order to interrupt if a
        # preempt is requested and to return success if/when the robot reaches the goal. Also check the is_active to
        # monitor whether the timeout_cb has already aborted the command
        rate = rospy.Rate(10)
        while (
            not rospy.is_shutdown()
            and not self.trajectory_server.is_preempt_requested()
            and not self.spot_wrapper.at_goal
            and self.trajectory_server.is_active()
            and not self.spot_wrapper._trajectory_status_unknown
        ):
            if self.spot_wrapper.near_goal:
                if self.spot_wrapper._last_trajectory_command_precise:
                    self.trajectory_server.publish_feedback(
                        TrajectoryFeedback("Near goal, performing final adjustments")
                    )
                else:
                    self.trajectory_server.publish_feedback(
                        TrajectoryFeedback("Near goal")
                    )
            else:
                self.trajectory_server.publish_feedback(
                    TrajectoryFeedback("Moving to goal")
                )
            rate.sleep()

        # If still active after exiting the loop, the command did not time out
        if self.trajectory_server.is_active():
            cmd_timeout.shutdown()
            if self.trajectory_server.is_preempt_requested():
                self.trajectory_server.publish_feedback(TrajectoryFeedback("Preempted"))
                self.trajectory_server.set_preempted()
                self.spot_wrapper.stop()
                return

            if self.spot_wrapper.at_goal:
                self.trajectory_server.publish_feedback(
                    TrajectoryFeedback("Reached goal")
                )
                self.trajectory_server.set_succeeded(TrajectoryResult(resp[0], resp[1]))
            else:
                self.trajectory_server.publish_feedback(
                    TrajectoryFeedback("Failed to reach goal")
                )
                self.trajectory_server.set_aborted(
                    TrajectoryResult(False, "Failed to reach goal")
                )

    def handle_roll_over_right(self, req):
        """Robot sit down and roll on to it its side for easier battery access"""
        del req
        resp = self.spot_wrapper.battery_change_pose(1)
        return TriggerResponse(resp[0], resp[1])

    def handle_roll_over_left(self, req):
        """Robot sit down and roll on to it its side for easier battery access"""
        del req
        resp = self.spot_wrapper.battery_change_pose(2)
        return TriggerResponse(resp[0], resp[1])

    def handle_dock(self, req):
        """Dock the robot"""
        resp = self.spot_wrapper.dock(req.dock_id)
        return DockResponse(resp[0], resp[1])

    def handle_undock(self, req):
        """Undock the robot"""
        resp = self.spot_wrapper.undock()
        return TriggerResponse(resp[0], resp[1])

    def handle_dock_action(self, req: DockGoal):
        if req.undock:
            resp = self.spot_wrapper.undock()
        else:
            resp = self.spot_wrapper.dock(req.dock_id)

        if resp[0]:
            self.dock_as.set_succeeded(DockResult(resp[0], resp[1]))
        else:
            self.dock_as.set_aborted(DockResult(resp[0], resp[1]))

    def handle_get_docking_state(self, req):
        """Get docking state of robot"""
        resp = self.spot_wrapper.get_docking_state()
        return GetDockStateResponse(GetDockStatesFromState(resp))

    def _send_trajectory_command(self, pose, duration, precise=True):
        """
        Send a trajectory command to the robot

        Args:
            pose: Pose the robot should go to. Must be in the body frame
            duration: After this duration, the command will time out and the robot will stop
            precise: If true, the robot will position itself precisely at the target pose, otherwise it will end up
                     near (within ~0.5m, rotation optional) the requested location

        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message

        """
        if not self.robot_allowed_to_move():
            rospy.logerr("send trajectory was called but motion is not allowed.")
            return

        if pose.header.frame_id != "body":
            rospy.logerr("Trajectory command poses must be in the body frame")
            return

        return self.spot_wrapper.trajectory_cmd(
            goal_x=pose.pose.position.x,
            goal_y=pose.pose.position.y,
            goal_heading=math_helpers.Quat(
                w=pose.pose.orientation.w,
                x=pose.pose.orientation.x,
                y=pose.pose.orientation.y,
                z=pose.pose.orientation.z,
            ).to_yaw(),
            cmd_duration=duration.to_sec(),
            precise_position=precise,
        )

    def cmdVelCallback(self, data):
        """Callback for cmd_vel command"""
        if not self.robot_allowed_to_move():
            rospy.logerr("cmd_vel received a message but motion is not allowed.")
            return

        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z)

    def in_motion_or_idle_pose_cb(self, data):
        """
        Callback for pose to be used while in motion or idling

        This sets the body control field in the mobility params. This means that the pose will be used while a motion
        command is executed. Only the pitch is maintained while moving. The roll and yaw will be applied by the idle
        stand command.
        """
        if not self.robot_allowed_to_move(autonomous_command=False):
            rospy.logerr("body pose received a message but motion is not allowed.")
            return

        self._set_in_motion_or_idle_body_pose(data)

    def handle_in_motion_or_idle_body_pose(self, goal):
        """
        Handle a goal received from the pose body actionserver

        Args:
            goal: PoseBodyGoal containing a pose to apply to the body

        Returns:

        """
        # We can change the body pose if autonomy is not allowed
        if not self.robot_allowed_to_move(autonomous_command=False):
            rospy.logerr("body pose actionserver was called but motion is not allowed.")
            return

        # If the body_pose is empty, we use the rpy + height components instead
        if goal.body_pose == Pose():
            # If the rpy+body height are all zero then we set the body to neutral pose
            if not any(
                [
                    goal.roll,
                    goal.pitch,
                    goal.yaw,
                    not math.isclose(goal.body_height, 0, abs_tol=1e-9),
                ]
            ):
                pose = Pose()
                pose.orientation.w = 1
                self._set_in_motion_or_idle_body_pose(pose)
            else:
                pose = Pose()
                # Multiplication order is important to get the correct quaternion
                orientation_quat = (
                    math_helpers.Quat.from_yaw(math.radians(goal.yaw))
                    * math_helpers.Quat.from_pitch(math.radians(goal.pitch))
                    * math_helpers.Quat.from_roll(math.radians(goal.roll))
                )
                pose.orientation.x = orientation_quat.x
                pose.orientation.y = orientation_quat.y
                pose.orientation.z = orientation_quat.z
                pose.orientation.w = orientation_quat.w
                pose.position.z = goal.body_height
                self._set_in_motion_or_idle_body_pose(pose)
        else:
            self._set_in_motion_or_idle_body_pose(goal.body_pose)
        # Give it some time to move
        rospy.sleep(2)
        self.motion_or_idle_body_pose_as.set_succeeded(
            PoseBodyResult(
                success=True, message="Successfully applied in-motion pose to body"
            )
        )

    def _set_in_motion_or_idle_body_pose(self, pose):
        """
        Set the pose of the body which should be applied while in motion or idle

        Args:
            pose: Pose to be applied to the body. Only the body height is taken from the position component

        Returns:

        """
        q = Quaternion()
        q.x = pose.orientation.x
        q.y = pose.orientation.y
        q.z = pose.orientation.z
        q.w = pose.orientation.w
        position = geometry_pb2.Vec3(z=pose.position.z)
        pose = geometry_pb2.SE3Pose(position=position, rotation=q)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

        mobility_params = self.spot_wrapper.get_mobility_params()
        mobility_params.body_control.CopyFrom(body_control)
        self.spot_wrapper.set_mobility_params(mobility_params)

    def handle_list_graph(self, upload_path):
        """ROS service handler for listing graph_nav waypoint_ids"""
        resp = self.spot_wrapper.list_graph(upload_path)
        return ListGraphResponse(resp)

    def handle_navigate_to_feedback(self):
        """Thread function to send navigate_to feedback"""
        while not rospy.is_shutdown() and self.run_navigate_to:
            localization_state = (
                self.spot_wrapper._graph_nav_client.get_localization_state()
            )
            if localization_state.localization.waypoint_id:
                self.navigate_as.publish_feedback(
                    NavigateToFeedback(localization_state.localization.waypoint_id)
                )
            rospy.Rate(10).sleep()

    def handle_navigate_to(self, msg):
        """ROS service handler to run mission of the robot.  The robot will replay a mission"""
        if not self.robot_allowed_to_move():
            rospy.logerr("navigate_to was requested but robot is not allowed to move.")
            self.navigate_as.set_aborted(
                NavigateToResult(False, "Autonomy is not enabled")
            )
            return

        # create thread to periodically publish feedback
        feedback_thraed = threading.Thread(
            target=self.handle_navigate_to_feedback, args=()
        )
        self.run_navigate_to = True
        feedback_thraed.start()
        # run navigate_to
        resp = self.spot_wrapper.navigate_to(
            upload_path=msg.upload_path,
            navigate_to=msg.navigate_to,
            initial_localization_fiducial=msg.initial_localization_fiducial,
            initial_localization_waypoint=msg.initial_localization_waypoint,
        )
        self.run_navigate_to = False
        feedback_thraed.join()

        # check status
        if resp[0]:
            self.navigate_as.set_succeeded(NavigateToResult(resp[0], resp[1]))
        else:
            self.navigate_as.set_aborted(NavigateToResult(resp[0], resp[1]))

    def populate_camera_static_transforms(self, image_data):
        """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
        transforms. This is the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.

        Args:
        image_data: Image protobuf data from the wrapper
        """
        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the mode_parent_odom_tf, and will be published
        # by the non-static transform publishing that is done by the state callback
        excluded_frames = [
            self.tf_name_vision_odom,
            self.tf_name_kinematic_odom,
            "body",
        ]
        for frame_name in image_data.shot.transforms_snapshot.child_to_parent_edge_map:
            if frame_name in excluded_frames:
                continue
            parent_frame = (
                image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(
                    frame_name
                ).parent_frame_name
            )
            existing_transforms = [
                (transform.header.frame_id, transform.child_frame_id)
                for transform in self.sensors_static_transforms
            ]
            if (parent_frame, frame_name) in existing_transforms:
                # We already extracted this transform
                continue

            transform = (
                image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(
                    frame_name
                )
            )
            local_time = self.spot_wrapper.robotToLocalTime(
                image_data.shot.acquisition_time
            )
            tf_time = rospy.Time(local_time.seconds, local_time.nanos)
            static_tf = populateTransformStamped(
                tf_time,
                transform.parent_frame_name,
                frame_name,
                transform.parent_tform_child,
            )
            self.sensors_static_transforms.append(static_tf)
            self.sensors_static_transform_broadcaster.sendTransform(
                self.sensors_static_transforms
            )

    def populate_lidar_static_transforms(self, point_cloud_data):
        """Check data received from one of the point cloud tasks and use the transform snapshot to extract the lidar frame
        transforms. This is the transforms from body->sensor, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.

        Args:
        point_cloud_data: PointCloud protobuf data from the wrapper
        """
        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the mode_parent_odom_tf, and will be published
        # by the non-static transform publishing that is done by the state callback
        excluded_frames = [
            self.tf_name_vision_odom,
            self.tf_name_kinematic_odom,
            "body",
        ]
        for (
            frame_name
        ) in (
            point_cloud_data.point_cloud.source.transforms_snapshot.child_to_parent_edge_map
        ):
            if frame_name in excluded_frames:
                continue
            parent_frame = point_cloud_data.point_cloud.source.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name
            ).parent_frame_name
            existing_transforms = [
                (transform.header.frame_id, transform.child_frame_id)
                for transform in self.sensors_static_transforms
            ]
            if (parent_frame, frame_name) in existing_transforms:
                # We already extracted this transform
                continue

            transform = point_cloud_data.point_cloud.source.transforms_snapshot.child_to_parent_edge_map.get(
                frame_name
            )
            local_time = self.spot_wrapper.robotToLocalTime(
                point_cloud_data.point_cloud.source.acquisition_time
            )
            tf_time = rospy.Time(local_time.seconds, local_time.nanos)
            static_tf = populateTransformStamped(
                tf_time,
                transform.parent_frame_name,
                frame_name,
                transform.parent_tform_child,
            )
            self.sensors_static_transforms.append(static_tf)
            self.sensors_static_transform_broadcaster.sendTransform(
                self.sensors_static_transforms
            )

    # Arm functions ##################################################
    def handle_arm_stow(self, srv_data):
        """ROS service handler to command the arm to stow, home position"""
        resp = self.spot_wrapper.arm_stow()
        return TriggerResponse(resp[0], resp[1])

    def handle_arm_unstow(self, srv_data):
        """ROS service handler to command the arm to unstow, joints are all zeros"""
        resp = self.spot_wrapper.arm_unstow()
        return TriggerResponse(resp[0], resp[1])

    def handle_arm_joint_move(self, srv_data: ArmJointMovementRequest):
        """ROS service handler to send joint movement to the arm to execute"""
        resp = self.spot_wrapper.arm_joint_move(joint_targets=srv_data.joint_target)
        return ArmJointMovementResponse(resp[0], resp[1])

    def handle_force_trajectory(self, srv_data: ArmForceTrajectoryRequest):
        """ROS service handler to send a force trajectory up or down a vertical force"""
        resp = self.spot_wrapper.force_trajectory(data=srv_data)
        return ArmForceTrajectoryResponse(resp[0], resp[1])

    def handle_gripper_open(self, srv_data):
        """ROS service handler to open the gripper"""
        resp = self.spot_wrapper.gripper_open()
        return TriggerResponse(resp[0], resp[1])

    def handle_gripper_close(self, srv_data):
        """ROS service handler to close the gripper"""
        resp = self.spot_wrapper.gripper_close()
        return TriggerResponse(resp[0], resp[1])

    def handle_gripper_angle_open(self, srv_data: GripperAngleMoveRequest):
        """ROS service handler to open the gripper at an angle"""
        resp = self.spot_wrapper.gripper_angle_open(gripper_ang=srv_data.gripper_angle)
        return GripperAngleMoveResponse(resp[0], resp[1])

    def handle_arm_carry(self, srv_data):
        """ROS service handler to put arm in carry mode"""
        resp = self.spot_wrapper.arm_carry()
        return TriggerResponse(resp[0], resp[1])

    def handle_hand_pose(self, srv_data: HandPoseRequest):
        """ROS service to give a position to the gripper"""
        resp = self.spot_wrapper.hand_pose(data=srv_data)
        return HandPoseResponse(resp[0], resp[1])

    def handle_grasp_3d(self, srv_data: Grasp3dRequest):
        """ROS service to grasp an object by x,y,z coordinates in given frame"""
        resp = self.spot_wrapper.grasp_3d(
            frame=srv_data.frame_name,
            object_rt_frame=srv_data.object_rt_frame,
        )
        return Grasp3dResponse(resp[0], resp[1])

    ##################################################################

    def shutdown(self):
        rospy.loginfo("Shutting down ROS driver for Spot")
        self.spot_wrapper.sit()
        rospy.Rate(0.25).sleep()
        self.spot_wrapper.disconnect()

    def publish_mobility_params(self):
        mobility_params_msg = MobilityParams()
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params_msg.body_control.position.x = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.position.x
            )
            mobility_params_msg.body_control.position.y = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.position.y
            )
            mobility_params_msg.body_control.position.z = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.position.z
            )
            mobility_params_msg.body_control.orientation.x = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.rotation.x
            )
            mobility_params_msg.body_control.orientation.y = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.rotation.y
            )
            mobility_params_msg.body_control.orientation.z = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.rotation.z
            )
            mobility_params_msg.body_control.orientation.w = (
                mobility_params.body_control.base_offset_rt_footprint.points[
                    0
                ].pose.rotation.w
            )
            mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint
            mobility_params_msg.stair_hint = mobility_params.stair_hint
            mobility_params_msg.swing_height = mobility_params.swing_height
            mobility_params_msg.obstacle_params.obstacle_avoidance_padding = (
                mobility_params.obstacle_params.obstacle_avoidance_padding
            )
            mobility_params_msg.obstacle_params.disable_vision_foot_obstacle_avoidance = (
                mobility_params.obstacle_params.disable_vision_foot_obstacle_avoidance
            )
            mobility_params_msg.obstacle_params.disable_vision_foot_constraint_avoidance = (
                mobility_params.obstacle_params.disable_vision_foot_constraint_avoidance
            )
            mobility_params_msg.obstacle_params.disable_vision_body_obstacle_avoidance = (
                mobility_params.obstacle_params.disable_vision_body_obstacle_avoidance
            )
            mobility_params_msg.obstacle_params.disable_vision_foot_obstacle_body_assist = (
                mobility_params.obstacle_params.disable_vision_foot_obstacle_body_assist
            )
            mobility_params_msg.obstacle_params.disable_vision_negative_obstacles = (
                mobility_params.obstacle_params.disable_vision_negative_obstacles
            )
            if mobility_params.HasField("terrain_params"):
                if mobility_params.terrain_params.HasField("ground_mu_hint"):
                    mobility_params_msg.terrain_params.ground_mu_hint = (
                        mobility_params.terrain_params.ground_mu_hint
                    )
                    # hasfield does not work on grated surfaces mode
                if hasattr(mobility_params.terrain_params, "grated_surfaces_mode"):
                    mobility_params_msg.terrain_params.grated_surfaces_mode = (
                        mobility_params.terrain_params.grated_surfaces_mode
                    )

            # The velocity limit values can be set independently so make sure each of them exists before setting
            if mobility_params.HasField("vel_limit"):
                if hasattr(mobility_params.vel_limit.max_vel.linear, "x"):
                    mobility_params_msg.velocity_limit.linear.x = (
                        mobility_params.vel_limit.max_vel.linear.x
                    )
                if hasattr(mobility_params.vel_limit.max_vel.linear, "y"):
                    mobility_params_msg.velocity_limit.linear.y = (
                        mobility_params.vel_limit.max_vel.linear.y
                    )
                if hasattr(mobility_params.vel_limit.max_vel, "angular"):
                    mobility_params_msg.velocity_limit.angular.z = (
                        mobility_params.vel_limit.max_vel.angular
                    )
        except Exception as e:
            rospy.logerr("Error:{}".format(e))
            pass
        self.mobility_params_pub.publish(mobility_params_msg)

    def publish_feedback(self):
        feedback_msg = Feedback()
        feedback_msg.standing = self.spot_wrapper.is_standing
        feedback_msg.sitting = self.spot_wrapper.is_sitting
        feedback_msg.moving = self.spot_wrapper.is_moving
        id_ = self.spot_wrapper.id
        try:
            feedback_msg.serial_number = id_.serial_number
            feedback_msg.species = id_.species
            feedback_msg.version = id_.version
            feedback_msg.nickname = id_.nickname
            feedback_msg.computer_serial_number = id_.computer_serial_number
        except:
            pass
        self.feedback_pub.publish(feedback_msg)

    def publish_allow_motion(self):
        self.motion_allowed_pub.publish(self.allow_motion)

    def check_for_subscriber(self):
        for pub in list(self.camera_pub_to_async_task_mapping.keys()):
            task_name = self.camera_pub_to_async_task_mapping[pub]
            if (
                task_name not in self.active_camera_tasks
                and pub.get_num_connections() > 0
            ):
                self.spot_wrapper.update_image_tasks(task_name)
                self.active_camera_tasks.append(task_name)
                print(
                    f"Detected subscriber for {task_name} task, adding task to publish"
                )

    def main(self):
        """Main function for the SpotROS class. Gets config from ROS and initializes the wrapper. Holds lease from
        wrapper and updates all async tasks at the ROS rate"""
        rospy.init_node("spot_ros", anonymous=True)

        self.rates = rospy.get_param("~rates", {})
        if "loop_frequency" in self.rates:
            loop_rate = self.rates["loop_frequency"]
        else:
            loop_rate = 50

        for param, rate in self.rates.items():
            if rate > loop_rate:
                rospy.logwarn(
                    "{} has a rate of {} specified, which is higher than the loop rate of {}. It will not "
                    "be published at the expected frequency".format(
                        param, rate, loop_rate
                    )
                )

        rate = rospy.Rate(loop_rate)
        self.robot_name = rospy.get_param("~robot_name", "spot")
        self.username = rospy.get_param("~username", "default_value")
        self.password = rospy.get_param("~password", "default_value")
        self.hostname = rospy.get_param("~hostname", "default_value")
        self.motion_deadzone = rospy.get_param("~deadzone", 0.05)
        self.start_estop = rospy.get_param("~start_estop", True)
        self.estop_timeout = rospy.get_param("~estop_timeout", 9.0)
        self.autonomy_enabled = rospy.get_param("~autonomy_enabled", True)
        self.allow_motion = rospy.get_param("~allow_motion", True)
        self.use_take_lease = rospy.get_param("~use_take_lease", False)
        self.get_lease_on_action = rospy.get_param("~get_lease_on_action", False)
        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", True)
        self.is_charging = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sensors_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()
        # Static transform broadcaster is super simple and just a latched publisher. Every time we add a new static
        # transform we must republish all static transforms from this source, otherwise the tree will be incomplete.
        # We keep a list of all the static transforms we already have so they can be republished, and so we can check
        # which ones we already have
        self.sensors_static_transforms = []

        # Spot has 2 types of odometries: 'odom' and 'vision'
        # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
        # These params enables to change which odometry frame is a parent of body frame and to change tf names of each odometry frames.
        self.mode_parent_odom_tf = rospy.get_param(
            "~mode_parent_odom_tf", "odom"
        )  # 'vision' or 'odom'
        self.tf_name_kinematic_odom = rospy.get_param("~tf_name_kinematic_odom", "odom")
        self.tf_name_raw_kinematic = "odom"
        self.tf_name_vision_odom = rospy.get_param("~tf_name_vision_odom", "vision")
        self.tf_name_raw_vision = "vision"
        if (
            self.mode_parent_odom_tf != self.tf_name_raw_kinematic
            and self.mode_parent_odom_tf != self.tf_name_raw_vision
        ):
            rospy.logerr(
                "rosparam '~mode_parent_odom_tf' should be 'odom' or 'vision'."
            )
            return

        self.logger = logging.getLogger("rosout")

        rospy.loginfo("Starting ROS driver for Spot")
        self.spot_wrapper = SpotWrapper(
            username=self.username,
            password=self.password,
            hostname=self.hostname,
            robot_name=self.robot_name,
            logger=self.logger,
            start_estop=self.start_estop,
            estop_timeout=self.estop_timeout,
            rates=self.rates,
            callbacks=self.callbacks,
            use_take_lease=self.use_take_lease,
            get_lease_on_action=self.get_lease_on_action
        )

        if not self.spot_wrapper.is_valid:
            return

        # Images #
        self.back_image_pub = rospy.Publisher("camera/back/image", Image, queue_size=10)
        self.frontleft_image_pub = rospy.Publisher(
            "camera/frontleft/image", Image, queue_size=10
        )
        self.frontright_image_pub = rospy.Publisher(
            "camera/frontright/image", Image, queue_size=10
        )
        self.left_image_pub = rospy.Publisher("camera/left/image", Image, queue_size=10)
        self.right_image_pub = rospy.Publisher(
            "camera/right/image", Image, queue_size=10
        )
        self.hand_image_mono_pub = rospy.Publisher(
            "camera/hand_mono/image", Image, queue_size=10
        )
        self.hand_image_color_pub = rospy.Publisher(
            "camera/hand_color/image", Image, queue_size=10
        )

        # Depth #
        self.back_depth_pub = rospy.Publisher("depth/back/image", Image, queue_size=10)
        self.frontleft_depth_pub = rospy.Publisher(
            "depth/frontleft/image", Image, queue_size=10
        )
        self.frontright_depth_pub = rospy.Publisher(
            "depth/frontright/image", Image, queue_size=10
        )
        self.left_depth_pub = rospy.Publisher("depth/left/image", Image, queue_size=10)
        self.right_depth_pub = rospy.Publisher(
            "depth/right/image", Image, queue_size=10
        )
        self.hand_depth_pub = rospy.Publisher("depth/hand/image", Image, queue_size=10)
        self.hand_depth_in_hand_color_pub = rospy.Publisher(
            "depth/hand/depth_in_color", Image, queue_size=10
        )
        self.frontleft_depth_in_visual_pub = rospy.Publisher(
            "depth/frontleft/depth_in_visual", Image, queue_size=10
        )
        self.frontright_depth_in_visual_pub = rospy.Publisher(
            "depth/frontright/depth_in_visual", Image, queue_size=10
        )

        # EAP Pointcloud #
        self.point_cloud_pub = rospy.Publisher(
            "lidar/points", PointCloud2, queue_size=10
        )

        # Image Camera Info #
        self.back_image_info_pub = rospy.Publisher(
            "camera/back/camera_info", CameraInfo, queue_size=10
        )
        self.frontleft_image_info_pub = rospy.Publisher(
            "camera/frontleft/camera_info", CameraInfo, queue_size=10
        )
        self.frontright_image_info_pub = rospy.Publisher(
            "camera/frontright/camera_info", CameraInfo, queue_size=10
        )
        self.left_image_info_pub = rospy.Publisher(
            "camera/left/camera_info", CameraInfo, queue_size=10
        )
        self.right_image_info_pub = rospy.Publisher(
            "camera/right/camera_info", CameraInfo, queue_size=10
        )
        self.hand_image_mono_info_pub = rospy.Publisher(
            "camera/hand_mono/camera_info", CameraInfo, queue_size=10
        )
        self.hand_image_color_info_pub = rospy.Publisher(
            "camera/hand_color/camera_info", CameraInfo, queue_size=10
        )

        # Depth Camera Info #
        self.back_depth_info_pub = rospy.Publisher(
            "depth/back/camera_info", CameraInfo, queue_size=10
        )
        self.frontleft_depth_info_pub = rospy.Publisher(
            "depth/frontleft/camera_info", CameraInfo, queue_size=10
        )
        self.frontright_depth_info_pub = rospy.Publisher(
            "depth/frontright/camera_info", CameraInfo, queue_size=10
        )
        self.left_depth_info_pub = rospy.Publisher(
            "depth/left/camera_info", CameraInfo, queue_size=10
        )
        self.right_depth_info_pub = rospy.Publisher(
            "depth/right/camera_info", CameraInfo, queue_size=10
        )
        self.hand_depth_info_pub = rospy.Publisher(
            "depth/hand/camera_info", CameraInfo, queue_size=10
        )
        self.hand_depth_in_color_info_pub = rospy.Publisher(
            "camera/hand/depth_in_color/camera_info", CameraInfo, queue_size=10
        )
        self.frontleft_depth_in_visual_info_pub = rospy.Publisher(
            "depth/frontleft/depth_in_visual/camera_info", CameraInfo, queue_size=10
        )
        self.frontright_depth_in_visual_info_pub = rospy.Publisher(
            "depth/frontright/depth_in_visual/camera_info", CameraInfo, queue_size=10
        )

        self.camera_pub_to_async_task_mapping = {
            self.frontleft_image_pub: "front_image",
            self.frontleft_depth_pub: "front_image",
            self.frontleft_image_info_pub: "front_image",
            self.frontright_image_pub: "front_image",
            self.frontright_depth_pub: "front_image",
            self.frontright_image_info_pub: "front_image",
            self.back_image_pub: "rear_image",
            self.back_depth_pub: "rear_image",
            self.back_image_info_pub: "rear_image",
            self.right_image_pub: "side_image",
            self.right_depth_pub: "side_image",
            self.right_image_info_pub: "side_image",
            self.left_image_pub: "side_image",
            self.left_depth_pub: "side_image",
            self.left_image_info_pub: "side_image",
            self.hand_image_color_pub: "hand_image",
            self.hand_image_mono_pub: "hand_image",
            self.hand_image_mono_info_pub: "hand_image",
            self.hand_depth_pub: "hand_image",
            self.hand_depth_in_hand_color_pub: "hand_image",
        }

        # Status Publishers #
        self.joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )
        """Defining a TF publisher manually because of conflicts between Python3 and tf"""
        self.tf_pub = rospy.Publisher("tf", TFMessage, queue_size=10)
        self.metrics_pub = rospy.Publisher("status/metrics", Metrics, queue_size=10)
        self.lease_pub = rospy.Publisher("status/leases", LeaseArray, queue_size=10)
        self.odom_twist_pub = rospy.Publisher(
            "odometry/twist", TwistWithCovarianceStamped, queue_size=10
        )
        self.odom_pub = rospy.Publisher("odometry", Odometry, queue_size=10)
        self.odom_corrected_pub = rospy.Publisher(
            "odometry_corrected", Odometry, queue_size=10
        )
        self.feet_pub = rospy.Publisher("status/feet", FootStateArray, queue_size=10)
        self.estop_pub = rospy.Publisher("status/estop", EStopStateArray, queue_size=10)
        self.wifi_pub = rospy.Publisher("status/wifi", WiFiState, queue_size=10)
        self.power_pub = rospy.Publisher(
            "status/power_state", PowerState, queue_size=10
        )
        self.battery_pub = rospy.Publisher(
            "status/battery_states", BatteryStateArray, queue_size=10
        )
        self.behavior_faults_pub = rospy.Publisher(
            "status/behavior_faults", BehaviorFaultState, queue_size=10
        )
        self.system_faults_pub = rospy.Publisher(
            "status/system_faults", SystemFaultState, queue_size=10
        )
        self.motion_allowed_pub = rospy.Publisher(
            "status/motion_allowed", Bool, queue_size=10
        )

        self.feedback_pub = rospy.Publisher("status/feedback", Feedback, queue_size=10)

        self.mobility_params_pub = rospy.Publisher(
            "status/mobility_params", MobilityParams, queue_size=10
        )

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback, queue_size=1)
        rospy.Subscriber(
            "go_to_pose", PoseStamped, self.trajectory_callback, queue_size=1
        )
        rospy.Subscriber(
            "in_motion_or_idle_body_pose",
            Pose,
            self.in_motion_or_idle_pose_cb,
            queue_size=1,
        )
        rospy.Service("claim", Trigger, self.handle_claim)
        rospy.Service("release", Trigger, self.handle_release)
        rospy.Service("self_right", Trigger, self.handle_self_right)
        rospy.Service("sit", Trigger, self.handle_sit)
        rospy.Service("stand", Trigger, self.handle_stand)
        rospy.Service("power_on", Trigger, self.handle_power_on)
        rospy.Service("power_off", Trigger, self.handle_safe_power_off)

        rospy.Service("estop/hard", Trigger, self.handle_estop_hard)
        rospy.Service("estop/gentle", Trigger, self.handle_estop_soft)
        rospy.Service("estop/release", Trigger, self.handle_estop_disengage)

        rospy.Service("allow_motion", SetBool, self.handle_allow_motion)

        rospy.Service("stair_mode", SetBool, self.handle_stair_mode)
        rospy.Service("locomotion_mode", SetLocomotion, self.handle_locomotion_mode)
        rospy.Service("swing_height", SetSwingHeight, self.handle_swing_height)
        rospy.Service("velocity_limit", SetVelocity, self.handle_vel_limit)
        rospy.Service(
            "clear_behavior_fault", ClearBehaviorFault, self.handle_clear_behavior_fault
        )
        rospy.Service("terrain_params", SetTerrainParams, self.handle_terrain_params)
        rospy.Service("obstacle_params", SetObstacleParams, self.handle_obstacle_params)
        rospy.Service("posed_stand", PosedStand, self.handle_posed_stand)

        rospy.Service("list_graph", ListGraph, self.handle_list_graph)

        rospy.Service("roll_over_right", Trigger, self.handle_roll_over_right)
        rospy.Service("roll_over_left", Trigger, self.handle_roll_over_left)
        # Docking
        rospy.Service("dock", Dock, self.handle_dock)
        rospy.Service("undock", Trigger, self.handle_undock)
        rospy.Service("docking_state", GetDockState, self.handle_get_docking_state)
        # Arm Services #########################################
        rospy.Service("arm_stow", Trigger, self.handle_arm_stow)
        rospy.Service("arm_unstow", Trigger, self.handle_arm_unstow)
        rospy.Service("gripper_open", Trigger, self.handle_gripper_open)
        rospy.Service("gripper_close", Trigger, self.handle_gripper_close)
        rospy.Service("arm_carry", Trigger, self.handle_arm_carry)
        rospy.Service(
            "gripper_angle_open", GripperAngleMove, self.handle_gripper_angle_open
        )
        rospy.Service("arm_joint_move", ArmJointMovement, self.handle_arm_joint_move)
        rospy.Service(
            "force_trajectory", ArmForceTrajectory, self.handle_force_trajectory
        )
        rospy.Service("gripper_pose", HandPose, self.handle_hand_pose)
        rospy.Service("grasp_3d", Grasp3d, self.handle_grasp_3d)
        #########################################################

        self.navigate_as = actionlib.SimpleActionServer(
            "navigate_to",
            NavigateToAction,
            execute_cb=self.handle_navigate_to,
            auto_start=False,
        )
        self.navigate_as.start()

        self.trajectory_server = actionlib.SimpleActionServer(
            "trajectory",
            TrajectoryAction,
            execute_cb=self.handle_trajectory,
            auto_start=False,
        )
        self.trajectory_server.start()

        self.motion_or_idle_body_pose_as = actionlib.SimpleActionServer(
            "motion_or_idle_body_pose",
            PoseBodyAction,
            execute_cb=self.handle_in_motion_or_idle_body_pose,
            auto_start=False,
        )
        self.motion_or_idle_body_pose_as.start()

        self.body_pose_as = actionlib.SimpleActionServer(
            "body_pose",
            PoseBodyAction,
            execute_cb=self.handle_posed_stand_action,
            auto_start=False,
        )
        self.body_pose_as.start()

        self.dock_as = actionlib.SimpleActionServer(
            "dock",
            DockAction,
            execute_cb=self.handle_dock_action,
            auto_start=False,
        )
        self.dock_as.start()

        # Stop service calls other services so initialise it after them to prevent crashes which can happen if
        # the service is immediately called
        rospy.Service("stop", Trigger, self.handle_stop)
        rospy.Service("locked_stop", Trigger, self.handle_locked_stop)

        rospy.on_shutdown(self.shutdown)

        max_linear_x = rospy.get_param("~max_linear_velocity_x", 0)
        max_linear_y = rospy.get_param("~max_linear_velocity_y", 0)
        max_angular_z = rospy.get_param("~max_angular_velocity_z", 0)
        self.set_velocity_limits(max_linear_x, max_linear_y, max_angular_z)

        self.auto_claim = rospy.get_param("~auto_claim", False)
        self.auto_power_on = rospy.get_param("~auto_power_on", False)
        self.auto_stand = rospy.get_param("~auto_stand", False)

        if self.auto_claim:
            self.spot_wrapper.claim()
            if self.auto_power_on:
                self.spot_wrapper.power_on()
                if self.auto_stand:
                    self.spot_wrapper.stand()

        rate_limited_feedback = RateLimitedCall(
            self.publish_feedback, self.rates["feedback"]
        )
        rate_limited_mobility_params = RateLimitedCall(
            self.publish_mobility_params, self.rates["mobility_params"]
        )
        rate_check_for_subscriber = RateLimitedCall(
            self.check_for_subscriber, self.rates["check_subscribers"]
        )
        rate_limited_motion_allowed = RateLimitedCall(self.publish_allow_motion, 10)
        rospy.loginfo("Driver started")
        while not rospy.is_shutdown():
            self.spot_wrapper.updateTasks()
            rate_limited_feedback()
            rate_limited_mobility_params()
            rate_limited_motion_allowed()
            rate_check_for_subscriber()
            rate.sleep()
