import time
import typing
import logging

from bosdyn.util import seconds_to_duration
from bosdyn.client.frame_helpers import (
    ODOM_FRAME_NAME,
    BODY_FRAME_NAME,
)
from bosdyn.client import robot_command
from bosdyn.client.robot import Robot
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import robot_command_pb2
from bosdyn.api import arm_command_pb2
from bosdyn.api import synchronized_command_pb2
from bosdyn.api import geometry_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.api import manipulation_api_pb2
from google.protobuf.duration_pb2 import Duration

from geometry_msgs.msg import Pose
from spot_msgs.srv import HandPose, HandPoseResponse, HandPoseRequest


class SpotArm:
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_params: typing.Dict[str, typing.Any],
        robot_clients: typing.Dict[str, typing.Any],
    ):
        """
        Constructor for SpotArm class.
        :param robot: Robot object
        :param logger: Logger object
        :param robot_params: Dictionary of robot parameters
            - robot_params['is_standing']: True if robot is standing, False otherwise
        :param robot_clients: Dictionary of robot clients
            - robot_clients['robot_command_client']: RobotCommandClient object
            - robot_clients['robot_command_method']: RobotCommand method
        """
        self._robot = robot
        self._logger = logger
        self._robot_params = robot_params
        self._robot_command_client: RobotCommandClient = robot_clients[
            "robot_command_client"
        ]
        self._manipulation_client: ManipulationApiClient = robot_clients[
            "manipulation_client"
        ]
        self._robot_command: typing.Callable = robot_clients["robot_command_method"]

    def ensure_arm_power_and_stand(self) -> typing.Tuple[bool, str]:
        if not self._robot.has_arm():
            return False, "Spot with an arm is required for this service"

        try:
            self._logger.info("Spot is powering on within the timeout of 20 secs")
            self._robot.power_on(timeout_sec=20)
            assert self._robot.is_powered_on(), "Spot failed to power on"
            self._logger.info("Spot is powered on")
        except Exception as e:
            return (
                False,
                "Exception occured while Spot or its arm were trying to power on",
            )

        if not self._robot_params["is_standing"]:
            robot_command.blocking_stand(
                command_client=self._robot_command_client, timeout_sec=10
            )
            self._logger.info("Spot is standing")
        else:
            self._logger.info("Spot is already standing")

        return True, "Spot has an arm, is powered on, and standing"

    def arm_stow(self) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Stow Arm
                stow = RobotCommandBuilder.arm_stow_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(stow)
                self._logger.info("Command stow issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while trying to stow"

        return True, "Stow arm success"

    def arm_unstow(self) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Unstow Arm
                unstow = RobotCommandBuilder.arm_ready_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(unstow)
                self._logger.info("Command unstow issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while trying to unstow"

        return True, "Unstow arm success"

    def arm_carry(self) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Get Arm in carry mode
                carry = RobotCommandBuilder.arm_carry_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(carry)
                self._logger.info("Command carry issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while carry mode was issued"

        return True, "Carry mode success"

    def make_arm_trajectory_command(
        self, arm_joint_trajectory: arm_command_pb2.ArmJointTrajectory
    ) -> robot_command_pb2.RobotCommand:
        """Helper function to create a RobotCommand from an ArmJointTrajectory.
        Copy from 'spot-sdk/python/examples/arm_joint_move/arm_joint_move.py'"""

        joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(
            trajectory=arm_joint_trajectory
        )
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_joint_move_command=joint_move_command
        )
        sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        arm_sync_robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=sync_arm
        )
        return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)

    def arm_joint_move(self, joint_targets) -> typing.Tuple[bool, str]:
        # All perspectives are given when looking at the robot from behind after the unstow service is called
        # Joint1: 0.0 arm points to the front. positive: turn left, negative: turn right)
        # RANGE: -3.14 -> 3.14
        # Joint2: 0.0 arm points to the front. positive: move down, negative move up
        # RANGE: 0.4 -> -3.13 (
        # Joint3: 0.0 arm straight. moves the arm down
        # RANGE: 0.0 -> 3.1415
        # Joint4: 0.0 middle position. negative: moves ccw, positive moves cw
        # RANGE: -2.79253 -> 2.79253
        # # Joint5: 0.0 gripper points to the front. positive moves the gripper down
        # RANGE: -1.8326 -> 1.8326
        # Joint6: 0.0 Gripper is not rolled, positive is ccw
        # RANGE: -2.87 -> 2.87
        # Values after unstow are: [0.0, -0.9, 1.8, 0.0, -0.9, 0.0]
        if abs(joint_targets[0]) > 3.14:
            msg = "Joint 1 has to be between -3.14 and 3.14"
            self._logger.warn(msg)
            return False, msg
        elif joint_targets[1] > 0.4 or joint_targets[1] < -3.13:
            msg = "Joint 2 has to be between -3.13 and 0.4"
            self._logger.warn(msg)
            return False, msg
        elif joint_targets[2] > 3.14 or joint_targets[2] < 0.0:
            msg = "Joint 3 has to be between 0.0 and 3.14"
            self._logger.warn(msg)
            return False, msg
        elif abs(joint_targets[3]) > 2.79253:
            msg = "Joint 4 has to be between -2.79253 and 2.79253"
            self._logger.warn(msg)
            return False, msg
        elif abs(joint_targets[4]) > 1.8326:
            msg = "Joint 5 has to be between -1.8326 and 1.8326"
            self._logger.warn(msg)
            return False, msg
        elif abs(joint_targets[5]) > 2.87:
            msg = "Joint 6 has to be between -2.87 and 2.87"
            self._logger.warn(msg)
            return False, msg
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                trajectory_point = (
                    RobotCommandBuilder.create_arm_joint_trajectory_point(
                        joint_targets[0],
                        joint_targets[1],
                        joint_targets[2],
                        joint_targets[3],
                        joint_targets[4],
                        joint_targets[5],
                    )
                )
                arm_joint_trajectory = arm_command_pb2.ArmJointTrajectory(
                    points=[trajectory_point]
                )
                arm_command = self.make_arm_trajectory_command(arm_joint_trajectory)

                # Send the request
                cmd_id = self._robot_command_client.robot_command(arm_command)

                # Query for feedback to determine how long it will take
                feedback_resp = self._robot_command_client.robot_command_feedback(
                    cmd_id
                )
                joint_move_feedback = (
                    feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
                )
                time_to_goal: Duration = joint_move_feedback.time_to_goal
                time_to_goal_in_seconds: float = time_to_goal.seconds + (
                    float(time_to_goal.nanos) / float(10**9)
                )
                time.sleep(time_to_goal_in_seconds)
                return True, "Spot Arm moved successfully"

        except Exception as e:
            return False, "Exception occured during arm movement: " + str(e)

    def force_trajectory(self, data):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:

                def create_wrench_from_msg(forces, torques):
                    force = geometry_pb2.Vec3(x=forces[0], y=forces[1], z=forces[2])
                    torque = geometry_pb2.Vec3(x=torques[0], y=torques[1], z=torques[2])
                    return geometry_pb2.Wrench(force=force, torque=torque)

                # Duration in seconds.
                traj_duration = data.duration

                # first point on trajectory
                wrench0 = create_wrench_from_msg(data.forces_pt0, data.torques_pt0)
                t0 = seconds_to_duration(0)
                traj_point0 = trajectory_pb2.WrenchTrajectoryPoint(
                    wrench=wrench0, time_since_reference=t0
                )

                # Second point on the trajectory
                wrench1 = create_wrench_from_msg(data.forces_pt1, data.torques_pt1)
                t1 = seconds_to_duration(traj_duration)
                traj_point1 = trajectory_pb2.WrenchTrajectoryPoint(
                    wrench=wrench1, time_since_reference=t1
                )

                # Build the trajectory
                trajectory = trajectory_pb2.WrenchTrajectory(
                    points=[traj_point0, traj_point1]
                )

                # Build the trajectory request, putting all axes into force mode
                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=data.frame,
                    wrench_trajectory_in_task=trajectory,
                    x_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    y_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    z_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rx_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    ry_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rz_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                )
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command
                )
                synchronized_command = (
                    synchronized_command_pb2.SynchronizedCommand.Request(
                        arm_command=arm_command
                    )
                )
                robot_command = robot_command_pb2.RobotCommand(
                    synchronized_command=synchronized_command
                )

                # Send the request
                self._robot_command_client.robot_command(robot_command)
                self._logger.info("Force trajectory command sent")

                time.sleep(float(traj_duration) + 1.0)

        except Exception as e:
            return False, "Exception occured during arm movement"

        return True, "Moved arm successfully"

    def gripper_open(self) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Open gripper
                command = RobotCommandBuilder.claw_gripper_open_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open sent")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Open gripper success"

    def gripper_close(self) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Close gripper
                command = RobotCommandBuilder.claw_gripper_close_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper close sent")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Closed gripper successfully"

    def gripper_angle_open(self, gripper_ang: float) -> typing.Tuple[bool, str]:
        # takes an angle between 0 (closed) and 90 (fully opened) and opens the
        # gripper at this angle
        if gripper_ang > 90 or gripper_ang < 0:
            return False, "Gripper angle must be between 0 and 90"
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # The open angle command does not take degrees but the limits
                # defined in the urdf, that is why we have to interpolate
                closed = 0.349066
                opened = -1.396263
                angle = gripper_ang / 90.0 * (opened - closed) + closed
                command = RobotCommandBuilder.claw_gripper_open_angle_command(angle)

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open angle sent")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Opened gripper successfully"

    def hand_pose(self, data: HandPoseRequest):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                pose_point = data.pose_point
                # Move the arm to a spot in front of the robot given a pose for the gripper.
                # Build a position to move the arm to (in meters, relative to the body frame origin.)
                position = geometry_pb2.Vec3(
                    x=pose_point.position.x,
                    y=pose_point.position.y,
                    z=pose_point.position.z,
                )

                # # Rotation as a quaternion.
                rotation = geometry_pb2.Quaternion(
                    w=pose_point.orientation.w,
                    x=pose_point.orientation.x,
                    y=pose_point.orientation.y,
                    z=pose_point.orientation.z,
                )

                seconds = data.duration
                duration = seconds_to_duration(seconds)

                # Build the SE(3) pose of the desired hand position in the moving body frame.
                hand_pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
                hand_pose_traj_point = trajectory_pb2.SE3TrajectoryPoint(
                    pose=hand_pose, time_since_reference=duration
                )
                hand_trajectory = trajectory_pb2.SE3Trajectory(
                    points=[hand_pose_traj_point]
                )

                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=data.frame,
                    pose_trajectory_in_task=hand_trajectory,
                    force_remain_near_current_joint_configuration=True,
                )
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command
                )
                synchronized_command = (
                    synchronized_command_pb2.SynchronizedCommand.Request(
                        arm_command=arm_command
                    )
                )

                robot_command = robot_command_pb2.RobotCommand(
                    synchronized_command=synchronized_command
                )

                command = RobotCommandBuilder.build_synchro_command(robot_command)

                # Send the request
                self._robot_command_client.robot_command(robot_command)
                self._logger.info("Moving arm to position.")

                time.sleep(2.0)

        except Exception as e:
            return (
                False,
                "An error occured while trying to move arm \n Exception:" + str(e),
            )

        return True, "Moved arm successfully"

    def grasp_3d(self, frame, object_rt_frame):
        try:
            frm = str(frame)
            pos = geometry_pb2.Vec3(
                x=object_rt_frame[0], y=object_rt_frame[1], z=object_rt_frame[2]
            )

            grasp = manipulation_api_pb2.PickObject(frame_name=frm, object_rt_frame=pos)

            # Ask the robot to pick up the object
            grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                pick_object=grasp
            )
            # Send the request
            cmd_response = self._manipulation_client.manipulation_api_command(
                manipulation_api_request=grasp_request
            )

            # Get feedback from the robot
            while True:
                feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                    manipulation_cmd_id=cmd_response.manipulation_cmd_id
                )

                # Send the request
                response = self._manipulation_client.manipulation_api_feedback_command(
                    manipulation_api_feedback_request=feedback_request
                )

                print(
                    "Current state: ",
                    manipulation_api_pb2.ManipulationFeedbackState.Name(
                        response.current_state
                    ),
                )

                if (
                    response.current_state
                    == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED
                    or response.current_state
                    == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED
                ):
                    break

                time.sleep(0.25)

            self._robot.logger.info("Finished grasp.")

        except Exception as e:
            return False, "An error occured while trying to grasp from pose"

        return True, "Grasped successfully"
