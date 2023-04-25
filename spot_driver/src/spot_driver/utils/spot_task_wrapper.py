# Spot wrapper
from ..spot_wrapper import SpotWrapper

# Messages
from bosdyn.api import (robot_command_pb2,
                        geometry_pb2,
                        mobility_command_pb2, 
                        basic_command_pb2,
                        arm_command_pb2)

from bosdyn.api.arm_command_pb2 import (ArmCommand, ArmCartesianCommand)
from bosdyn.api.basic_command_pb2 import (ArmDragCommand)
from bosdyn.api.robot_command_pb2 import (RobotCommand)
from bosdyn.api.synchronized_command_pb2 import (SynchronizedCommand)

from bosdyn.client.math_helpers import SE2Pose as bdSE2Pose
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat    as bdQuat



# Clients
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, 
                                         ODOM_FRAME_NAME, 
                                         VISION_FRAME_NAME,
                                         get_se2_a_tform_b,
                                         )

from bosdyn.client.robot_command import (block_until_arm_arrives, block_for_trajectory_cmd)


from bosdyn.client.robot_command import RobotCommandBuilder as CmdBuilder

# Others
from bosdyn.util import seconds_to_duration

import numpy as np
import logging
import time



# Temp
from bosdyn.client import math_helpers

class SpotTaskWrapper:

    def __init__(self, spot: SpotWrapper, logger:logging.Logger=None):
        self.spot = spot
        self._robot = spot._robot
        self._init_logger(logger)
        assert self._robot.has_arm(), "Robot requires an arm to run this example."
        self._manip_cli = self._robot.ensure_client(ManipulationApiClient.default_service_name)
        self.default_ref_frame = ODOM_FRAME_NAME  # VISION_FRAME_NAME, ODOM_FRAME_NAME
        self._log.info('Task Wrapper Initialized.')

    def _init_logger(self, logger:logging.Logger=None):
        class LogHandler(logging.Handler):
            def __init__(self, ref): 
                super().__init__()
                self._ref = ref
            def emit(self, record): 
                self._ref._string_feedback = str(record)
        self._log_handler = LogHandler(self)
        if logger is None: logger = logging.getLogger(__name__)
        logger.addHandler(self._log_handler)
        self._log = logger

    @property
    def feedback(self): return self._string_feedback

    def _pose_np_to_bd(self, pose:np.array, se3=False):
        if pose.shape == (3,3):
            pose = bdSE2Pose.from_matrix(pose)
            if se3: pose = pose.get_closest_se3_transform()
        else: 
            pose = bdSE3Pose.from_matrix(pose)
        return pose    
     
    def _pose_bd_to_vectors(self, pose:bdSE3Pose):
        pos = [pose.position.x, pose.position.y, pose.position.z]
        rot = [pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z]
        return pos, rot

    def _offset_pose(self, pose:bdSE3Pose, distance, axis):
        all_axis = {'x':0, 'y':1, 'z':2}
        dir = np.eye(3)[all_axis[axis]] 
        offset = np.eye(4)
        offset[0:3, 3] = -dir * distance
        offset = bdSE3Pose.from_matrix(offset)
        return pose * offset

    def go_to(self, 
              pose, 
              relative_frame:str, 
              distance:float=0.0, 
              dir_axis:str='x', 
              up_axis:str='z', 
              body_height=None, 
              blocking=True):
        '''Moves the robot to a desired pose.'''
        if isinstance(pose, np.ndarray):
            pose = self._pose_np_to_bd(pose, se3=True)
        pose = self._offset_pose(pose, distance, dir_axis)
        pos, rot = pose.position, pose.rotation

        heading = rot.to_roll() if up_axis == 'x'\
             else rot.to_pitch() if up_axis == 'y'\
             else rot.to_yaw()        
    
        # TODO: handle this response
        self.spot.trajectory_cmd(
            goal_x=pos.x, goal_y=pos.y, 
            goal_heading=heading,
            cmd_duration=5, reference_frame=relative_frame,
            blocking=blocking,
        )

    def _end_grasp(self):
        self.spot.arm_stow()
        self.spot.gripper_close()

    def _to_bd_se3(self, pose, ref, se3=True):
        '''Changes input pose into a bd se3 pose.'''
        if isinstance(pose, np.ndarray):
            pose = self._pose_np_to_bd(pose, se3=se3)
        pose = self.spot._transform_bd_pose(pose, ref, self.default_ref_frame)
        return pose

    def grasp(self, pose, reference_frame:str, **kwargs):
        self._log.info(f'Grasping pose {pose} in frame {reference_frame}')
        pose = self._to_bd_se3(pose, reference_frame)

        self._log.info('Approaching desired robot pose.')
        self.go_to(pose, self.default_ref_frame, distance=1.0, **kwargs)

        def _to(p, d):
            pos, rot = self._pose_bd_to_vectors(p)
            status, msg = self.spot.hand_pose(pos, rot, 
                                reference_frame=self.default_ref_frame, 
                                duration=d)
            self._log.info(f'status: {msg}')
            if status is False: 
                self._end_grasp()
                raise(Exception('Failed...'))

        pre_grasp = self._offset_pose(pose, 0.25, 'x')
        self._log.info(f'Approaching Pre-grasp...')
        _to(pre_grasp, 1.5)
        self._log.info('Approaching object...')
        self.spot.gripper_open()
        _to(pose, 1.0)
        self.spot.gripper_close()
        self._log.info('Succeeded')
        return True
    

    def move_object(self, pose, reference_frame:str, **kwargs):
        '''Commands the robot to move an object to a desired pose.'''
        self._log.info(f'Moving object to pose {pose} in frame {reference_frame}')
        # return self._follow_arm_to(pose, reference_frame)
        return self._joint_body_arm_impedance_cmd(pose, reference_frame)

    def _follow_arm_to(self, pose, reference_frame):
        pose = self._to_bd_se3(pose, reference_frame)

        # Create the arm & body command.
        arm_command = CmdBuilder.arm_pose_command(
            pose.x, pose.y, pose.z, 
            pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z, 
            self.default_ref_frame, 
            2.0)
        follow_arm_command = CmdBuilder.follow_arm_command()
        command = CmdBuilder.build_synchro_command(follow_arm_command, 
                                                   arm_command)

        # Send the request
        cmd_client = self.spot._robot_command_client
        move_command_id = cmd_client.robot_command(command)
        self._log.info('Moving arm to position.')

        block_until_arm_arrives(cmd_client, move_command_id, 6.0)
        self._log.info('Succeeded')
        return True
    
    def _drag_arm_to(self, pose:bdSE3Pose, reference_frame):

        self._log.info(f'Building drag command.')
        robot_cmd = RobotCommand(
            synchronized_command = SynchronizedCommand.Request(
                arm_command = ArmCommand.Request(
                    arm_drag_command = ArmDragCommand.Request()
                )
            )
        )
        self._log.info(f'Building se2 command.')
        pose = pose.get_closest_se2_transform()
        robot_cmd = CmdBuilder.synchro_se2_trajectory_point_command(
            goal_x=pose.x,
            goal_y=pose.y,
            goal_heading=pose.angle,
            frame_name=reference_frame,
            params=CmdBuilder.mobility_params(),
            build_on_command=robot_cmd
        )

        ### robot_cmd = robot_command_pb2.RobotCommand()
        ### robot_cmd.synchronized_command.mobility_command.CopyFrom(mobility_cmd)
        ### robot_cmd.synchronized_command.arm_command.CopyFrom(arm_cmd)
        
        self._log.info(f'Sending Robot Command.')
        self._log.info(f'Robot Command: {robot_cmd}')
        end_time = 5.0
        cmd_id = self.spot._robot_command_client.robot_command(command=robot_cmd, 
                                                               lease=None, 
                                                               end_time_secs=time.time() + end_time)
        block_for_trajectory_cmd(self.spot._robot_command_client, cmd_id, 
                                 feedback_interval_secs=0.5, 
                                 timeout_sec=end_time,
                                 logger=self._log)
        
        # time.sleep(5.0)

    def _joint_mobility_arm_cmd(self, pose, reference_frame):
        '''Command the arm to a certain pose and request that the 
        body follows. This does not expect there to be any contact.'''
        raise NotImplementedError('This is not currently supported by the robot.')


    def _joint_body_arm_impedance_cmd(self, pose, reference_frame):
        '''Commands the robot to move an object to a desired pose.
        This involves dragging the object near the goal pose, then
        moving the arm to properly position it.
        
        Ideally this would be done using the ArmImpedanceCommand, but
        this is not currently supported by the robot.
        Instead ArmDragCommand has to first be used followed by 
        Arm impedance command.'''

        pose = self._to_bd_se3(pose, reference_frame)

        # Check if pose is farther than some threshold
        # If so, use drag command to move it closer.
        in_body = self.spot._transform_bd_pose(pose, 
                                        self.default_ref_frame, 
                                        BODY_FRAME_NAME)
        body_dist = np.linalg.norm([in_body.x, in_body.y, in_body.z])
        if body_dist > 0.2:
            self._log.info(f'Object is too far away. Dragging it closer.')

            # Define where the object should stand to position the object.
            stance_t = [-0.2, 0.2, 0.0]
            stance_R = bdQuat.from_yaw(np.pi/2)
            stance = pose * bdSE3Pose(stance_t, stance_R)
            
            self._drag_arm_to(stance, self.default_ref_frame)

        return True
        # # To move the object to place, we could move the arm then 
        # # move the robot body based on some heuristic.




        # #  # Create the arm & body command.
        # # arm_command = CmdBuilder.arm_pose_command(
        # #     pose.x, pose.y, pose.z, 
        # #     pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z, 
        # #     self.default_ref_frame, 
        # #     2.0)
        
        # robot_cmd = robot_command_pb2.RobotCommand()
        
        # mobility_command = mobility_command_pb2.MobilityCommand.Request(
        #     follow_arm_request=basic_command_pb2.FollowArmCommand.Request())
        # robot_cmd.synchronized_command.mobility_command.CopyFrom(mobility_command)

        # arm_cmd = robot_cmd.synchronized_command.arm_command.arm_impedance_command

        # # Set up our root frame, task frame, and tool frame.
        # # NOTE: root_tform_task, wrist_tform_tool can be set to identify
        # #   the desired task frame and tool frame. Check arm command proto 
        # #   for more details.
        # arm_cmd.root_frame_name = self.default_ref_frame

        # # Set up downward force.
        # ffwd_wrench = arm_cmd.feed_forward_wrench_at_tool_in_desired_tool
        # ffwd_wrench.force.x = -2.0  # Newtons
        # ffwd_wrench.force.y = 0
        # ffwd_wrench.force.z = 0
        # ffwd_wrench.torque.x = -0.5
        # ffwd_wrench.torque.y = 0
        # ffwd_wrench.torque.z = 0

        # # Set up stiffness and damping matrices.
        # # NOTE: Max stiffness: [500, 500, 500, 60, 60, 60]
        # #      Max damping: [2.5, 2.5, 2.5, 1.0, 1.0, 1.0]
        # arm_cmd.diagonal_stiffness_matrix.CopyFrom(
        #     geometry_pb2.Vector(values=[200, 200, 200, 30, 30, 30]))
        # arm_cmd.diagonal_damping_matrix.CopyFrom(
        #     geometry_pb2.Vector(values=[1.5, 1.5, 1.5, 0.5, 0.5, 0.5]))

        # # Set up our `desired_tool` trajectory. This is where we want the tool to be with respect to
        # # the task frame. The stiffness we set will drag the tool towards `desired_tool`.
        # traj = arm_cmd.task_tform_desired_tool
        # pt1 = traj.points.add()
        # pt1.time_since_reference.CopyFrom(seconds_to_duration(4.0))
        # pt1.pose.CopyFrom(pose.to_proto())
        # # pt2 = traj.points.add()
        # # pt2.time_since_reference.CopyFrom(seconds_to_duration(4.0))
        # # pose.x -= 0.1
        # # pt2.pose.CopyFrom(pose.to_proto())

        # # Execute the impedance command
        # # follow_arm_cmd = CmdBuilder.follow_arm_command()
        # # cmd = CmdBuilder.build_synchro_command(follow_arm_cmd, robot_cmd)
        # cmd_id = self.spot._robot_command_client.robot_command(robot_cmd)
        # time.sleep(5.0)