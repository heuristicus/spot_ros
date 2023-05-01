# Spot wrapper
from ..spot_wrapper import SpotWrapper

# Messages
from bosdyn.api import (robot_command_pb2,
                        geometry_pb2,
                        mobility_command_pb2, 
                        basic_command_pb2,
                        arm_command_pb2)
from bosdyn.api.spot.robot_command_pb2 import (BodyControlParams, 
                                               MobilityParams)

from bosdyn.api.arm_command_pb2 import (ArmCommand, ArmCartesianCommand)
from bosdyn.api.basic_command_pb2 import (ArmDragCommand)
from bosdyn.api.robot_command_pb2 import (RobotCommand )
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

    def go_to(self, pose, relative_frame:str, 
              distance:float=0.0, dir_axis:str='x', 
              up_axis:str='z', blocking=True):
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
            cmd_duration=10, 
            reference_frame=relative_frame,
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
        if not self.spot.arm_stow()[0]:
            raise Exception('Failed to stow arm.')

        self._log.debug(f'Grasping pose {pose} in frame {reference_frame}')
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
        return self._move_heavy_object(pose, reference_frame)

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
    
    def _drag_arm_to(self, pose:bdSE3Pose, reference_frame, duration_sec=15.0):
        '''Commands the robot to position the robot at the commanded
        pose while leaving the arm commpliant thus allowing it to drag
        objects.
        Args:
            pose: The desired pose of the robot.
            reference_frame: The frame in which the pose is defined.
        returns:
                '''
        self._log.info(f'Building drag command.')
        robot_cmd = RobotCommand(
            synchronized_command = SynchronizedCommand.Request(
                arm_command = ArmCommand.Request(
                    arm_drag_command = ArmDragCommand.Request()
                )
            )
        )
        # Set the claw to apply force        
        robot_cmd = CmdBuilder.claw_gripper_close_command(robot_cmd) 

        pose = pose.get_closest_se2_transform()
        self._log.info(f'Sending Robot Command.')
        succeeded, _, id = self.spot.trajectory_cmd(
            goal_x=pose.x, goal_y=pose.y,
            goal_heading=pose.angle,
            cmd_duration=duration_sec, 
            reference_frame=reference_frame,
            blocking=False,
            build_on_command=robot_cmd
        )
        if succeeded:
            block_for_trajectory_cmd(self.spot._robot_command_client,
                                     cmd_id=id, 
                                     feedback_interval_secs=0.5, 
                                     timeout_sec=duration_sec,
                                     logger=self._log)
        else: 
            self._log.error('Failed to send trajectory command.')
            return False
        return True
    
    def _get_body_assist_stance_command(self, build_on_command=None):
        '''A assistive stance is used when manipulating heavy
        objects or interacting with the environment. 
        
        Returns: A body assist stance command'''
        manip_assist = BodyControlParams.BodyAssistForManipulation(
                            enable_hip_height_assist=True, 
                            enable_body_yaw_assist=False
                        )
        body_control = BodyControlParams(
                            body_assist_for_manipulation=manip_assist
                        )
        stand_command = CmdBuilder.synchro_stand_command(
                        params=MobilityParams(body_control=body_control),
                        build_on_command=build_on_command
                    )
        return stand_command


    def _arm_impedance_cmd(self, pose, reference_frame, duration_sec=10.0):
        '''Commands the robot arm to perform an impedance command
        which allows it to move heavy objects or perform surface 
        interaction.
        This will force the robot to stand and support the arm. 
        Args:
            pose: The desired end-effector pose.
            reference_frame: The frame in which the pose is defined.
            duration_sec: The max duration given for the command to conclude.
        Returns:
            True if the command succeeded, False otherwise.
        TODO:
            - Try to apply and retain force on the gripper
            - If no object is in hand, raise an error
            - Try work with stiffness
            
        Reference: Spot sdk python examples: arm_impedance_control.py'''



        robot_cmd = self._get_body_assist_stance_command()
        arm_cmd = robot_cmd.synchronized_command.arm_command.arm_impedance_command
        
        # Set the reference frame
        arm_cmd.root_frame_name = reference_frame

        # Set up stiffness and damping matrices.
        # NOTE: Max stiffness: [500, 500, 500, 60, 60, 60]
        #      Max damping: [2.5, 2.5, 2.5, 1.0, 1.0, 1.0]
        arm_cmd.diagonal_stiffness_matrix.CopyFrom(
            geometry_pb2.Vector(values=[200, 200, 200, 30, 30, 30]))
        arm_cmd.diagonal_damping_matrix.CopyFrom(
            geometry_pb2.Vector(values=[1.5, 1.5, 1.5, 0.5, 0.5, 0.5]))

        # Set up our `desired_tool` trajectory.
        traj = arm_cmd.task_tform_desired_tool
        pt1 = traj.points.add()
        pt1.time_since_reference.CopyFrom(seconds_to_duration(4.0))
        pt1.pose.CopyFrom(pose.to_proto())

        # Set the claw to apply force        
        robot_cmd = CmdBuilder.claw_gripper_close_command(robot_cmd) 



        # Execute the impedance command
        cmd_id = self.spot._robot_command_client.robot_command(robot_cmd)
        succeeded = block_until_arm_arrives(self.spot._robot_command_client, 
                                            cmd_id, self._log,
                                            timeout_sec=duration_sec)
        return succeeded


    def _joint_mobility_arm_cmd(self, pose, reference_frame):
        '''Command the arm to a certain pose and request that the 
        body follows. This does not expect there to be any contact.'''
        raise NotImplementedError('This is not currently supported by the robot.')


    def _move_heavy_object(self, pose, reference_frame):
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
            t = [-1.0, 0.5, 0.0]
            R = bdQuat.from_yaw(-np.pi/4)  # np.pi/2
            stance = pose * bdSE3Pose(x=t[0], y=t[1], z=t[2], rot=R)
            self._drag_arm_to(stance, self.default_ref_frame,
                              duration_sec=30.0)
        # Move arm to adjust pose. 
        self._arm_impedance_cmd(pose, self.default_ref_frame,
                                duration_sec=15.0)