# Spot wrapper
from .spot_wrapper import SpotWrapper

# Clients
from bosdyn.client.manipulation_api_client import ManipulationApiClient

# Messages
from google.protobuf.wrappers_pb2 import FloatValue
from bosdyn.api import manipulation_api_pb2
from bosdyn.api.manipulation_api_pb2 import (WalkToObjectRayInWorld, 
                                             ManipulationApiRequest, 
                                             ManipulationApiFeedbackRequest,
                                             )

from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion

from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)



import bosdyn.client
import bosdyn.client.util
from bosdyn.api import basic_command_pb2
from bosdyn.api import geometry_pb2 as geo
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient


# others
import numpy as np
import time
from enum import Enum
from scipy.spatial.transform import Rotation as R 

class SpotTaskWrapper:

    def __init__(self, spot: SpotWrapper):
        self.spot = spot
        self._robot = spot._robot


        assert self._robot.has_arm(), "Robot requires an arm to run this example."

        self._manip_cli = self._robot.ensure_client(ManipulationApiClient.default_service_name)
        self._string_feedback = ''


    @property
    def feedback(self): return self._string_feedback

    def walk_to(self, dx, dy, dyaw, frame_name, robot_command_client, robot_state_client, stairs=False):
        frame_tree = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
        # We do not want to command this goal in body frame because the body will move, thus shifting
        # our goal. Instead, we transform this offset to get the goal position in the output frame
        # (which will be either odom or vision).
        out_tform_body = get_se2_a_tform_b(frame_tree, frame_name, BODY_FRAME_NAME)
        out_tform_goal = out_tform_body * body_tform_goal

        # Command the robot to go to the goal point in the specified frame. The command will stop at the
        # new position.
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
        end_time = 10.0
        cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                    end_time_secs=time.time() + end_time)
        # Wait until the robot has reached the goal.
        while True:
            feedback = robot_command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                print("Failed to reach the goal")
                return False
            traj_feedback = mobility_feedback.se2_trajectory_feedback
            if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                    traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
                print("Arrived at the goal.")
                return True
            time.sleep(1)


    def _to_se3(self, pose:np.array):
        if pose.shape == (3,3):
            pose3 = np.eye(4)
            pose3[0:2, 0:2], pose3[0:2, 3] = pose[0:2, 0:2], pose[0:2, 2]
            pose = pose3
        elif pose.shape == (3,) or pose.shape == (3,1):
            pose3 = np.eye(4); pose3[0:3, 3] = pose
            pose = pose3
        elif pose.shape == (2,) or pose.shape == (2,1):
            pose3 = np.eye(4); pose3[0:2, 3] = pose
            pose = pose3
        return pose

    def _offset_pose(self, pose, distance, axis):
        all_axis = {'x':0, 'y':1, 'z':2}
        dir = np.eye(3)[all_axis[axis]] 
        pose = self._to_se3(pose)
        offset = np.eye(4)
        offset[0:3, 3] = -dir * distance
        return pose @ offset

    def go_to(self, 
              pose:np.array, 
              relative_frame:str, 
              distance:float=0.25, 
              dir_axis:str='x', 
              up_axis:str='z', 
              body_height=None, 
              blocking=True):
        '''
        TODO: Document
        '''
        axis = {'x':0, 'y':1, 'z':2}
        dir = np.eye(3)[axis[dir_axis]] 

        pose = self._to_se3(pose) # Treat it as se3

        offset = np.concatenate((-dir * distance, np.array([1])))
        position = (pose @ offset)[0:3]
        pose_rot = R.from_matrix(pose[:3,:3])
        pose_euler = pose_rot.as_euler(''.join([str(k).lower() for k in axis.keys()]))
        heading = pose_euler[axis[up_axis]]
    
        # TODO: handle this response
        self.spot.trajectory_cmd(
            goal_x=position[0], goal_y=position[1], 
            goal_heading=heading,
            cmd_duration=5, reference_frame=relative_frame,
            blocking=blocking,
        )

        # NOTE: UNTESTED # # # # # # # # # # # # # # # # # # # # # # # # # 
        if False:
            roll_axis = axis[dir_axis]
            for k in axis.keys():
                if k not in [dir_axis, up_axis]: pitch_axis = axis[str(k)]
            
            self.spot.stand(body_height=body_height, 
                            body_roll=pose_euler[roll_axis],
                            body_pitch=pose_euler[pitch_axis])
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    def _to_position_and_quaternion(self, pose:np.array):
        '''
        Returns position and quaternion from a pose matrix.
            quaternion is in the form (w, x, y, z)
        '''
        pose = self._to_se3(pose)
        p = pose[0:3, 3]
        q = R.from_matrix(pose[0:3, 0:3]).as_quat()[[3,0,1,2]]
        return p, q

    def grasp(self, pose:np.array, reference_frame:str, **kwargs):
        self._string_feedback = ''
        # try:
        self._string_feedback = f'Grasping pose {pose} in frame {reference_frame}'

        self._string_feedback = 'Approaching desired robot pose.'
        self.go_to(pose, reference_frame, distance=1.2, **kwargs)

        self._string_feedback = 'Preparing arm.'
        # self.spot.arm_unstow()
        # self.spot.gripper_open()

        pre_grasp = self._offset_pose(pose, 0.25, 'x')
        p, q = self._to_position_and_quaternion(pre_grasp)
        self.spot.hand_pose(p, q)

        self.spot.gripper_open()

        self._string_feedback = 'Approaching object.'
        grasp_pose = self._offset_pose(pose, 0.0, 'x')
        p, q = self._to_position_and_quaternion(grasp_pose)
        self.spot.hand_pose(p, q)

        self._string_feedback = 'Succeeded'
        status = True
        # except Exception as e:
        #     self._string_feedback = f'Failed to grasp object: {e}'
        #     status = False
            
        self.spot.arm_stow()
        self.spot.gripper_close()
        return status
