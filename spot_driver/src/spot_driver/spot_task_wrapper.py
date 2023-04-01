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

from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, 
                                         ODOM_FRAME_NAME, 
                                         VISION_FRAME_NAME,
                                         )

from bosdyn.client.math_helpers import SE2Pose as bdSE2Pose
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose


# others
import numpy as np
import logging
import time

class SpotTaskWrapper:

    def _init_logger(self, logger:logging.Logger=None):
        class LogHandler(logging.Handler):
            def emit(self, record): self._string_feedback = record
        self._log_handler = LogHandler()
        if logger is None: logger = logging.getLogger(__name__)
        logger.addHandler(self._log_handler)
        self._log = logger

    def __init__(self, spot: SpotWrapper, logger:logging.Logger=None):
        self.spot = spot
        self._robot = spot._robot
        self._init_logger(logger)
        assert self._robot.has_arm(), "Robot requires an arm to run this example."
        self._manip_cli = self._robot.ensure_client(ManipulationApiClient.default_service_name)

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
        '''
        TODO: Document
        '''
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

        # NOTE: UNTESTED # # # # # # # # # # # # # # # # # # # # # # # # # 
        if False:
            roll_axis = axis[dir_axis]
            for k in axis.keys():
                if k not in [dir_axis, up_axis]: pitch_axis = axis[str(k)]
            
            self.spot.stand(body_height=body_height, 
                            body_roll=euler[roll_axis],
                            body_pitch=euler[pitch_axis])
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    def grasp(self, pose, reference_frame:str, **kwargs):
        self._string_feedback = ''
        # try:
        self._log.info(f'Grasping pose {pose} in frame {reference_frame}')
        
        if isinstance(pose, np.ndarray):
            pose = self._pose_np_to_bd(pose, se3=True)

        TARGET_FRAME = VISION_FRAME_NAME
        pose = self.spot._transform_bd_pose(pose, reference_frame, TARGET_FRAME)

        self._log.info('Approaching desired robot pose.')
        self.go_to(pose, TARGET_FRAME, distance=1.0, **kwargs)

        self._string_feedback = 'Preparing arm.'
        print(self._string_feedback)

        pre_grasp = self._offset_pose(pose, 0.25, 'x')
        
        pos, rot = self._pose_bd_to_vectors(pre_grasp)
        status, msg = self.spot.hand_pose(pos, rot, 
                                          reference_frame=TARGET_FRAME, 
                                          duration=2.0)
        self._log.info(f'Pre-grasp status: {msg}')
        if status is False: raise(Exception('Failed to reach pre-grasp.'))

        self.spot.gripper_open()

        self._log.info('Approaching object...')
        pos, rot = self._pose_bd_to_vectors(pose)
        status, msg = self.spot.hand_pose(pos, rot, 
                                          reference_frame=TARGET_FRAME, 
                                          duration=1.0)
        self._log.info(f'Approach status: {msg}')
        if status is False: raise(Exception('Failed to reach pre-grasp.'))
        time.sleep(5)

        self._log.info('Succeeded')
        # except Exception as e:
        #     self._string_feedback = f'Failed to grasp object: {e}'
        #     status = False
            
        self.spot.arm_stow()
        self.spot.gripper_close()
