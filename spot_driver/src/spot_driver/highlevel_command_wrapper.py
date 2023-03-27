# Spot wrapper
from spot_wrapper import SpotWrapper

# Clients
from bosdyn.client.manipulation_api_client import ManipulationApiClient

# Messages
from google.protobuf.wrappers_pb2 import FloatValue
from bosdyn.api import manipulation_api_pb2
from bosdyn.api.manipulation_api_pb2 import (WalkToObjectRayInWorld, 
                                             ManipulationApiRequest, 
                                             ManipulationApiFeedbackRequest,
                                             )


# others
import numpy as np
import time
from enum import Enum


class HighlevelCommandSpotWrapper:

    def __init__(self, spot: SpotWrapper):
        self._spot = spot
        self._robot = spot._robot


        assert self._robot.has_arm(), "Robot requires an arm to run this example."

        self._manip_cli = self._robot.ensure_client(ManipulationApiClient.default_service_name)

        self._manip_state = Enum('ManipulationState', ['SUCCEEDED',  'FAILED', 'RUNNING', 'UNKNOWN'])

    def grasp(self, pose, constraints):
        pass


    def walk_to_object(self, pose:np.array, relative_frame:str, distance:float=0.5, axis:str='z'):
        '''Walk to the pose and face the z axis of the pose.'''
        
        if   axis == 'z': offset = np.array([0, 0, distance])
        elif axis == 'y': offset = np.array([0, distance, 0])
        elif axis == 'x': offset = np.array([distance, 0, 0])

        offset = np.concatenate((offset, np.array([1])))

        start = pose @ offset 
        end = pose[:3, 3]

        walkto = WalkToObjectRayInWorld(
            ray_start_rt_frame=start,
            ray_end_rt_frameend=end,
            frame_name=relative_frame,
            offset_distance=FloatValue(0.0)
        )

        # Ask the robot to pick up the object
        walk_to_request = ManipulationApiRequest(
            walk_to_object_in_image=walkto
        )

        # Send the request
        cmd_response = self._manip_cli.manipulation_api_command(
            manipulation_api_request=walk_to_request
        )

        # Get feedback from the robot
        while True:
            time.sleep(0.25)
            state = self._get_manipulation_feedback(cmd_response.manipulation_cmd_id)
            if  state == self._manip_state.SUCCEEDED: break
            elif state == self._manip_state.FAILED: raise Exception('Manipulation failed')


    def _get_manipulation_feedback(self, command_id):
        '''Dumbs down the feedback response to 3 states: Success, Failure, and Inprogress.'''
        feedback_request = ManipulationApiFeedbackRequest(
            manipulation_cmd_id=command_id)

        # Send the request
        response = self._manip_cli.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request)
                
        # TODO: log instead?
        # print('Current state: ', manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

        if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
            return self._manip_state.SUCCEEDED
        elif response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED or\
             response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP or\
             response.current_state == manipulation_api_pb2.MANIP_STATE_PLACE_FAILED_TO_RAYCAST_INTO_MAP or\
             response.current_state == manipulation_api_pb2.MANIP_STATE_PLACE_FAILED:
            return self._manip_state.FAILED
        else: 
            return self._manip_state.RUNNING

