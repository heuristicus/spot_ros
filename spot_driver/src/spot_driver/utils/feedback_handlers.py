import time
from enum import Enum

BASIC_STATES = Enum('ManipulationState', ['SUCCEEDED',  'FAILED', 'RUNNING', 'UNKNOWN'])



# from bosdyn.api import synchronized_command_pb2
from bosdyn.api import robot_command_pb2 
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus

def handle_se2_traj_cmd_feedback(command_id, client, sleep=0.25, log_fx=None, timeout=10):
    timeout = time.time() + timeout
    while time.time() < timeout:

        feedback = client.robot_command_feedback(command_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            if log_fx: log_fx("Failed to reach the goal")
            return False
        
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            if log_fx: log_fx("Arrived at the goal.")
            return True
        
        time.sleep(sleep)
        
    if log_fx: log_fx("Feedback handler timed out.")
    return False




from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import manipulation_api_pb2

from bosdyn.api.manipulation_api_pb2 import (WalkToObjectRayInWorld, 
                                             ManipulationApiRequest, 
                                             ManipulationApiFeedbackRequest,
                                             )

def handle_manipulation_feedback(
        command_id, 
        client:ManipulationApiClient, 
        response=None, 
        sleep=0.25
        )->BASIC_STATES:
    feedback = BASIC_STATES.RUNNING
    while feedback == BASIC_STATES.RUNNING:
        time.sleep(sleep)
        feedback = get_manipulation_feedback(command_id, client, response) 
    return feedback

def get_manipulation_feedback(command_id, client:ManipulationApiClient, response=None)->BASIC_STATES:

    '''Dumbs down the feedback response to 3 states: Success, Failure, and Inprogress.'''
    feedback_request = ManipulationApiFeedbackRequest(
        manipulation_cmd_id=command_id)

    # Send the request
    response = client.manipulation_api_feedback_command(
        manipulation_api_feedback_request=feedback_request)
            
    # TODO: log instead?
    # print('Current state: ', manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

    if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
        return BASIC_STATES.SUCCEEDED
    elif response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED or\
            response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP or\
            response.current_state == manipulation_api_pb2.MANIP_STATE_PLACE_FAILED_TO_RAYCAST_INTO_MAP or\
            response.current_state == manipulation_api_pb2.MANIP_STATE_PLACE_FAILED:
        return BASIC_STATES.FAILED
    else: 
        return BASIC_STATES.RUNNING

