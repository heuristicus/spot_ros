#!/usr/bin/env python3
#Testing script for recording functions
import logging
import time
import os
import sys
# Get the absolute path of the current file
current_dir_of_file = os.path.dirname(os.path.abspath("recording_testing.py"))
# Add the parent directory of spot_driver to the Python path
parent_dir = os.path.join(current_dir_of_file, '..')
sys.path.append(parent_dir)


from spot_driver.spot_wrapper import SpotWrapper
#from spot_driver.spot_task_wrapper import SpotTaskWrapper

import numpy as np
class RecordingTester:
    def __init__(self, uploadPath, power_off=False):
        '''
        NOTE: This script test the WorldObjectHandler class without requiring 
        any ros specific code. The wrapper could be used with other pure python 
        code.
        ''' 
        self.power_off = power_off
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.log = logging.getLogger("rosout")
        self.log.debug('Starting code.')

        self.spot = SpotWrapper('admin', 
                                'pvwmr4j08osj', 
                                '192.168.80.3',  #'192.168.80.3','10.0.0.3', 
                                logger=self.log,
                                estop_timeout=9.0,)

        self.log.setLevel('DEBUG')
        
        self.log.debug('Powering on...')
        self.spot.getLease(hijack=True)
        self.spot.power_on()





    def __upload_recording_test__(self, uploadPath): #Test Script for testing an upload of a pre-recorded graph
        self.log.debug('Attempting the upload Test...')

        self.log.debug('Standing...')
        self.spot.ensure_arm_power_and_stand()

        self.log.debug('Attempting to clear any pre-uploaded maps...')
        self.spot._clear_graph()

        self.log.debug('Attempting to upload a pre-recorded map')
        self.spot._upload_graph_and_snapshots(uploadPath)

        self.log.debug('Attempting to list out the waypoint IDs')
        self.spot._get_localization_state()
        self.spot._list_graph_waypoint_and_edge_ids()

        self.log.debug('Attempting to localize to waypoint number 1')
        self.spot._set_initial_localization_waypoint(["cb"])

        self.log.debug("Navigating waypoints...")

        self.spot._navigate_to(["cb"])
        time.sleep(4)
        self.spot._navigate_route(["sd","db"])
        time.sleep(2)
        

    def __obtain_recording_test__(self): #Test script for recording a path and downloading the recording to local machine
        self.log.debug('Standing...')
        self.spot.ensure_arm_power_and_stand()

        self.log.debug('Attemptiong to clear maps...')
        self.spot._clear_graph()

        self.log.debug('Getting status of the recording...')
        self.spot.get_recording_status()

        self.log.debug('Attempting to start recording...')
        self.spot.record()

        self.log.debug('Getting recording status')
        self.spot.get_recording_status()

        self.log.debug('Walking forward ...')
        self.spot.trajectory_cmd(1, 0, 90, 20)

        self.log.debug('Attempting to stop recording...')
        self.spot.stop_recording()

        self.log.debug('Getting recording status')
        self.spot.get_recording_status()

        self.log.debug('Attempting to download the recording')
        self.spot.download_recording()

    #########################################################################################
    ###### Ignore these #####################################################################
    def __del__(self):
        time.sleep(5)
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')

    def test_grasp(self):
        self.log.debug('Testing Grasp.')
        pose = np.eye(4); pose[0:3, 3] = [1.0, 0, 0]
        self.task.grasp(pose, 'body')

    def test_go_to(self):
        pose = np.array([[ 0.0,  1.0,  0.0,  0.0],
                        [-1.0,  0.0,  0.0,  0.0],
                        [ 0.0,  0.0,  1.0,  0.0],
                        [ 0.0,  0.0,  0.0,  1.0]])
        self.task.go_to(pose, 'body', distance=0.0, dir_axis='x', up_axis='z')
        # pose = np.eye(4); pose[0:3, 3] = [0.2, 0, 0]

        self.log.debug('Turn 90 degrees counter-clockwise.')
        pose = np.array([[ 0.0, -1.0,  0.0],
                        [ 1.0,  0.0,  0.0],
                        [ 0.0,  0.0,  1.0]])
        self.task.go_to(pose, 'body', distance=0.0, dir_axis='x', up_axis='z')
        
        self.log.debug('Move forward 20cm.')
        self.task.go_to(np.eye(4), 'body', distance=-0.2, dir_axis='x', up_axis='z')

        self.log.debug('Move back 20cm.')
        self.task.go_to(np.eye(4), 'body', distance=0.2, dir_axis='x', up_axis='z')


if __name__=='__main__': # Here's where stuff is running
    working_path = os.path.abspath("downloaded_graph")
    print(working_path)
    Testrun = RecordingTester(working_path)
    Testrun.__upload_recording_test__(working_path)