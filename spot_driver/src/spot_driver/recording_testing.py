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
    def __init__(self, power_off=False):
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

        self.log.debug('Standing...')
        self.spot.ensure_arm_power_and_stand()

        
        

    def __del__(self):
        # time.sleep(5)
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


if __name__=='__main__':
    RecordingTester()