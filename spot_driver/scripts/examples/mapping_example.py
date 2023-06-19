import logging
import time 
import sys
import os
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/mapping_example.py
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
class MappingWrapperTester:
    def __init__(self, download_path, power_off=False):
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
        
        # example of recording a graph
        self.log.debug('Attemptiong to clear maps...')
        self.spot._clear_graph()

        self.log.debug('Getting status of the recording...')
        self.spot.get_recording_status()

        self.log.debug('Attempting to start recording...')
        self.spot.record()

        self.log.debug('Getting recording status')
        self.spot.get_recording_status()

        self.log.debug('Walking forward ...')
        self.spot.trajectory_cmd(1, 0, 0, 2)

        time.sleep(2)
        self.spot.trajectory_cmd(0, 0, 0.6, 3)
        time.sleep(3)
        self.spot.trajectory_cmd(0.5, 0, 0, 3)
        time.sleep(3)
        self.spot.trajectory_cmd(0, 0, -1.2, 3)
        time.sleep(3)
        self.spot.trajectory_cmd(0.5, 0, 0, 3)
        time.sleep(3)

        self.log.debug('Attempting to stop recording...')
        self.spot.stop_recording()

        self.log.debug('Getting recording status')
        self.spot.get_recording_status()

        self.log.debug('Attempting to download the recording')
        self.spot.download_recording(download_path)
       
        # example of downloading and navigating a graph
        self.spot._clear_graph()
        self.log.debug("Uploading graph...")
        self.spot._upload_graph_and_snapshots(download_path + "/downloaded_graph")

        self.spot._get_localization_state()

        self.log.debug("localizing ...")
        # waypoint arguments must be passed in as a list.
        # passing in strings directly will lead to waypoints not being found
        waypoints = self.spot.list_graph()
        self.spot._set_initial_localization_waypoint([waypoints[0]])
        self.log.debug("Navigating waypoints...")

        # when using navigate_to, must specify path to map
        # destination waypoint, and localization method.
        # if using waypoints, must specify localization waypoint
        print(waypoints[-1])
        print(waypoints[0])
        self.spot.navigate_to(download_path + "/downloaded_graph", waypoints[0], False, waypoints[-1])
        
        time.sleep(2)

        # _navigate_to() method is used to navigate to a waypoint from a map thatis already uploaded to spot
        # self.spot._navigate_to([waypoints[1]])

        # _navigate_route() method is used to navigate through a list of specified waypoints.
        # there must be edges between each adjacent pair of waypoints in the list
        # self.spot._navigate_route(waypoints)
        time.sleep(2)
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')
if __name__ == "__main__":
    # specify the path to download and upload the graph here
    download_path = os.getcwd() + "/scripts/examples"
    MappingWrapperTester(download_path)
