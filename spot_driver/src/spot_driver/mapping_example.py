import logging
import time 
import sys
sys.path.append("/mnt/c/Users/rhett/UMN_Github/repo-olso9295-5/SpotRobotProject/spot_ros/spot_driver/src")
#print(sys.path)
from spot_driver.spot_wrapper import SpotWrapper
class MappingWrapperTester:
    def __init__(self, upload_path, power_off=False):
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

        #self.log.debug('Standing...')
        #self.spot.ensure_arm_power_and_stand()
        #self.log.debug("Uploading graph...")
        #self.spot._upload_graph_and_snapshots(upload_path)

        #self.spot._get_localization_state()

        #self.spot._list_graph_waypoint_and_edge_ids()

        #self.log.debug("localizing ...")
        #self.spot._set_initial_localization_waypoint("ef ")
        self.log.debug("Navigating waypoints...")
        # when using navigate_to, must specify path to map
        # destination waypoint, and localization method.
        # if using waypoints, must specify localization waypoint
        self.spot.navigate_to(upload_path, "ss", False, "ef")
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')
if __name__ == "__main__":
    MappingWrapperTester("/mnt/c/Users/rhett/UMN_Github/repo-olso9295-5/SpotRobotProject/spot-sdk/python/examples/graph_nav_command_line/downloaded_graph")
