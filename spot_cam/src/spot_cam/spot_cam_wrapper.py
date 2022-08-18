import logging
import bosdyn.client
from bosdyn.client import spot_cam
from spot_driver.spot_wrapper import SpotWrapper


class SpotCamWrapper:
    def __init__(self, hostname, username, password, logger):
        self._hostname = hostname
        self._username = username
        self._password = password
        self._logger = logger

        # Create robot object and authenticate.
        sdk = bosdyn.client.create_standard_sdk("Spot CAM Client")
        spot_cam.register_all_service_clients(sdk)

        robot = sdk.create_robot(self._hostname)
        SpotWrapper.authenticate(robot, self._hostname, self._username, self._password, self._logger)
