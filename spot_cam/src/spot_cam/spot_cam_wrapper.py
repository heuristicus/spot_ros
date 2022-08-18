import enum
import typing

import bosdyn.client
from bosdyn.client import spot_cam
from bosdyn.client.spot_cam.lighting import LightingClient

from spot_driver.spot_wrapper import SpotWrapper


class SpotCamLightingHandler:
    """
    Handler for interacting with LED brightnesses
    """
    class LEDPosition(enum.Enum):
        """
        Values indicate the position of the specified LED in the brightness list
        """

        REAR_LEFT = 0
        FRONT_LEFT = 1
        FRONT_RIGHT = 2
        REAR_RIGHT = 3

    def __init__(self, robot):
        self.client = robot.ensure_client(LightingClient.default_service_name)

    def set_led_brightness(self, brightness):
        """
        Set the brightness of the LEDs to the specified brightness

        Args:
            brightness: LEDs will all be set to this brightness, which should be in the range [0, 1]. The value will
                        be clipped if outside this range.
        """
        # Clamp brightness to [0,1] range
        brightness = min(max(brightness, 0), 1)
        self.client.set_led_brightness([brightness] * 4)

    def get_led_brightness(self) -> typing.List[float]:
        """
        Get the brightness of the LEDS

        Returns:
            List of floats indicating current brightness of each LED, in the order they are specified in the
            LEDPosition enum
        """
        return self.client.get_led_brightness()


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
        SpotWrapper.authenticate(
            robot, self._hostname, self._username, self._password, self._logger
        )

        self.lighting = SpotCamLightingHandler(robot)
