import typing
import logging

from bosdyn.client.robot import Robot
from bosdyn.client import robot_command
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.api.docking import docking_pb2


class SpotDocking:
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_params: typing.Dict[str, typing.Any],
        robot_clients: typing.Dict[str, typing.Any],
    ):
        self._robot = robot
        self._logger = logger
        self._docking_client: DockingClient = robot_clients["docking_client"]
        self._robot_command_client: robot_command.RobotCommandClient = robot_clients[
            "robot_command_client"
        ]
        self._robot_params = robot_params

    def dock(self, dock_id: int) -> typing.Tuple[bool, str]:
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            if not self._robot_params["is_standing"]:
                robot_command.blocking_stand(
                    command_client=self._robot_command_client, timeout_sec=10
                )
                self._logger.info("Spot is standing")
            else:
                self._logger.info("Spot is already standing")
            # Dock the robot
            self.last_docking_command = dock_id
            blocking_dock_robot(self._robot, dock_id)
            self.last_docking_command = None
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def undock(self, timeout: int = 20) -> typing.Tuple[bool, str]:
        """Power motors on and undock the robot from the station."""
        try:
            # Maker sure we're powered on
            self._robot.power_on()
            # Undock the robot
            blocking_undock(self._robot, timeout)
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def get_docking_state(self, **kwargs) -> docking_pb2.DockState:
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state  # type: ignore
