# Separate code from spot_wrapper into a SpotEstopLease class

# Import required modules
import typing
import logging

from bosdyn.client.robot import Robot
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive, EstopClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive


class SpotEstopLease:
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_params: typing.Dict[str, typing.Any],
        robot_clients: typing.Dict[str, typing.Any],
    ):
        """
        Initialize the SpotEstopLease class
        """
        self._robot = robot
        self._logger = logger
        self._estop_client: EstopClient = robot_clients["estop_client"]
        self._lease_client: LeaseClient = robot_clients["lease_client"]
        self._estop_timeout: float = robot_params["estop_timeout"]
        self._robot_params = robot_params
        self._estop_keepalive = None

    def claim(self) -> typing.Tuple[bool, str]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self._robot_params["robot_id"] = self._robot.get_id()
            self.getLease()
            self.resetEStop()
            return True, "Success, lease and eStop claimed"
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False, str(err)

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(
            self._estop_client, "ros", self._estop_timeout
        )
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe: bool = True) -> typing.Tuple[bool, str]:
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_keepalive.stop()  # type: ignore
            else:
                self._estop_keepalive.settle_then_cut()  # type: ignore

            return True, "Success, E-Stop engaged"
        except:
            return False, "Error, E-Stop already engaged or released"

    def disengageEStop(self) -> typing.Tuple[bool, str]:
        """Disengages the E-Stop"""
        try:
            self._estop_keepalive.allow()  # type: ignore
            return True, "Success, E-Stop disengaged"
        except:
            return False, "Error, E-Stop already released"

    def releaseEStop(self):
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self):
        """Get a lease for the robot and keep the lease alive automatically."""
        self._lease = self._lease_client.acquire()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def releaseLease(self):
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self) -> typing.Tuple[bool, str]:
        """Return the lease on the body and the eStop handle."""
        try:
            self.releaseLease()
            self.releaseEStop()
            return True, "Success, lease and eStop released"
        except Exception as e:
            return False, str(e)
