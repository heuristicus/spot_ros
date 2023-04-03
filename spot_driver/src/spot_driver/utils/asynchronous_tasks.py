import time

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client import ResponseError, RpcError
from bosdyn.api import basic_command_pb2


class AsyncMetrics(AsyncPeriodicQuery):
    """Class to get robot metrics at regular intervals.  get_robot_metrics_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncMetrics, self).__init__(
            "robot-metrics", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_metrics_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncLease(AsyncPeriodicQuery):
    """Class to get lease state at regular intervals.  list_leases_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncLease, self).__init__(
            "lease", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.list_leases_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncImageService(AsyncPeriodicQuery):
    """Class to get images at regular intervals.  get_image_from_sources_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, image_requests):
        super(AsyncImageService, self).__init__(
            "robot_image_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_image_async(self._image_requests)
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncIdle(AsyncPeriodicQuery):
    """Class to check if the robot is moving, and if not, command a stand with the set mobility parameters

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        spot_wrapper: A handle to the wrapper library
    """

    def __init__(self, client, logger, rate, spot_wrapper):
        super(AsyncIdle, self).__init__("idle", client, logger, period_sec=1.0 / rate)

        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if self._spot_wrapper._last_stand_command != None:
            try:
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_stand_command
                )
                status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status
                )
                self._spot_wrapper._is_sitting = False
                if status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
                    self._spot_wrapper._is_standing = True
                    self._spot_wrapper._last_stand_command = None
                elif (
                    status == basic_command_pb2.StandCommand.Feedback.STATUS_IN_PROGRESS
                ):
                    self._spot_wrapper._is_standing = False
                else:
                    self._logger.warn(f"Stand command in unknown state {status}")
                    self._spot_wrapper._is_standing = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_stand_command = None

        if self._spot_wrapper._last_sit_command != None:
            try:
                self._spot_wrapper._is_standing = False
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_sit_command
                )
                if (
                    response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status
                    == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING
                ):
                    self._spot_wrapper._is_sitting = True
                    self._spot_wrapper._last_sit_command = None
                else:
                    self._spot_wrapper._is_sitting = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_sit_command = None

        is_moving = False

        if self._spot_wrapper._last_velocity_command_time != None:
            if time.time() < self._spot_wrapper._last_velocity_command_time:
                is_moving = True
            else:
                self._spot_wrapper._last_velocity_command_time = None

        if self._spot_wrapper._last_trajectory_command != None:
            try:
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_trajectory_command
                )
                status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
                )
                # STATUS_AT_GOAL always means that the robot reached the goal. If the trajectory command did not
                # request precise positioning, then STATUS_NEAR_GOAL also counts as reaching the goal
                if (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL
                    or (
                        status
                        == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL
                        and not self._spot_wrapper._last_trajectory_command_precise
                    )
                ):
                    self._spot_wrapper._at_goal = True
                    # Clear the command once at the goal
                    self._spot_wrapper._last_trajectory_command = None
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_GOING_TO_GOAL
                ):
                    is_moving = True
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL
                ):
                    is_moving = True
                    self._spot_wrapper._near_goal = True
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_UNKNOWN
                ):
                    self._spot_wrapper._trajectory_status_unknown = True
                    self._spot_wrapper._last_trajectory_command = None
                else:
                    self._logger.error(
                        "Received trajectory command status outside of expected range, value is {}".format(
                            status
                        )
                    )
                    self._spot_wrapper._last_trajectory_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_trajectory_command = None

        self._spot_wrapper._is_moving = is_moving

        if (
            self._spot_wrapper.is_standing
            and not self._spot_wrapper.is_moving
            and self._spot_wrapper._last_trajectory_command is not None
            and self._spot_wrapper._last_stand_command is not None
            and self._spot_wrapper._last_velocity_command_time is not None
            and self._spot_wrapper._last_docking_command is not None
        ):
            self._spot_wrapper.stand(False)


class AsyncEStopMonitor(AsyncPeriodicQuery):
    """Class to check if the estop endpoint is still valid

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        spot_wrapper: A handle to the wrapper library
    """

    def __init__(self, client, logger, rate, spot_wrapper):
        super(AsyncEStopMonitor, self).__init__(
            "estop_alive", client, logger, period_sec=1.0 / rate
        )
        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if not self._spot_wrapper._estop_keepalive:
            self._logger.debug("No keepalive yet - the lease has not been claimed.")
            return

        last_estop_status = self._spot_wrapper._estop_keepalive.status_queue.queue[-1]
        if (
            last_estop_status[0]
            == self._spot_wrapper._estop_keepalive.KeepAliveStatus.ERROR
        ):
            self._logger.error(
                "Estop keepalive has an error: {}".format(last_estop_status[1])
            )
        elif (
            last_estop_status
            == self._spot_wrapper._estop_keepalive.KeepAliveStatus.DISABLED
        ):
            self._logger.error(
                "Estop keepalive is disabled: {}".format(last_estop_status[1])
            )
        else:
            # estop keepalive is ok
            pass
