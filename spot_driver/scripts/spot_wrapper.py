import time

from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.power import PowerClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive

import bosdyn.api.robot_state_pb2 as robot_state_proto

front_image_sources = ['frontleft_fisheye_image', 'frontright_fisheye_image', 'frontleft_depth', 'frontright_depth']
side_image_sources = ['left_fisheye_image', 'right_fisheye_image', 'left_depth', 'right_depth']
rear_image_sources = ['back_fisheye_image', 'back_depth']

class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__("robot-state", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
        callback_future = self._client.get_robot_state_async()
        callback_future.add_done_callback(self._callback)
        return callback_future

class AsyncMetrics(AsyncPeriodicQuery):
    """Class to get robot metrics at regular intervals.  get_robot_metrics_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncMetrics, self).__init__("robot-metrics", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
        callback_future = self._client.get_robot_metrics_async()
        callback_future.add_done_callback(self._callback)
        return callback_future

class AsyncRobotCommand(AsyncPeriodicQuery):
    """Class to get robot command status at regular intervals.  robot_command_feedback_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotCommand, self).__init__("robot-command", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
        callback_future = self._client.robot_command_feedback_async()
        callback_future.add_done_callback(self._callback)
        return callback_future

class AsyncPower(AsyncPeriodicQuery):
    """Class to get power status at regular intervals.  power_command_feedback_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncPower, self).__init__("power", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
        callback_future = self._client.power_command_feedback_async()
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
        super(AsyncLease, self).__init__("lease", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
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
        super(AsyncImageService, self).__init__("robot_image_service", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        callback_future = self._client.get_image_async(self._image_requests)
        callback_future.add_done_callback(self._callback)
        return callback_future

class AsyncEStop(AsyncPeriodicQuery):
    """Class to get estop state at regular intervals.  get_status_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncEStop, self).__init__("estop", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
        callback_future = self._client.get_status_async()
        callback_future.add_done_callback(self._callback)
        return callback_future

class SpotWrapper():
    def __init__(self, username, password, token, hostname, logger, rates = {}, callbacks = {}):
        self._username = username
        self._password = password
        self._token = token
        self._hostname = hostname
        self._logger = logger
        self._rates = rates
        self._callbacks = callbacks
        self._keep_alive = True

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(build_image_request(source, image_format=image_pb2.Image.Format.FORMAT_RAW))

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(build_image_request(source, image_format=image_pb2.Image.Format.FORMAT_RAW))

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(build_image_request(source, image_format=image_pb2.Image.Format.FORMAT_RAW))

        self._sdk = create_standard_sdk('ros_spot')
        self._sdk.load_app_token(self._token)
        self._robot = self._sdk.create_robot(self._hostname)

        try:
            self._robot.authenticate(self._username, self._password)
            self._robot.start_time_sync()
        except RpcError as err:
            self._logger.error("Failed to communicate with robot: %s", err)
            self._robot = None

        if self._robot:
            # Clients
            self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
            #self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
            #self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
            self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
            self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)

            # Async Tasks
            self._robot_state_task = AsyncRobotState(self._robot_state_client, self._logger, self._rates.get("robot_state", 1.0), self._callbacks.get("robot_state", lambda:None))
            self._robot_metrics_task = AsyncMetrics(self._robot_state_client, self._logger, self._rates.get("metrics", 1.0), self._callbacks.get("metrics", lambda:None))
            #self._robot_command_task = AsyncRobotCommand(self._robot_command_client, self._logger, self._rates.get("robot_command", 1.0), self._callbacks.get("robot_command", lambda:None))
            #self._power_task = AsyncPower(self._power_client, self._logger, self._rates.get("power", 1.0), self._callbacks.get("power", lambda:None))
            self._lease_task = AsyncLease(self._lease_client, self._logger, self._rates.get("lease", 1.0), self._callbacks.get("lease", lambda:None))
            self._front_image_task = AsyncImageService(self._image_client, self._logger, self._rates.get("front_image", 1.0), self._callbacks.get("front_image", lambda:None), self._front_image_requests)
            self._side_image_task = AsyncImageService(self._image_client, self._logger, self._rates.get("side_image", 1.0), self._callbacks.get("side_image", lambda:None), self._side_image_requests)
            self._rear_image_task = AsyncImageService(self._image_client, self._logger, self._rates.get("rear_image", 1.0), self._callbacks.get("rear_image", lambda:None), self._rear_image_requests)
            self._estop_task = AsyncEStop(self._estop_client, self._logger, self._rates.get("estop", 1.0), self._callbacks.get("estop", lambda:None))

            self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', 9.0)

            self._async_tasks = AsyncTasks(
                [self._robot_state_task, self._robot_metrics_task, self._lease_task, self._front_image_task, self._side_image_task, self._rear_image_task, self._estop_task])

            self._robot_id = None
            self._lease = None

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def metrics(self):
        """Return latest proto from the _robot_metrics_task"""
        return self._robot_metrics_task.proto

    @property
    def robot_command(self):
        """Return latest proto from the _robot_command_task"""
        return self._robot_command_task.proto

    @property
    def power(self):
        """Return latest proto from the _power_task"""
        return self._power_task.proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def front_images(self):
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self):
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self):
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto

    @property
    def estop(self):
        """Return latest proto from the _estop_task"""
        return self._estop_task.proto

    def connect(self):
        try:
            self._robot_id = self._robot.get_id()
            self._lease = self._lease_client.acquire()
            self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False

    def updateTasks(self):
        self._async_tasks.update()

    def getLease(self):
        return LeaseKeepAlive(self._lease_client)

    def disconnect(self):
        """Release control of robot as gracefully as posssible."""
        self._logger.info("Shutting down ROS interface")
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None
