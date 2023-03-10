import time
import logging
import typing
import logging
import typing

from bosdyn.geometry import EulerZXY
from bosdyn.client.auth import InvalidLoginError
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.power import PowerClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.docking import DockingClient
from bosdyn.client.time_sync import TimeSyncEndpoint
from bosdyn.client.estop import EstopClient
from bosdyn.client.spot_check import SpotCheckClient
from bosdyn.client.docking import DockingClient
from bosdyn.client.estop import EstopClient
from bosdyn.client import power
from bosdyn.client import frame_helpers
from bosdyn.client import math_helpers
from bosdyn.client.point_cloud import PointCloudClient, build_pc_request
from bosdyn.api import image_pb2


from .spot_arm import SpotArm
from .spot_estop_lease import SpotEstopLease
from .spot_docking import SpotDocking
from .spot_graph_nav import SpotGraphNav
from .spot_check import SpotCheck

from bosdyn.api import robot_command_pb2
from bosdyn.api import robot_id_pb2, point_cloud_pb2
from bosdyn.api import image_pb2, robot_state_pb2, lease_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import basic_command_pb2
from bosdyn.api import robot_command_pb2
from google.protobuf.timestamp_pb2 import Timestamp

front_image_sources = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "frontleft_depth",
    "frontright_depth",
]
"""List of image sources for front image periodic query"""
side_image_sources = [
    "left_fisheye_image",
    "right_fisheye_image",
    "left_depth",
    "right_depth",
]
"""List of image sources for side image periodic query"""
rear_image_sources = ["back_fisheye_image", "back_depth"]
"""List of image sources for rear image periodic query"""
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"
"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
point_cloud_sources = ["velodyne-point-cloud"]
"""List of point cloud sources"""
hand_image_sources = [
    "hand_image",
    "hand_depth",
    "hand_color_image",
    "hand_depth_in_hand_color_frame",
]
"""List of image sources for hand image periodic query"""


class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(
        self,
        client: RobotStateClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Callable,
    ):
        super(AsyncRobotState, self).__init__(
            "robot-state", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
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

    def __init__(
        self,
        client: RobotStateClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Callable,
    ):
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

    def __init__(
        self,
        client: LeaseClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Callable,
    ):
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

    def __init__(
        self,
        client: ImageClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Callable,
        image_requests: typing.List[image_pb2.ImageRequest],
    ):
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


class AsyncPointCloudService(AsyncPeriodicQuery):
    """
    Class to get point cloud at regular intervals.  get_point_cloud_from_sources_async query sent to the robot at
    every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, point_cloud_requests):
        super(AsyncPointCloudService, self).__init__(
            "robot_point_cloud_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._point_cloud_requests = point_cloud_requests

    def _start_query(self):
        if self._callback and self._point_cloud_requests:
            callback_future = self._client.get_point_cloud_async(
                self._point_cloud_requests
            )
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

    def __init__(
        self,
        client: RobotCommandClient,
        logger: logging.Logger,
        rate: float,
        spot_wrapper: "SpotWrapper",
    ):
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
                self._spot_wrapper._robot_params["is_sitting"] = False
                if status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
                    self._spot_wrapper._robot_params["is_standing"] = True
                    self._spot_wrapper._last_stand_command = None
                elif (
                    status == basic_command_pb2.StandCommand.Feedback.STATUS_IN_PROGRESS
                ):
                    self._spot_wrapper._robot_params["is_standing"] = False
                else:
                    self._logger.warn("Stand command in unknown state")
                    self._spot_wrapper._robot_params["is_standing"] = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_stand_command = None

        if self._spot_wrapper._last_sit_command != None:
            try:
                self._spot_wrapper._robot_params["is_standing"] = False
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_sit_command
                )
                if (
                    response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status
                    == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING
                ):
                    self._spot_wrapper._robot_params["is_sitting"] = True
                    self._spot_wrapper._last_sit_command = None
                else:
                    self._spot_wrapper._robot_params["is_sitting"] = False
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

        self._spot_wrapper._robot_params["is_moving"] = is_moving

        # We must check if any command currently has a non-None value for its id. If we don't do this, this stand
        # command can cause other commands to be interrupted before they get to start
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

    def __init__(
        self,
        client: EstopClient,
        logger: logging.Logger,
        rate: float,
        spot_wrapper: "SpotWrapper",
    ):
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


class SpotWrapper:
    """Generic wrapper class to encompass release 3.2.0 API features as well as maintaining leases automatically

    Attributes:
        username: Username to authenticate with the robot
        password: Password to authenticate with the robot
        hostname: Hostname of the robot
        logger: Logger object
        estop_timeout: Timeout (seconds) to wait for estop to be claimed
        rates: Dictionary of rates (Hz) to trigger the queries
        callbacks: Dictionary of callback functions to be called when a query is triggered

    Note:
        Certain features of the 3.2.0 API are not supported by this wrapper. These include:
            - Autowalk
            - Arm impedance control
            - Fan power control
            - Ground clutter
            - Safely sit on stairs feedback
            - GripperCameraParamService
            - RayCastService
            - Auto return
            - Choreography
            - Constrained manipulation
            - Door opening
    """

    def __init__(
        self,
        username: str,
        password: str,
        hostname: str,
        logger: logging.Logger,
        estop_timeout: float = 9.0,
        rates: typing.Dict[str, float] = {},
        callbacks: typing.Dict[str, typing.Callable] = {},
    ):
        self._username = username
        self._password = password
        self._hostname = hostname
        self._logger = logger
        self._rates = rates
        self._callbacks = callbacks
        self._keep_alive = True
        self._valid = True

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._at_goal = False
        self._near_goal = False
        self._trajectory_status_unknown = False
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_trajectory_command_precise = None
        self._last_velocity_command_time = None
        self._last_docking_command = None
        self._robot_params = {
            "is_standing": False,
            "is_sitting": True,
            "is_moving": False,
            "robot_id": None,
            "estop_timeout": estop_timeout,
        }

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._point_cloud_requests = []
        for source in point_cloud_sources:
            self._point_cloud_requests.append(build_pc_request(source))

        self._hand_image_requests = []
        for source in hand_image_sources:
            self._hand_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        try:
            self._sdk = create_standard_sdk("ros_spot")
        except Exception as e:
            self._logger.error("Error creating SDK object: %s", e)
            self._valid = False
            return

        self._logger.info("Initialising robot at {}".format(self._hostname))
        self._robot = self._sdk.create_robot(self._hostname)

        if not self._robot:
            self._logger.error("Failed to create robot object")
            self._valid = False
            return

        authenticated = False
        while not authenticated:
            try:
                self._logger.info("Trying to authenticate with robot...")
                self._robot.authenticate(self._username, self._password)
                self._robot.start_time_sync()
                self._logger.info("Successfully authenticated.")
                authenticated = True
            except RpcError as err:
                sleep_secs = 15
                self._logger.warn(
                    "Failed to communicate with robot: {}\nEnsure the robot is powered on and you can "
                    "ping {}. Robot may still be booting. Will retry in {} seconds".format(
                        err, self._hostname, sleep_secs
                    )
                )
                time.sleep(sleep_secs)
            except InvalidLoginError as err:
                self._logger.error("Failed to log in to robot: {}".format(err))
                self._valid = False
                return
            try:
                self._point_cloud_client = self._robot.ensure_client(
                    VELODYNE_SERVICE_NAME
                )
            except Exception as e:
                self._point_cloud_client = None
                self._logger.warn("No point cloud services are available.")

        if self._robot:
            # Clients
            self._logger.info("Creating clients...")
            initialised = False
            while not initialised:
                try:
                    self._robot_state_client = self._robot.ensure_client(
                        RobotStateClient.default_service_name
                    )
                    self._robot_command_client = self._robot.ensure_client(
                        RobotCommandClient.default_service_name
                    )
                    self._graph_nav_client = self._robot.ensure_client(
                        GraphNavClient.default_service_name
                    )
                    self._power_client = self._robot.ensure_client(
                        PowerClient.default_service_name
                    )
                    self._lease_client = self._robot.ensure_client(
                        LeaseClient.default_service_name
                    )
                    self._image_client = self._robot.ensure_client(
                        ImageClient.default_service_name
                    )
                    self._estop_client = self._robot.ensure_client(
                        EstopClient.default_service_name
                    )
                    self._docking_client = self._robot.ensure_client(
                        DockingClient.default_service_name
                    )
                    self._spot_check_client = self._robot.ensure_client(
                        SpotCheckClient.default_service_name
                    )
                    initialised = True

                    self._robot_clients = {
                        "robot_state_client": self._robot_state_client,
                        "robot_command_client": self._robot_command_client,
                        "graph_nav_client": self._graph_nav_client,
                        "power_client": self._power_client,
                        "lease_client": self._lease_client,
                        "image_client": self._image_client,
                        "estop_client": self._estop_client,
                        "docking_client": self._docking_client,
                        "spot_check_client": self._spot_check_client,
                        "robot_command_method": self._robot_command,
                    }
                    if self._point_cloud_client:
                        self._robot_clients[
                            "point_cloud_client"
                        ] = self._point_cloud_client
                    self._logger.info(
                        "Successfully created Spot SDK clients in SpotWrapper."
                    )
                except Exception as e:
                    sleep_secs = 15
                    self._logger.warn(
                        "Unable to create client service: {}. This usually means the robot hasn't "
                        "finished booting yet. Will wait {} seconds and try again.".format(
                            e, sleep_secs
                        )
                    )
                    time.sleep(sleep_secs)

            # Async Tasks
            self._async_task_list = []
            self._robot_state_task = AsyncRobotState(
                self._robot_state_client,
                self._logger,
                max(0.0, self._rates.get("robot_state", 0.0)),
                self._callbacks.get("robot_state", lambda: None),
            )
            self._robot_metrics_task = AsyncMetrics(
                self._robot_state_client,
                self._logger,
                max(0.0, self._rates.get("metrics", 0.0)),
                self._callbacks.get("metrics", lambda: None),
            )
            self._lease_task = AsyncLease(
                self._lease_client,
                self._logger,
                max(0.0, self._rates.get("lease", 0.0)),
                self._callbacks.get("lease", lambda: None),
            )
            self._front_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("front_image", 0.0)),
                self._callbacks.get("front_image", lambda: None),
                self._front_image_requests,
            )
            self._side_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("side_image", 0.0)),
                self._callbacks.get("side_image", lambda: None),
                self._side_image_requests,
            )
            self._rear_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("rear_image", 0.0)),
                self._callbacks.get("rear_image", lambda: None),
                self._rear_image_requests,
            )
            self._hand_image_task = AsyncImageService(
                self._image_client,
                self._logger,
                max(0.0, self._rates.get("hand_image", 0.0)),
                self._callbacks.get("hand_image", lambda: None),
                self._hand_image_requests,
            )
            self._idle_task = AsyncIdle(
                self._robot_clients["robot_command_client"], self._logger, 10.0, self
            )
            self._estop_monitor = AsyncEStopMonitor(
                self._estop_client, self._logger, 20.0, self
            )

            self._estop_endpoint = None
            self._estop_keepalive = None

            robot_tasks = [
                self._robot_state_task,
                self._robot_metrics_task,
                self._lease_task,
                self._front_image_task,
                self._idle_task,
                self._estop_monitor,
            ]

            if self._point_cloud_client:
                self._point_cloud_task = AsyncPointCloudService(
                    self._point_cloud_client,
                    self._logger,
                    max(0.0, self._rates.get("point_cloud", 0.0)),
                    self._callbacks.get("lidar_points", lambda: None),
                    self._point_cloud_requests,
                )
                robot_tasks.append(self._point_cloud_task)

            self._async_tasks = AsyncTasks(robot_tasks)

            self.camera_task_name_to_task_mapping = {
                "hand_image": self._hand_image_task,
                "side_image": self._side_image_task,
                "rear_image": self._rear_image_task,
                "front_image": self._front_image_task,
            }

            if self._robot.has_arm():
                self._async_tasks.add_task(self._hand_image_task)
                self._spot_arm = SpotArm(
                    self._robot, self._logger, self._robot_params, self._robot_clients
                )

            self._spot_estop_lease = SpotEstopLease(
                self._robot, self._logger, self._robot_params, self._robot_clients
            )
            self._spot_docking = SpotDocking(
                self._robot, self._logger, self._robot_params, self._robot_clients
            )
            self._spot_graph_nav = SpotGraphNav(
                self._robot, self._logger, self._robot_params, self._robot_clients
            )

            self._spot_check = SpotCheck(
                self._robot, self._logger, self._robot_params, self._robot_clients
            )

            self._lease = None

    @property
    def spot_arm(self) -> SpotArm:
        """Return SpotArm instance"""
        if not self._robot.has_arm():
            raise Exception("SpotArm is not available on this robot")
        else:
            return self._spot_arm

    @property
    def spot_estop_lease(self) -> SpotEstopLease:
        """Return SpotEstopLease instance"""
        return self._spot_estop_lease

    @property
    def spot_docking(self) -> SpotDocking:
        """Return SpotDocking instance"""
        return self._spot_docking

    @property
    def spot_graph_nav(self) -> SpotGraphNav:
        """Return SpotGraphNav instance"""
        return self._spot_graph_nav

    @property
    def spot_check(self) -> SpotCheck:
        """Return SpotCheck instance"""
        return self._spot_check

    @property
    def logger(self) -> logging.Logger:
        """Return logger instance of the SpotWrapper"""
        return self._logger

    @property
    def is_valid(self) -> bool:
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._valid

    @property
    def id(self) -> typing.Optional[robot_id_pb2.RobotId]:
        """Return robot's ID. Note that it may return None when the robot is still initializing"""
        return self._robot_params["robot_id"]

    @property
    def robot_state(self) -> robot_state_pb2.RobotState:
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def metrics(self) -> robot_state_pb2.RobotMetrics:
        """Return latest proto from the _robot_metrics_task"""
        return self._robot_metrics_task.proto

    @property
    def lease(self) -> typing.List[lease_pb2.LeaseResource]:
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def front_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto

    @property
    def hand_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest proto from the _hand_image_task"""
        return self._hand_image_task.proto

    @property
    def point_clouds(self) -> typing.List[point_cloud_pb2.PointCloudResponse]:
        """Return latest proto from the _point_cloud_task"""
        return self._point_cloud_task.proto

    @property
    def is_standing(self) -> bool:
        """Return boolean of standing state"""
        return self._robot_params["is_standing"]

    @property
    def is_sitting(self) -> bool:
        """Return boolean of standing state"""
        return self._robot_params["is_sitting"]

    @property
    def is_moving(self) -> bool:
        """Return boolean of walking state"""
        return self._robot_params["is_moving"]

    @property
    def near_goal(self) -> bool:
        return self._near_goal

    @property
    def at_goal(self) -> bool:
        return self._at_goal

    @property
    def time_skew(self) -> Timestamp:
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def resetMobilityParams(self):
        """
        Resets the mobility parameters used for motion commands to the default values provided by the bosdyn api.
        Returns:
        """
        self._mobility_params = RobotCommandBuilder.mobility_params()

    def robotToLocalTime(self, timestamp: Timestamp) -> Timestamp:
        """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """

        rtime = Timestamp()

        rtime.seconds = timestamp.seconds - self.time_skew.seconds
        rtime.nanos = timestamp.nanos - self.time_skew.nanos
        if rtime.nanos < 0:
            rtime.nanos = rtime.nanos + 1000000000
            rtime.seconds = rtime.seconds - 1

        # Workaround for timestamps being incomplete
        if rtime.seconds < 0:
            rtime.seconds = 0
            rtime.nanos = 0

        return rtime

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        try:
            self._async_tasks.update()
        except Exception as e:
            print(f"Update tasks failed with error: {str(e)}")

    def disconnect(self):
        """Release control of robot as gracefully as posssible."""
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.spot_estop_lease.releaseLease()
        self.spot_estop_lease.releaseEStop()

    def _robot_command(
        self,
        command_proto: robot_command_pb2.RobotCommand,
        end_time_secs: typing.Optional[float] = None,
        timesync_endpoint: typing.Optional[TimeSyncEndpoint] = None,
    ) -> typing.Tuple[bool, str, typing.Optional[str]]:
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
            timesync_endpoint: (optional) Time sync endpoint
        """
        try:
            id = self._robot_clients["robot_command_client"].robot_command(
                lease=None,
                command=command_proto,
                end_time_secs=end_time_secs,
                timesync_endpoint=timesync_endpoint,
            )
            return True, "Success", id
        except Exception as e:
            return False, str(e), None

    def stop(self) -> typing.Tuple[bool, str]:
        """Stop the robot's motion."""
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    def self_right(self) -> typing.Tuple[bool, str]:
        """Have the robot self-right itself."""
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self) -> typing.Tuple[bool, str]:
        """Stop the robot's motion and sit down if able."""
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    def stand(
        self,
        monitor_command: bool = True,
        body_height: float = 0,
        body_yaw: float = 0,
        body_pitch: float = 0,
        body_roll: float = 0,
    ) -> typing.Tuple[bool, str]:
        """
        If the e-stop is enabled, and the motor power is enabled, stand the robot up.
        Executes a stand command, but one where the robot will assume the pose specified by the given parameters.

        If no parameters are given this behave just as a normal stand command

        Args:
            monitor_command: Track the state of the command in the async idle, which sets is_standing
            body_height: Offset of the body relative to normal stand height, in metres
            body_yaw: Yaw of the body in radians
            body_pitch: Pitch of the body in radians
            body_roll: Roll of the body in radians

        """
        if any([body_height, body_yaw, body_pitch, body_roll]):
            # If any of the orientation parameters are nonzero use them to pose the body
            body_orientation = EulerZXY(yaw=body_yaw, pitch=body_pitch, roll=body_roll)
            response = self._robot_command(
                RobotCommandBuilder.synchro_stand_command(
                    body_height=body_height, footprint_R_body=body_orientation
                )
            )
        else:
            # Otherwise just use the mobility params
            response = self._robot_command(
                RobotCommandBuilder.synchro_stand_command(params=self._mobility_params)
            )

        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    def safe_power_off(self) -> typing.Tuple[bool, str]:
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        response = self._robot_command(RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def clear_behavior_fault(self, id: int):
        """Clear the behavior fault defined by id."""
        try:
            rid = self._robot_clients["robot_command_client"].clear_behavior_fault(
                behavior_fault_id=id, lease=None
            )
            return True, "Success", rid
        except Exception as e:
            return False, str(e), None

    def power_on(self) -> typing.Tuple[bool, str]:
        """Enble the motor power if e-stop is enabled."""
        try:
            power.power_on(self._power_client)
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def battery_change_pose(self, dir_hint: int = 1) -> typing.Tuple[bool, str]:
        """Robot sit down and roll on to it its side for easier battery access"""
        response = self._robot_command(
            RobotCommandBuilder.battery_change_pose_command(dir_hint)
        )
        return response[0], response[1]

    def set_mobility_params(self, mobility_params: spot_command_pb2.MobilityParams):
        """Set Params for mobility and movement

        Args:
            mobility_params: spot.MobilityParams, params for spot mobility commands.
        """
        self._mobility_params = mobility_params

    def get_mobility_params(self) -> spot_command_pb2.MobilityParams:
        """Get mobility params"""
        return self._mobility_params

    def velocity_cmd(
        self,
        v_x: float,
        v_y: float,
        v_rot: float,
        cmd_duration: float = 0.125,  # TODO: Investigate what this should be for SDK 3.2.0, ref github issue #96
    ) -> typing.Tuple[bool, str]:
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        end_time = time.time() + cmd_duration
        response = self._robot_command(
            RobotCommandBuilder.synchro_velocity_command(
                v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params
            ),
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )
        self._last_velocity_command_time = end_time
        return response[0], response[1]

    def trajectory_cmd(
        self,
        goal_x: float,
        goal_y: float,
        goal_heading: float,
        cmd_duration: float,
        frame_name: str = "odom",
        precise_position: bool = False,
    ) -> typing.Tuple[bool, str]:
        """Send a trajectory motion command to the robot.

        Args:
            goal_x: Position X coordinate in meters
            goal_y: Position Y coordinate in meters
            goal_heading: Pose heading in radians
            cmd_duration: Time-to-live for the command in seconds.
            frame_name: frame_name to be used to calc the target position. 'odom' or 'vision'
            precise_position: if set to false, the status STATUS_NEAR_GOAL and STATUS_AT_GOAL will be equivalent. If
            true, the robot must complete its final positioning before it will be considered to have successfully
            reached the goal.

        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        self._at_goal = False
        self._near_goal = False
        self._trajectory_status_unknown = False
        self._last_trajectory_command_precise = precise_position
        self._logger.info("got command duration of {}".format(cmd_duration))
        end_time = time.time() + cmd_duration
        if frame_name == "vision":
            vision_tform_body = frame_helpers.get_vision_tform_body(
                self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            )
            body_tform_goal = math_helpers.SE3Pose(
                x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading)
            )
            vision_tform_goal = vision_tform_body * body_tform_goal
            response = self._robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=vision_tform_goal.x,
                    goal_y=vision_tform_goal.y,
                    goal_heading=vision_tform_goal.rot.to_yaw(),
                    frame_name=frame_helpers.VISION_FRAME_NAME,
                    params=self._mobility_params,
                ),
                end_time_secs=end_time,
            )
        elif frame_name == "odom":
            odom_tform_body = frame_helpers.get_odom_tform_body(
                self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            )
            body_tform_goal = math_helpers.SE3Pose(
                x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading)
            )
            odom_tform_goal = odom_tform_body * body_tform_goal
            response = self._robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=odom_tform_goal.x,
                    goal_y=odom_tform_goal.y,
                    goal_heading=odom_tform_goal.rot.to_yaw(),
                    frame_name=frame_helpers.ODOM_FRAME_NAME,
                    params=self._mobility_params,
                ),
                end_time_secs=end_time,
            )
        else:
            raise ValueError("frame_name must be 'vision' or 'odom'")
        if response[0]:
            self._last_trajectory_command = response[2]
        return response[0], response[1]

    def update_image_tasks(self, image_name):
        """Adds an async tasks to retrieve images from the specified image source"""

        task_to_add = self.camera_task_name_to_task_mapping[image_name]

        if task_to_add == self._hand_image_task and not self._robot.has_arm():
            self._logger.warn(
                "Robot has no arm, therefore the arm image task can not be added"
            )
            return

        if task_to_add in self._async_tasks._tasks:
            self._logger.warn(
                f"Task {image_name} already in async task list, will not be added again"
            )
            return

        self._async_tasks.add_task(self.camera_task_name_to_task_mapping[image_name])
