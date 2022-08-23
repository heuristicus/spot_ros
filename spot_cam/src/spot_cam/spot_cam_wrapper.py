import asyncio
import datetime
import enum
import threading
import typing

import bosdyn.client
import cv2
import numpy as np
from aiortc import RTCConfiguration
from bosdyn.client import Robot
from bosdyn.client import spot_cam
from bosdyn.client.spot_cam.compositor import CompositorClient
from bosdyn.client.spot_cam.lighting import LightingClient
from bosdyn.client.spot_cam.power import PowerClient
from bosdyn.client.exceptions import InvalidRequestError

from spot_cam.webrtc_client import WebRTCClient
from spot_driver.spot_wrapper import SpotWrapper


class LightingWrapper:
    """
    Wrapper for LED brightness interaction
    """

    class LEDPosition(enum.Enum):
        """
        Values indicate the position of the specified LED in the brightness list
        """

        REAR_LEFT = 0
        FRONT_LEFT = 1
        FRONT_RIGHT = 2
        REAR_RIGHT = 3

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: LightingClient = robot.ensure_client(
            LightingClient.default_service_name
        )

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


class PowerWrapper:
    """
    Wrapper for power interaction
    """

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: PowerClient = robot.ensure_client(PowerClient.default_service_name)

    def get_power_status(self):
        """
        Get power status for the devices
        """
        return self.client.get_power_status()

    def set_power_status(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
    ):
        """
        Set power status for each of the devices

        Args:
            ptz:
            aux1: ??
            aux2: ??
            external_mic:
        """
        self.client.set_power_status(ptz, aux1, aux2, external_mic)

    def cycle_power(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
    ):
        """
        Cycle power of the specified devices

        Args:
            ptz:
            aux1:
            aux2:
            external_mic:
        """
        self.client.cycle_power(ptz, aux1, aux2, external_mic)


class CompositorWrapper:
    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: CompositorClient = robot.ensure_client(
            CompositorClient.default_service_name
        )

    def list_screens(self) -> typing.List[str]:
        """
        List the available screens - this includes individual cameras and also panoramic and other stitched images
        provided by the camera

        Returns:
             List of strings indicating available screens
        """
        return [screen.name for screen in self.client.list_screens()]

    def set_screen(self, screen):
        """
        Set the screen to be streamed over the network

        Args:
            screen: Screen to show
        """
        self.client.set_screen(screen)

    def set_ir_colormap(self, colormap, min_temp, max_temp, auto_scale=True):
        """
        Set the colormap used for the IR camera

        Args:
            colormap: Colormap to use, options are https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ircolormap-colormap
            min_temp: Minimum temperature on the scale
            max_temp: Maximum temperature on the scale
            auto_scale: Auto-scales the colormap according to the image. If this is set min_temp and max_temp are
                        ignored
        """
        self.client.set_ir_colormap(colormap, min_temp, max_temp, auto_scale)

    def set_ir_meter_overlay(self, x, y, enable=True):
        """
        Set the reticle position on the Spot CAM IR.

        Args:
            x: Horizontal coordinate between 0 and 1
            y: Vertical coordinate between 0 and 1
            enable: If true, enable the reticle on the display
        """
        self.client.set_ir_meter_overlay(x, y, enable)


class ImageStreamWrapper:
    """
    A wrapper for the image stream from WebRTC

    Contains functions adapted from
    https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/spot_cam/webrtc.py
    """

    def __init__(
        self,
        hostname: str,
        username: str,
        password: str,
        logger,
        sdp_port=31102,
        sdp_filename="h264.sdp",
        cam_ssl_cert_path=None,
    ):
        """
        Initialise the wrapper

        Args:
            hostname: Hostname/IP of the robot
            username: Username on the robot
            password: Password for the given user
            sdp_port: SDP port of Spot's WebRTC server
            sdp_filename: File being streamed from the WebRTC server
            cam_ssl_cert_path: Path to the Spot CAM's client cert to check with Spot CAM server
        """
        self.shutdown_flag = threading.Event()
        self.logger = logger
        self.last_image_time = None
        self.image_lock = threading.Lock()
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        config = RTCConfiguration(iceServers=[])
        self.client = WebRTCClient(
            hostname,
            username,
            password,
            sdp_port,
            sdp_filename,
            cam_ssl_cert_path if cam_ssl_cert_path else False,
            config,
        )

        asyncio.gather(
            self.client.start(),
            self._process_func(),
            self._monitor_shutdown(),
        )
        # Put the async loop into a separate thread so we can continue initialisation
        self.async_thread = threading.Thread(target=loop.run_forever)
        self.async_thread.start()

    async def _monitor_shutdown(self):
        while not self.shutdown_flag.is_set():
            await asyncio.sleep(1.0)

        self.logger.info("Image stream wrapper received shutdown flag")
        await self.client.pc.close()
        asyncio.get_event_loop().stop()

    async def _process_func(self):
        while asyncio.get_event_loop().is_running():
            try:
                frame = await self.client.video_frame_queue.get()

                pil_image = frame.to_image()
                cv_image = np.array(pil_image)
                # OpenCV needs BGR
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                with self.image_lock:
                    self.last_image_time = datetime.datetime.now()
                    self.last_image = cv_image
            except Exception as e:
                self.logger.error(f"Image stream wrapper exception {e}")
            try:
                # discard audio frames
                while not self.client.audio_frame_queue.empty():
                    await self.client.audio_frame_queue.get()
            except Exception as e:
                self.logger.error(
                    f"Image stream wrapper exception while discarding audio frames {e}"
                )

        self.shutdown_flag.set()


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

        # TODO: Work out how to distinguish between spot cam, spot cam +, and spot cam + IR
        self.lighting = LightingWrapper(robot, self._logger)
        self.power = PowerWrapper(robot, self._logger)
        self.compositor = CompositorWrapper(robot, self._logger)
        self.image = ImageStreamWrapper(
            self._hostname, self._username, self._password, self._logger
        )

        self._logger.info("Finished setting up spot cam wrapper components")

    def shutdown(self):
        self._logger.info("Shutting down Spot CAM wrapper")
        self.image.shutdown_flag.set()
