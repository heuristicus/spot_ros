import asyncio
import datetime
import enum
import os.path
import threading
import typing
import wave

import bosdyn.client
import cv2
import numpy as np
from aiortc import RTCConfiguration
from bosdyn.client import Robot
from bosdyn.client import spot_cam
from bosdyn.client.spot_cam.compositor import CompositorClient
from bosdyn.client.spot_cam.lighting import LightingClient
from bosdyn.client.spot_cam.power import PowerClient
from bosdyn.client.spot_cam.health import HealthClient
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.api.spot_cam import audio_pb2

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


class HealthWrapper:
    """
    Wrapper for health details
    """

    def __init__(self, robot, logger):
        self.client: HealthClient = robot.ensure_client(
            HealthClient.default_service_name
        )
        self.logger = logger

    def get_bit_status(
        self,
    ) -> typing.Tuple[typing.List[str], typing.List[typing.Tuple[int, str]]]:
        """
        Get fault events and degradations

        Returns:
            Dictionary

        """
        bit_status = self.client.get_bit_status()
        events = []
        for event in bit_status[0]:
            events.append(event)

        degradations = []
        for degradation in bit_status[1]:
            degradations.append((degradation.type, degradation.description))
        return events, degradations

    def get_temperature(self) -> typing.Tuple[str, float]:
        """
        Get temperatures of various components of the camera

        Returns:
            Tuple of string and float indicating the component and its temperature in celsius
        """
        return [
            (composite.channel_name, composite.temperature / 1000.0)
            for composite in self.client.get_temperature()
        ]

    # def get_system_log(self):
    #     """
    #     This seems to always time out
    #     """
    #     return self.client.get_system_log()


class AudioWrapper:
    """
    Wrapper for audio commands on the camera
    """

    def __init__(self, robot, logger):
        self.client: AudioClient = robot.ensure_client(AudioClient.default_service_name)
        self.logger = logger

    def list_sounds(self) -> typing.List[str]:
        """
        List sounds available on the device

        Returns:
            List of names of available sounds
        """
        return self.client.list_sounds()

    def set_volume(self, percentage):
        """
        Set the volume at which sounds should be played

        Args:
            percentage: How loud sounds should be from 0 to 100%
        """
        self.client.set_volume(percentage)

    def get_volume(self):
        """
        Get the current volume at which sounds are played

        Returns:
            Current volume as a percentage
        """
        return self.client.get_volume()

    def play_sound(self, sound_name, gain=1.0):
        """
        Play a sound which is on the device

        Args:
            sound_name: Name of the sound to play
            gain: Volume gain multiplier
        """
        sound = audio_pb2.Sound(name=sound_name)
        self.client.play_sound(sound, gain)

    def load_sound(self, sound_file, name):
        """
        Load a sound from a wav file and save it with the given name onto the device
        Args:
            sound_file: Wav file to read from
            name: Name to assign to the sound

        Raises:
            IOError: If the given file is not a file
            wave.Error: If the given file is not a wav file
        """
        full_path = os.path.abspath(os.path.expanduser(sound_file))
        print(full_path)
        if not os.path.isfile(full_path):
            raise IOError(f"Tried to load sound from {full_path} but it is not a file.")

        sound = audio_pb2.Sound(name=name)

        with wave.open(full_path, "rb") as fh:
            # Use this to make sure that the file is actually a wav file
            pass

        with open(full_path, "rb") as fh:
            data = fh.read()

        self.client.load_sound(sound, data)

    def delete_sound(self, name):
        """
        Delete a sound from the device

        Args:
            name: Name of the sound to delete
        """
        self.client.delete_sound(audio_pb2.Sound(name=name))


class ImageStreamWrapper:
    """
    A wrapper for the image stream from WebRTC

    Contains functions adapted from
    https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/spot_cam/webrtc.py
    """

    def __init__(
        self,
        hostname: str,
        robot,
        logger,
        sdp_port=31102,
        sdp_filename="h264.sdp",
        cam_ssl_cert_path=None,
    ):
        """
        Initialise the wrapper

        Args:
            hostname: Hostname/IP of the robot
            robot: Handle for the robot the camera is on
            logger: Logger to use
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
            sdp_port,
            sdp_filename,
            cam_ssl_cert_path if cam_ssl_cert_path else False,
            robot.user_token,
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
        self.sdk = bosdyn.client.create_standard_sdk("Spot CAM Client")
        spot_cam.register_all_service_clients(self.sdk)

        self.robot = self.sdk.create_robot(self._hostname)
        SpotWrapper.authenticate(
            self.robot, self._hostname, self._username, self._password, self._logger
        )

        # TODO: Work out how to distinguish between spot cam, spot cam +, and spot cam + IR
        self.lighting = LightingWrapper(self.robot, self._logger)
        self.power = PowerWrapper(self.robot, self._logger)
        self.compositor = CompositorWrapper(self.robot, self._logger)
        self.image = ImageStreamWrapper(self._hostname, self.robot, self._logger)
        self.health = HealthWrapper(self.robot, self._logger)
        self.audio = AudioWrapper(self.robot, self._logger)

        self._logger.info("Finished setting up spot cam wrapper components")

    def shutdown(self):
        self._logger.info("Shutting down Spot CAM wrapper")
        self.image.shutdown_flag.set()