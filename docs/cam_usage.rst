Spot CAM
========

.. warning::

  Installing the wrong version of aiortc can break pip on certain versions of ubuntu due to an openssl upgrade. On
  ubuntu 20 you must install ``aiortc==1.3.2``. If you break pip, it should be fixable by going to
  ``~/.local/lib/python3.8/site-packages`` and removing the OpenSSL directory and then installing aiortc again correctly.

.. note::

  The spot cam has a fixed IP address of 192.168.50.6. If you have a payload with that address the driver will not
  function. You must change the address of your payload.


Basics
------

The spot CAM driver runs separately to the main spot driver. The camera does not require a lease to be held to
control it, but it is necessary to authenticate to use it.

The CAM launch has the same authentication parameters as the spot launch. When connected to the robot via wifi, you
can start the driver with

.. code:: bash

  roslaunch spot_cam spot_cam.launch username:=user password:=spot_password hostname:=192.168.80.3

/spot/cam/cycle_power
/spot/cam/image
/spot/cam/image/compressed
/spot/cam/image/compressed/parameter_descriptions
/spot/cam/image/compressed/parameter_updates
/spot/cam/ptz/list
/spot/cam/ptz/positions
/spot/cam/ptz/velocities
/spot/cam/screens
/spot/cam/set_leds
/spot/cam/set_power
/spot/cam/status/built_in_test
/spot/cam/status/leds
/spot/cam/status/power
/spot/cam/status/temperatures
/spot/cam/stream/params


/spot/cam/audio/set_volume
/spot/cam/image/compressed/set_parameters
/spot/cam/ptz/look_at_point
/spot/cam/ptz/reset_autofocus
/spot/cam/ptz/set_position
/spot/cam/ptz/set_velocity
/spot/cam/set_ir_colormap
/spot/cam/set_ir_meter_overlay
/spot/cam/set_screen
/spot/cam/stream/enable_congestion_control
/spot/cam/stream/set_params


Screens
-------

The CAM does not provide all the images from itself at the same time. It is up to the user to select a "screen" to
display.

You can find some more details about the screens in the cam's`WebRTC guide <https://support.bostondynamics.com/s/article/Spot-CAM-WebRTC-guide>`_. Part of description below is taken from there.

    - pano: panoramic (the 360 degree view)
    - digi: center of panoramic (user can move this subsection on the Tablet)
    - c: raw fisheye feed from the 0 thru 4 cameras of the 360 (e.g "c0"...c4")
    - mech: image from the ptz

    The composites are organized based on whether they feature digi, mech, or c.


There are three varieties of the digi or mech using the following adjectives:
- no adjective: The digi or mech image on top with stitched 360 image along the bottom of the image.
- overlay: The same as "no adjective" except the mech or digi image is much larger.
- full: mech or digi is full screen.

Each screen displays different images or combinations of images. What is available depends on the specific model of
the CAM you have.

+---------------+-------------+-------------+
|Name           | CAM version | Description |
+===============+=============+=============+
|c0             |     CAM     |             |
+---------------+-------------+-------------+
|c1             |     CAM     |             |
+---------------+-------------+-------------+
|c2             |     CAM     |             |
+---------------+-------------+-------------+
|c3             |     CAM     |             |
+---------------+-------------+-------------+
|c4             |     CAM     |             |
+---------------+-------------+-------------+
|pano_full      |     CAM     |             |
+---------------+-------------+-------------+
|digi           |     CAM     |             |
+---------------+-------------+-------------+
|digi_overlay   |     CAM     |             |
+---------------+-------------+-------------+
|digi_full      |     CAM     |             |
+---------------+-------------+-------------+
|mech           |    CAM+     |             |
+---------------+-------------+-------------+
|mech_full      |    CAM+     |             |
+---------------+-------------+-------------+
|mech_overlay   |    CAM+     |             |
+---------------+-------------+-------------+
|mech_ir        |   CAM+IR    |             |
+---------------+-------------+-------------+
|mech_full_ir   |   CAM+IR    |             |
+---------------+-------------+-------------+
|mech_overlay_ir|   CAM+IR    |             |
+---------------+-------------+-------------+

You can list the available screens and set the screen to display.

PTZs
----

While there is only one true pan-tilt-zoom camera on the CAM, there are pseudo-PTZs that exist for convenience when
looking at the panoramic camera or other components.

+---------------+-------------+
|Name           | Description |
+===============+=============+
|pano           |             |
+---------------+-------------+
|full_pano      |             |
+---------------+-------------+
|overlay_pano   |             |
+---------------+-------------+
|digi           |             |
+---------------+-------------+
|full_digi      |             |
+---------------+-------------+
|overlay_digi   |             |
+---------------+-------------+
|mech           |             |
+---------------+-------------+

You can control a PTZ through services

Audio
-----

You can use the CAM to play audio files that you load onto it. The files must be in the ``wav`` format.
Use the ``/spot/cam/audio/load`` service to load a sound into the cam. Each sound has a ``name`` which is used to refer to it.

CHECK IF SOUNDS PERSIST OVER SHUTDOWN

.. code::

  rosservice call /spot/cam/audio/load "name: 'cam_test' wav_path: '~/Downloads/cam_test.wav'"

You can play sounds with the ``/spot/cam/audio/play`` service. You can make the sound louder using the ``gain`` argument.

.. code::

  rosservice call /spot/cam/audio/play "name: 'cam_test' gain: 0.0"

You can delete sounds with the ``/spot/cam/audio/delete`` service