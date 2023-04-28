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

Once you have started the cam driver, you can access a lot of functionality through the rviz panel launched with ``roslaunch spot_viz view_robot.launch``

.. image:: images/cam_panel.png

Screens
-------

The CAM does not provide all the images from itself at the same time. It is up to the user to select a "screen" to
display.

The currently showing screen is displayed on the ``/spot/cam/image`` and ``/spot/cam/image/compressed`` topics.

You can find some more details about the screens in the cam's `WebRTC guide <https://support.bostondynamics.com/s/article/Spot-CAM-WebRTC-guide>`_. Part of description below is taken from there.

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

+---------------+-------------+-------------------------------------------------------------------------------------------+
|Name           | CAM version |Description                                                                                |
+===============+=============+===========================================================================================+
|c0             |     CAM     |Rear left panoramic camera                                                                 |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|c1             |     CAM     |Front left panoramic camera                                                                |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|c2             |     CAM     |Front panoramic camera                                                                     |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|c3             |     CAM     |Front right panoramic camera                                                               |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|c4             |     CAM     |Rear right panoramic camera                                                                |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|pano_full      |     CAM     |Stitched panoramic camera view                                                             |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|digi           |     CAM     |Small view of panoramic pseudo-PTZ, with stitched panorama along the bottom                |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|digi_overlay   |     CAM     |Larger view of the panoramic pseudo-PTZ, small stitched panorama                           |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|digi_full      |     CAM     |Full view of the panoramic pseudo-PTZ                                                      |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|mech           |    CAM+     |Small view of the PTZ, with stitched panorama along the bottom                             |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|mech_full      |    CAM+     |Larger view of the PTZ, small stitched panorama                                            |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|mech_overlay   |    CAM+     |Full view of the PTZ                                                                       |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|mech_ir        |   CAM+IR    |Small view of the IR camera, with stitched panorama                                        |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|mech_full_ir   |   CAM+IR    |Larger view of the IR camera, small stitched panorama                                      |
+---------------+-------------+-------------------------------------------------------------------------------------------+
|mech_overlay_ir|   CAM+IR    |Full view of the IR camera                                                                 |
+---------------+-------------+-------------------------------------------------------------------------------------------+

To see the available screens, you can echo the ``/spot/cam/screens`` topic.

To set the screen that should be displayed by ``/spot/cam/image``, use the ``/spot/cam/set_screen`` service.


Stream control
^^^^^^^^^^^^^^

You can control some of the parameters of how the stream is output.

To check the current parameters, see the ``/spot/cam/stream/params`` topic.

You can also set the parameters with the ``/spot/cam/stream/set_params`` service.
See `stream params <https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#streamparams>`_ for more details.

The ``/spot/cam/stream/enable_congestion_control`` service enables or disables receiver congestion control.

PTZs
----

While there is only one true pan-tilt-zoom camera on the CAM, there are pseudo-PTZs that exist for convenience when
looking at the panoramic camera or other components.

+---------------+---------------------------------------------------------------------------------------+
|Name           |Description                                                                            |
+===============+=======================================================================================+
|pano           |Control the orientation of the panoramic image                                         |
+---------------+---------------------------------------------------------------------------------------+
|full_pano      |Same as pano (?)                                                                       |
+---------------+---------------------------------------------------------------------------------------+
|overlay_pano   |Same as pano (?)                                                                       |
+---------------+---------------------------------------------------------------------------------------+
|digi           |Control which part of the panoramic image the cutout image is taken from               |
+---------------+---------------------------------------------------------------------------------------+
|full_digi      |Same as digi (?)                                                                       |
+---------------+---------------------------------------------------------------------------------------+
|overlay_digi   |Same as digi (?)                                                                       |
+---------------+---------------------------------------------------------------------------------------+
|mech           |Control the mechanical PTZ                                                             |
+---------------+---------------------------------------------------------------------------------------+


topic

The ``/spot/cam/ptz/list`` topic shows the names and pan/tilt/zoom limits for each ptz.

.. code::

    rostopic echo /spot/cam/ptz/list
    ptzs:
      -
        name: "digi"
        pan_limit:
          min: 0.0
          max: 0.0
        tilt_limit:
          min: -90.0
          max: 90.0
        zoom_limit:
          min: 1.0
          max: 30.0
    ...
      -
        name: "mech"
        pan_limit:
          min: 0.0
          max: 0.0
        tilt_limit:
          min: -30.0
          max: 90.0
        zoom_limit:
          min: 1.0
          max: 30.0
    ---


The ``/spot/cam/ptz/positions`` topic shows the current position of each available ptz.

.. code::

    rostopic echo /spot/cam/ptz/positions
    ptzs:
      -
        header:
          seq: 0
          stamp:
            secs: 1681224859
            nsecs: 768042325
          frame_id: ''
        ptz:
          name: "digi"
          pan_limit:
            min: 0.0
            max: 0.0
          tilt_limit:
            min: -90.0
            max: 90.0
          zoom_limit:
            min: 1.0
            max: 30.0
        pan: 5.008956577512436e-06
        tilt: 30.0
        zoom: 1.0
    ...
      -
        header:
          seq: 0
          stamp:
            secs: 1681224861
            nsecs:  49465894
          frame_id: ''
        ptz:
          name: "mech"
          pan_limit:
            min: 0.0
            max: 0.0
          tilt_limit:
            min: -30.0
            max: 90.0
          zoom_limit:
            min: 1.0
            max: 30.0
        pan: 99.92222595214844
        tilt: 0.0
        zoom: 1.0
    ---

The ``/spot/cam/ptz/velocities`` topic generates similar output to the positions topic but specifying the velocities
of the ptzs instead.

The ``/spot/cam/ptz/reset_autofocus`` service will reset the autofocus of the mechanical PTZ.

The ``/spot/cam/ptz/set_position`` service will set the position of the named ptz according to the specified pan,
tilt, and zoom. The limits and header are ignored.

.. code::

    rosservice call /spot/cam/ptz/set_position "command:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
      ptz:
        name: 'mech'
        pan_limit: {min: 0.0, max: 0.0}
        tilt_limit: {min: 0.0, max: 0.0}
        zoom_limit: {min: 0.0, max: 0.0}
      pan: 50
      tilt: 10
      zoom: 30"

The ``/spot/cam/ptz/set_velocity`` service should set the velocity of the PTZs, but it seems that the PTZs do not
allow setting velocities.

Looking at a point
^^^^^^^^^^^^^^^^^^

The ``/spot/cam/ptz/look_at_point`` service provides a method for making the camera point at a specific xyz coordinate
in an arbitrary tf frame. This is also available as an actionserver.

.. code::

    rosservice call /spot/cam/ptz/look_at_point "target:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs: 0
        frame_id: 'body'
      point:
        x: 1.0
        y: 2.0
        z: 3.0
    image_width: 0.0
    zoom_level: 0.0
    track: false"

The ``target`` field is a ``PointStamped`` which you want to point the camera at.

The ``image_width`` in metres is a request to make the image output by the ptz camera have this width on the diagonal axis,
based on the distance of the point from the camera. This is based on focal length computations and may not be
precisely achieved. This also overrides a specifically requested zoom level if both are provided.

The ``zoom_level`` specifies the optical zoom that should be set, between 1 and 30.

.. note::

  The ``track`` boolean does not work very well if the robot is moving quickly.

``track`` is a request to track the point as the robot moves.

IR Camera
---------

There are two services for controlling some IR camera parameters.

The ``/spot/cam/set_ir_colormap`` service can be used to change the colourmap, set the minimum and maximum temperature
that should be used to scale the image, or set automatic scaling.

.. code::

    rosservice call /spot/cam/set_ir_colormap "colormap: 4
    min: 0.0
    max: 0.0
    auto_scale: true"

.. figure:: images/ir_colormaps.png

  IR colourmaps from left to right: greyscale (1), jet (2), inferno (3), turbo (4)

The ``/spot/cam/set_ir_meter_overlay`` service can be used to turn the point temperature overlay on or off, or specify
its x-y position in the image frame. The x-y values are specified between 0 and 1, and start at the top left corner.

.. code::

    rosservice call /spot/cam/set_ir_meter_overlay "x: 0.0
    y: 0.0
    enable: false"

Audio
-----

You can use the CAM to play audio files that you load onto it. The files must be in the ``wav`` format.
Use the ``/spot/cam/audio/load`` service to load a sound into the cam. Each sound has a ``name`` which is used to
refer to it. Loaded sounds persist across reboots of the robot.

.. code::

  rosservice call /spot/cam/audio/load "name: 'cam_test' wav_path: '~/Downloads/cam_test.wav'"

You can play sounds with the ``/spot/cam/audio/play`` service. You can make the sound louder using the ``gain`` argument.

.. code::

  rosservice call /spot/cam/audio/play "name: 'cam_test' gain: 0.0"

You can delete sounds with the ``/spot/cam/audio/delete`` service

.. code::

  rosservice call /spot/cam/audio/delete "name: 'cam_test'"

There is additional volume control with the ``/spot/cam/audio/set_volume`` service, which can be used to set a volume
between 0 and 100 which will be applied to all sounds.

.. code::

  rosservice call /spot/cam/audio/set_volume "value: 100"


LEDs
----

The brightness of the LEDs can be viewed using the ``/spot/cam/status/leds`` topic.

To set the brightness of all the LEDs, you can use the ``/spot/cam/set_leds`` topic, which sets all of them to the
same specified brightness.

Power control
-------------

The ``/spot/cam/status/power`` topic shows the power status of various components.

Publishing to the ``/spot/cam/cycle_power`` topic can cycle power to the various components.

Publishing to the ``/spot/cam/set_power`` topic can be used to turn on or off various components.

Other topics
-------------

There are several status topics available for the camera.

The ``/spot/cam/status/built_in_test`` topic will give information about camera events or degradations that have occurred.

The ``/spot/cam/status/temperatures`` topic shows the temperatures of various components of the camera.