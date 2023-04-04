Spot CAM
========

.. warning::

  Installing the wrong version of aiortc can break pip on certain versions of ubuntu due to an openssl upgrade. On
  ubuntu 20 you must install ``aiortc==1.3.2``. If you break pip, it should be fixable by going to
  ``~/.local/lib/python3.8/site-packages`` and removing the OpenSSL directory and then installing aiortc again correctly.

.. note::

  The spot cam has a fixed IP address of 192.168.50.6. If you have a payload with that address the driver will not function.

The driver can also send commands to the robots arm, if it has one. The following services allow control of various
parts of the arm.

  - mech_full_ir
  - mech_ir
  - mech_overlay_ir
  - pano_full
  - c0
  - mech_full
  - c1
  - c2
  - c3
  - digi_overlay
  - c4
  - digi_full
  - mech_overlay
  - mech
  - digi


    name: "digi"
    name: "full_digi"
    name: "full_pano"
    name: "overlay_digi"
    name: "overlay_pano"
    name: "pano"
    name: "mech"

You can find some more details about the screens in the cam's `WebRTC guide _<https://support.bostondynamics.com/s/article/Spot-CAM-WebRTC-guide>`.