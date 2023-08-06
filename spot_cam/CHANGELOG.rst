^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spot_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.1.0 (2023-06-14)
------------------
* fix playsound service, add action for look at point
* rename hold->track, look at point in the cam panel
* using hold function seems to cause issues with tf broadcaster? try and catch
* initial implementation of hold functionality for spot cam look at point
* Add look at point functionality for cam
  Co-authored-by: Tobit Flatscher <53856473+2b-t@users.noreply.github.com>
* remove cam wrapper in this repo
* update cam files to use wrapper submodule
* panel for spot cam interaction
* add some basic structure for look at point functionality, more value checking in service calls
* formatting and stuff from rebase
* change the frame id of the image output depending on the camera displayed in the stream
* publish transforms for cameras on the device
* remove redundant conversion to dict in wrapper
* ptz handler publishes position and velocity of ptzs, can set position and velocity
* initial implementation of ptz wrapper and handler, can list ptzs
* add stream quality wrapper and ros handler
* update webrtc_client to 3.2 version
* add handler and wrapper for audio commands
* Add health wrapper, move body of robotToLocalTime out of spot wrapper object
  robotToLocalTime now takes the timestamp and a robot object, which allows it to
  be used by the spot cam wrapper as well.
* add compositor to handle IR and webrtc stream selection with services
  Add timestamp for the webrtc images
  Add compressed version of the webrtc image stream
* most basic functional image stream publisher with webrtc
* Add power control, but unclear if it is actually possible to set power for aux and external mic
* simple led brightness control, only able to set all leds to same value
* Initial layout of spot cam package, authenticating with robot
* Contributors: Michal Staniaszek

1.0.0 (2023-04-07 16:17)
------------------------

0.0.0 (2023-04-07 15:10)
------------------------
