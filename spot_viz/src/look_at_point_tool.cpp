#include <OGRE/OgreSceneNode.h>
#include <geometry_msgs/Pose.h>

#include <actionlib/client/terminal_state.h>
#include <ros/console.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <spot_cam/LookAtPoint.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "look_at_point_tool.h"

namespace spot_viz {

LookAtPointTool::LookAtPointTool() {
  shortcut_key_ = 'l';
  track_property_ = new rviz::BoolProperty(
      "Track", false,
      "If true, the camera will track the point as the robot moves",
      getPropertyContainer());
  zoom_property_ = new rviz::FloatProperty(
      "Zoom level", 1, "Zoom level to apply", getPropertyContainer());
  image_width_property_ = new rviz::FloatProperty(
      "Image width", 0,
      "Length of the diagonal of the image at the distance to the point",
      getPropertyContainer());
}

LookAtPointTool::~LookAtPointTool() {}

void LookAtPointTool::activate() {
  // (re)initialise the add node server on tool activation to reduce likelihood
  // of it not being available. we don't want to wait for the service to become
  // available because this causes rviz to hang
  lookAtPointSrv_ = nh_.serviceClient<spot_cam::LookAtPoint>(
      "/spot/cam/ptz/look_at_point", true);
}

int LookAtPointTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
  int flags = 0;

  Ogre::Vector3 pos;
  bool success = context_->getSelectionManager()->get3DPoint(
      event.viewport, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success) {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Point the CAM PTZ at this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus(s.str().c_str());
  }

  if (event.leftUp()) {
    spot_cam::LookAtPoint srv;
    srv.request.target.point.x = pos.x;
    srv.request.target.point.y = pos.y;
    srv.request.target.point.z = pos.z;
    srv.request.target.header.stamp = ros::Time::now();
    // Frame ID is the fixed frame rviz is currently using
    srv.request.target.header.frame_id = context_->getFixedFrame().toStdString();
    srv.request.image_width = image_width_property_->getFloat();
    srv.request.zoom_level = zoom_property_->getFloat();
    srv.request.track = track_property_->getBool();
    if (lookAtPointSrv_.call(srv)) {
      if (srv.response.success) {
        std::ostringstream s;
        s << "Sent goal to look at point: " << srv.response.message.c_str();
        setStatus(s.str().c_str());
      } else {
        std::ostringstream s;
        s << "Failed to look at point: " << srv.response.message.c_str();
        setStatus(s.str().c_str());
      }
    }
    else {
      std::string warnStr("Failed to call look at point service. See error "
                          "messages in rviz output.");
      setStatus(warnStr.c_str());
      ROS_WARN("%s", warnStr.c_str());
    }
    flags |= Finished;
  }

  return flags;
}

} // namespace spot_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(spot_viz::LookAtPointTool, rviz::Tool)
