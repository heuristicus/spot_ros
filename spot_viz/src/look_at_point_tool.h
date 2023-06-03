#ifndef LOOK_AT_POINT_TOOL_H
#define LOOK_AT_POINT_TOOL_H

#ifndef Q_MOC_RUN
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <rviz/default_plugin/tools/focus_tool.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#endif

namespace rviz {
class DisplayContext;
class Arrow;
} // namespace rviz

namespace spot_viz {

class LookAtPointTool : public rviz::FocusTool {
  Q_OBJECT
public:
  LookAtPointTool();
  ~LookAtPointTool();

  void activate() override;

  virtual int processMouseEvent(rviz::ViewportMouseEvent &event) override;

private:
  rviz::BoolProperty* track_property_;
  rviz::FloatProperty* zoom_property_;
  rviz::FloatProperty* image_width_property_;
  ros::ServiceClient lookAtPointSrv_;
  ros::NodeHandle nh_;
};
} // end namespace spot_viz

#endif // LOOK_AT_POINT_TOOL_H