#ifndef SPOT_CONTROL_PANEL_H
#define SPOT_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <map>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <std_msgs/Bool.h>
#include <spot_msgs/EStopStateArray.h>
#include <spot_msgs/MobilityParams.h>
#include <spot_msgs/TerrainParams.h>
#include <spot_msgs/SetSwingHeight.h>
#include <spot_msgs/LeaseArray.h>
#include <spot_msgs/SetLocomotion.h>
#include <spot_msgs/BatteryStateArray.h>
#include <spot_msgs/PowerState.h>


namespace spot_viz
{

class ControlPanel : public rviz::Panel
{
    Q_OBJECT
    public:

    ControlPanel(QWidget *parent=0);
    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config &config);

    private Q_SLOTS:
    void sit();
    void stand();
    void claimLease();
    void releaseLease();
    void powerOn();
    void powerOff();
    void sendBodyPose();
    void sendNeutralBodyPose();
    void setMaxVel();
    void gentleStop();
    void hardStop();
    void releaseStop();
    void stop();
    void setGait();
    void setSwingHeight();
    void setTerrainParams();
    void setObstacleParams();
    void allowMotion();

    private:

    // These maps allow us to set up the comboboxes for selections in the order
    // of the enum, and ensure that the correct value is sent when the user wants to set it
    // We can also use them to ensure that non-consecutive values are also correctly handled
    const std::map<uint, std::string> gaitMap = {
        {spot_msgs::SetLocomotion::Request::HINT_UNKNOWN, "Unknown"},
        {spot_msgs::SetLocomotion::Request::HINT_TROT, "Trot"},
        {spot_msgs::SetLocomotion::Request::HINT_SPEED_SELECT_TROT, "Speed sel trot"},
        {spot_msgs::SetLocomotion::Request::HINT_CRAWL, "Crawl"},
        {spot_msgs::SetLocomotion::Request::HINT_AMBLE, "Amble"},
        {spot_msgs::SetLocomotion::Request::HINT_SPEED_SELECT_AMBLE, "Speed sel amble"},
        {spot_msgs::SetLocomotion::Request::HINT_AUTO,  "Auto"},
        {spot_msgs::SetLocomotion::Request::HINT_JOG, "Jog"},
        {spot_msgs::SetLocomotion::Request::HINT_HOP, "Hop"},
        {spot_msgs::SetLocomotion::Request::HINT_SPEED_SELECT_CRAWL, "Speed sel crawl"}
    };

    const std::map<uint, std::string> swingHeightMap = {
        {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_UNKNOWN, "Unknown"},
        {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_LOW, "Low"},
        {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_MEDIUM, "Medium"},
        {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_HIGH, "High"}
    };

    const std::map<uint, std::string> gratedSurfacesMap = {
        {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_UNKNOWN, "Unknown"},
        {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_OFF, "Off"},
        {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_ON, "On"},
        {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_AUTO, "Auto"}
    };

    void setupComboBoxes();
    void setupStopButtons();
    void setupSpinBoxes();
    void setControlButtons();
    void toggleBodyPoseButtons();
    bool callTriggerService(ros::ServiceClient service, std::string serviceName);
    template <typename T>
    bool callCustomTriggerService(ros::ServiceClient service, std::string serviceName, T serviceRequest);
    void updateLabelTextWithLimit(QLabel* label, double limit_lower, double limit_upper);
    void leaseCallback(const spot_msgs::LeaseArray::ConstPtr &leases);
    void estopCallback(const spot_msgs::EStopStateArray::ConstPtr &estops);
    void mobilityParamsCallback(const spot_msgs::MobilityParams::ConstPtr &params);
    void batteryCallback(const spot_msgs::BatteryStateArray::ConstPtr &battery);
    void powerCallback(const spot_msgs::PowerState::ConstPtr &power);
    void motionAllowedCallback(const std_msgs::Bool &motion_allowed);

    ros::NodeHandle nh_;
    ros::ServiceClient sitService_;
    ros::ServiceClient standService_;
    ros::ServiceClient claimLeaseService_;
    ros::ServiceClient releaseLeaseService_;
    ros::ServiceClient powerOnService_;
    ros::ServiceClient powerOffService_;
    ros::ServiceClient maxVelocityService_;
    ros::ServiceClient hardStopService_;
    ros::ServiceClient gentleStopService_;
    ros::ServiceClient releaseStopService_;
    ros::ServiceClient stopService_;
    ros::ServiceClient gaitService_;
    ros::ServiceClient swingHeightService_;
    ros::ServiceClient terrainParamsService_;
    ros::ServiceClient obstacleParamsService_;
    ros::ServiceClient allowMotionService_;
    ros::ServiceClient bodyPoseService_;

    ros::Subscriber leaseSub_;
    ros::Subscriber estopSub_;
    ros::Subscriber mobilityParamsSub_;
    ros::Subscriber batterySub_;
    ros::Subscriber powerSub_;
    ros::Subscriber motionAllowedSub_;

    QPushButton* claimLeaseButton;
    QPushButton* releaseLeaseButton;
    QPushButton* powerOnButton;
    QPushButton* powerOffButton;
    QPushButton* setBodyPoseButton;
    QPushButton* setBodyNeutralButton;
    QPushButton* sitButton;
    QPushButton* standButton;
    QPushButton* setMaxVelButton;
    QPushButton* hardStopButton;
    QPushButton* gentleStopButton;
    QPushButton* releaseStopButton;
    QPushButton* stopButton;
    QPushButton* setGaitButton;
    QPushButton* setSwingHeightButton;
    QPushButton* setObstaclePaddingButton;
    QPushButton* setGratedSurfacesButton;
    QPushButton* setFrictionButton;
    QPushButton* allowMotionButton;

    QLabel* linearXLabel;
    QLabel* linearYLabel;
    QLabel* angularZLabel;
    QLabel* statusLabel;
    QLabel* bodyHeightLabel;
    QLabel* rollLabel;
    QLabel* pitchLabel;
    QLabel* yawLabel;
    QLabel* estimatedRuntimeLabel;
    QLabel* batteryStateLabel;
    QLabel* motorStateLabel;
    QLabel* batteryTempLabel;
    QLabel* estopLabel;

    QComboBox* gaitComboBox;
    QComboBox* swingHeightComboBox;
    QComboBox* gratedSurfacesComboBox;

    QDoubleSpinBox* linearXSpin;
    QDoubleSpinBox* linearYSpin;
    QDoubleSpinBox* angularZSpin;
    QDoubleSpinBox* bodyHeightSpin;
    QDoubleSpinBox* rollSpin;
    QDoubleSpinBox* pitchSpin;
    QDoubleSpinBox* yawSpin;
    QDoubleSpinBox* frictionSpin;
    QDoubleSpinBox* obstaclePaddingSpin;

    spot_msgs::MobilityParams _lastMobilityParams;

    bool haveLease;
    bool isEStopped;
    bool motionAllowed;
};

} // end namespace spot_viz

#endif // SPOT_CONTROL_PANEL_H
