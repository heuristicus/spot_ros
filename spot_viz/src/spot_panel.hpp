#ifndef SPOT_CONTROL_PANEL_H
#define SPOT_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <spot_msgs/LeaseArray.h>

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
    void setMaxVel();

    private:

    void setControlButtons();
    void toggleBodyPoseButtons();
    bool callTriggerService(ros::ServiceClient service, std::string serviceName);
    void updateLabelTextWithLimit(QLabel* label, double limit);
    void leaseCallback(const spot_msgs::LeaseArray::ConstPtr &leases);

    ros::NodeHandle nh_;
    ros::ServiceClient sitService_;
    ros::ServiceClient standService_;
    ros::ServiceClient claimLeaseService_;
    ros::ServiceClient releaseLeaseService_;
    ros::ServiceClient powerOnService_;
    ros::ServiceClient powerOffService_;
    ros::ServiceClient maxVelocityService_;
    ros::Publisher bodyPosePub_;
    ros::Subscriber leaseSub_;

    QPushButton* claimLeaseButton;
    QPushButton* releaseLeaseButton;
    QPushButton* powerOnButton;
    QPushButton* powerOffButton;
    QPushButton* setBodyPoseButton;
    QPushButton* sitButton;
    QPushButton* standButton;
    QPushButton* setMaxVelButton;
    QLabel* linearXLabel;
    QLabel* linearYLabel;
    QLabel* angularZLabel;
    QLabel* statusLabel;
    QLabel* bodyHeightLabel;
    QLabel* rollLabel;
    QLabel* pitchLabel;
    QLabel* yawLabel;
    QDoubleSpinBox* linearXSpin;
    QDoubleSpinBox* linearYSpin;
    QDoubleSpinBox* angularZSpin;
    QDoubleSpinBox* bodyHeightSpin;
    QDoubleSpinBox* rollSpin;
    QDoubleSpinBox* pitchSpin;
    QDoubleSpinBox* yawSpin;

    bool haveLease;
};

} // end namespace spot_viz

#endif // SPOT_CONTROL_PANEL_H
