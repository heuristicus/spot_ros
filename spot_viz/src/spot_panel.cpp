#include "spot_panel.hpp"

#include <algorithm>
#include <cmath>
#include <QFile>
#include <QUiLoader>
#include <QVBoxLayout>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <QDoubleValidator>
#include <QStandardItemModel>
#include <tf/transform_datatypes.h>
#include <spot_msgs/SetVelocity.h>
#include <spot_msgs/LeaseArray.h>
#include <spot_msgs/EStopState.h>
#include <spot_msgs/SetObstacleParams.h>
#include <spot_msgs/SetTerrainParams.h>
#include <spot_msgs/PosedStand.h>
#include <string.h>


namespace spot_viz
{

    ControlPanel::ControlPanel(QWidget *parent) {
        std::string packagePath = ros::package::getPath("spot_viz") + "/resource/spot_control.ui";
        ROS_INFO("Getting ui file from package path %s", packagePath.c_str());
        QFile file(packagePath.c_str());
        file.open(QIODevice::ReadOnly);

        QUiLoader loader;
        QWidget* ui = loader.load(&file, parent);
        file.close();
        QVBoxLayout* topLayout = new QVBoxLayout();
        this->setLayout(topLayout);
        topLayout->addWidget(ui);
        haveLease = false;
        motionAllowed = false;

        sitService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/sit");
        standService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/stand");
        claimLeaseService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/claim");
        releaseLeaseService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/release");
        powerOnService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/power_on");
        powerOffService_ = nh_.serviceClient<std_srvs::Trigger>("spot/power_off");
        maxVelocityService_ = nh_.serviceClient<spot_msgs::SetVelocity>("/spot/velocity_limit");
        hardStopService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/estop/hard");
        gentleStopService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/estop/gentle");
        releaseStopService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/estop/release");
        stopService_ = nh_.serviceClient<std_srvs::Trigger>("/spot/stop");
        gaitService_ = nh_.serviceClient<spot_msgs::SetLocomotion>("/spot/locomotion_mode");
        swingHeightService_ = nh_.serviceClient<spot_msgs::SetSwingHeight>("/spot/swing_height");
        terrainParamsService_ = nh_.serviceClient<spot_msgs::SetTerrainParams>("/spot/terrain_params");
        obstacleParamsService_ = nh_.serviceClient<spot_msgs::SetObstacleParams>("/spot/obstacle_params");
        allowMotionService_ = nh_.serviceClient<std_srvs::SetBool>("/spot/allow_motion");
        bodyPoseService_ = nh_.serviceClient<spot_msgs::PosedStand>("/spot/posed_stand");

        claimLeaseButton = this->findChild<QPushButton*>("claimLeaseButton");
        releaseLeaseButton = this->findChild<QPushButton*>("releaseLeaseButton");
        powerOnButton = this->findChild<QPushButton*>("powerOnButton");
        powerOffButton = this->findChild<QPushButton*>("powerOffButton");
        standButton = this->findChild<QPushButton*>("standButton");
        sitButton = this->findChild<QPushButton*>("sitButton");
        setBodyPoseButton = this->findChild<QPushButton*>("setBodyPoseButton");
        setBodyNeutralButton = this->findChild<QPushButton*>("setBodyNeutralButton");
        setMaxVelButton = this->findChild<QPushButton*>("setMaxVelButton");
        setGaitButton = this->findChild<QPushButton*>("setGaitButton");
        setSwingHeightButton = this->findChild<QPushButton*>("setSwingHeightButton");
        setObstaclePaddingButton = this->findChild<QPushButton*>("setObstaclePaddingButton");
        setGratedSurfacesButton = this->findChild<QPushButton*>("setGratedSurfacesButton");
        setFrictionButton = this->findChild<QPushButton*>("setFrictionButton");
        allowMotionButton = this->findChild<QPushButton*>("allowMotionButton");

        statusLabel = this->findChild<QLabel*>("statusLabel");
        estimatedRuntimeLabel = this->findChild<QLabel*>("estimatedRuntimeLabel");
        batteryStateLabel = this->findChild<QLabel*>("batteryStateLabel");
        motorStateLabel = this->findChild<QLabel*>("motorStateLabel");
        batteryTempLabel = this->findChild<QLabel*>("batteryTempLabel");
        estopLabel = this->findChild<QLabel*>("estopLabel");

        gaitComboBox = this->findChild<QComboBox*>("gaitComboBox");
        swingHeightComboBox = this->findChild<QComboBox*>("swingHeightComboBox");
        gratedSurfacesComboBox = this->findChild<QComboBox*>("gratedSurfacesComboBox");

        obstaclePaddingSpin = this->findChild<QDoubleSpinBox*>("obstaclePaddingSpin");
        frictionSpin = this->findChild<QDoubleSpinBox*>("frictionSpin");

        setupComboBoxes();
        setupStopButtons();
        setupSpinBoxes();

        // Subscribe to things after everything is set up to avoid crashes when things aren't initialised
        leaseSub_ = nh_.subscribe("/spot/status/leases", 1, &ControlPanel::leaseCallback, this);
        estopSub_ = nh_.subscribe("/spot/status/estop", 1, &ControlPanel::estopCallback, this);
        mobilityParamsSub_ = nh_.subscribe("/spot/status/mobility_params", 1, &ControlPanel::mobilityParamsCallback, this);
        batterySub_ = nh_.subscribe("/spot/status/battery_states", 1, &ControlPanel::batteryCallback, this);
        powerSub_ = nh_.subscribe("/spot/status/power_state", 1, &ControlPanel::powerCallback, this);
        motionAllowedSub_ = nh_.subscribe("/spot/status/motion_allowed", 1, &ControlPanel::motionAllowedCallback, this);

        connect(claimLeaseButton, SIGNAL(clicked()), this, SLOT(claimLease()));
        connect(releaseLeaseButton, SIGNAL(clicked()), this, SLOT(releaseLease()));
        connect(powerOnButton, SIGNAL(clicked()), this, SLOT(powerOn()));
        connect(powerOffButton, SIGNAL(clicked()), this, SLOT(powerOff()));
        connect(sitButton, SIGNAL(clicked()), this, SLOT(sit()));
        connect(standButton, SIGNAL(clicked()), this, SLOT(stand()));
        connect(setBodyPoseButton, SIGNAL(clicked()), this, SLOT(sendBodyPose()));
        connect(setBodyNeutralButton, SIGNAL(clicked()), this, SLOT(sendNeutralBodyPose()));
        connect(setMaxVelButton, SIGNAL(clicked()), this, SLOT(setMaxVel()));
        connect(releaseStopButton, SIGNAL(clicked()), this, SLOT(releaseStop()));
        connect(hardStopButton, SIGNAL(clicked()), this, SLOT(hardStop()));
        connect(gentleStopButton, SIGNAL(clicked()), this, SLOT(gentleStop()));
        connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
        connect(setGaitButton, SIGNAL(clicked()), this, SLOT(setGait()));
        connect(setSwingHeightButton, SIGNAL(clicked()), this, SLOT(setSwingHeight()));
        connect(setObstaclePaddingButton, SIGNAL(clicked()), this, SLOT(setObstacleParams()));
        connect(setGratedSurfacesButton, SIGNAL(clicked()), this, SLOT(setTerrainParams()));
        connect(setFrictionButton, SIGNAL(clicked()), this, SLOT(setTerrainParams()));
        connect(allowMotionButton, SIGNAL(clicked()), this, SLOT(allowMotion()));
    }

    void ControlPanel::setupStopButtons() {
        stopButton = this->findChild<QPushButton*>("stopButton");
        QPalette pal = stopButton->palette();
        pal.setColor(QPalette::Button, QColor(255, 165, 0));
        stopButton->setAutoFillBackground(true);
        stopButton->setPalette(pal);
        stopButton->update();

        gentleStopButton = this->findChild<QPushButton*>("gentleStopButton");
        pal = gentleStopButton->palette();
        pal.setColor(QPalette::Button, QColor(255, 0, 255));
        gentleStopButton->setAutoFillBackground(true);
        gentleStopButton->setPalette(pal);
        gentleStopButton->update();

        hardStopButton = this->findChild<QPushButton*>("hardStopButton");
        hardStopButton->setText(QString::fromUtf8("\u26A0 Kill Motors"));
        pal = hardStopButton->palette();
        pal.setColor(QPalette::Button, QColor(255, 0, 0));
        hardStopButton->setAutoFillBackground(true);
        hardStopButton->setPalette(pal);
        hardStopButton->update();

        releaseStopButton = this->findChild<QPushButton*>("releaseStopButton");
        pal = releaseStopButton->palette();
        pal.setColor(QPalette::Button, QColor(0, 255, 0));
        releaseStopButton->setAutoFillBackground(true);
        releaseStopButton->setPalette(pal);
        releaseStopButton->update();
    }

    void ControlPanel::setupComboBoxes() {
        // Iterate over the map for this combobox and add items. By default items in the
        // map are in ascending order by key
        for (const auto& item : gaitMap) {
            gaitComboBox->addItem(QString(item.second.c_str()));
        }
        // Disable the unknown entry in the combobox so that it cannot be selected and sent to the service
        QStandardItemModel* model = qobject_cast<QStandardItemModel *>(gaitComboBox->model());
        QStandardItem* item = model->item(0);
        item->setFlags(item->flags() & ~Qt::ItemIsEnabled);

        for (const auto& item : swingHeightMap) {
            swingHeightComboBox->addItem(QString(item.second.c_str()));
        }
        model = qobject_cast<QStandardItemModel *>(swingHeightComboBox->model());
        item = model->item(0);
        item->setFlags(item->flags() & ~Qt::ItemIsEnabled);

        for (const auto& item : gratedSurfacesMap) {
            gratedSurfacesComboBox->addItem(QString(item.second.c_str()));
        }
        model = qobject_cast<QStandardItemModel *>(gratedSurfacesComboBox->model());
        item = model->item(0);
        item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }

    void ControlPanel::setupSpinBoxes() {
        double linearVelocityLimit = 2;
        linearXSpin = this->findChild<QDoubleSpinBox*>("linearXSpin");
        linearXLabel = this->findChild<QLabel*>("linearXLabel");
        updateLabelTextWithLimit(linearXLabel, 0, linearVelocityLimit);
        linearXSpin->setMaximum(linearVelocityLimit);
        linearXSpin->setMinimum(0);

        linearYSpin = this->findChild<QDoubleSpinBox*>("linearYSpin");
        linearYLabel = this->findChild<QLabel*>("linearYLabel");
        updateLabelTextWithLimit(linearYLabel, 0, linearVelocityLimit);
        linearYSpin->setMaximum(linearVelocityLimit);
        linearYSpin->setMinimum(0);

        angularZSpin = this->findChild<QDoubleSpinBox*>("angularZSpin");
        angularZLabel = this->findChild<QLabel*>("angularZLabel");
        updateLabelTextWithLimit(angularZLabel, 0, linearVelocityLimit);
        angularZSpin->setMaximum(linearVelocityLimit);
        angularZSpin->setMinimum(0);

        double bodyHeightLimit = 0.15;
        bodyHeightSpin = this->findChild<QDoubleSpinBox*>("bodyHeightSpin");
        bodyHeightLabel = this->findChild<QLabel*>("bodyHeightLabel");
        updateLabelTextWithLimit(bodyHeightLabel, -bodyHeightLimit, bodyHeightLimit);
        bodyHeightSpin->setMaximum(bodyHeightLimit);
        bodyHeightSpin->setMinimum(-bodyHeightLimit);

        double rollLimit = 20;
        rollSpin = this->findChild<QDoubleSpinBox*>("rollSpin");
        rollLabel = this->findChild<QLabel*>("rollLabel");
        updateLabelTextWithLimit(rollLabel, -rollLimit, rollLimit);
        rollSpin->setMaximum(rollLimit);
        rollSpin->setMinimum(-rollLimit);

        double pitchLimit = 30;
        pitchSpin = this->findChild<QDoubleSpinBox*>("pitchSpin");
        pitchLabel = this->findChild<QLabel*>("pitchLabel");
        updateLabelTextWithLimit(pitchLabel, -pitchLimit, pitchLimit);
        pitchSpin->setMaximum(pitchLimit);
        pitchSpin->setMinimum(-pitchLimit);

        double yawLimit = 30;
        yawSpin = this->findChild<QDoubleSpinBox*>("yawSpin");
        yawLabel = this->findChild<QLabel*>("yawLabel");
        updateLabelTextWithLimit(yawLabel, -yawLimit, yawLimit);
        yawSpin->setMaximum(yawLimit);
        yawSpin->setMinimum(-yawLimit);
    }

    void ControlPanel::updateLabelTextWithLimit(QLabel* label, double limit_lower, double limit_upper) {
        int precision = 1;
        // Kind of hacky but default to_string returns 6 digit precision which is unnecessary
        std::string limit_lower_value = std::to_string(limit_lower).substr(0, std::to_string(limit_lower).find(".") + precision + 1);
        std::string limit_upper_value = std::to_string(limit_upper).substr(0, std::to_string(limit_upper).find(".") + precision + 1);
        std::string limit_range = " [" + limit_lower_value + ", " + limit_upper_value + "]";
        std::string current_text = label->text().toStdString();
        label->setText(QString((current_text + limit_range).c_str()));
    }

    void ControlPanel::setControlButtons() {
        claimLeaseButton->setEnabled(!haveLease);
        releaseLeaseButton->setEnabled(haveLease);
        powerOnButton->setEnabled(haveLease);
        powerOffButton->setEnabled(haveLease);
        sitButton->setEnabled(haveLease);
        standButton->setEnabled(haveLease);
        setBodyPoseButton->setEnabled(haveLease);
        setBodyNeutralButton->setEnabled(haveLease);
        setMaxVelButton->setEnabled(haveLease);
        releaseStopButton->setEnabled(haveLease && isEStopped);
        hardStopButton->setEnabled(haveLease);
        gentleStopButton->setEnabled(haveLease);
        stopButton->setEnabled(haveLease);
        setGaitButton->setEnabled(haveLease);
        setSwingHeightButton->setEnabled(haveLease);
        setObstaclePaddingButton->setEnabled(haveLease);
        setFrictionButton->setEnabled(haveLease);
        setGratedSurfacesButton->setEnabled(haveLease);
        allowMotionButton->setEnabled(haveLease);
    }

    /**
     * @brief Call a ros std_msgs/Trigger service
     *
     * Modifies the status label text depending on the result
     *
     * @param service Service to call
     * @param serviceName Name of the service to use in labels
     * @return true if successfully called
     * @return false otherwise
     */
    bool ControlPanel::callTriggerService(ros::ServiceClient service, std::string serviceName) {
        std_srvs::Trigger req;
        return callCustomTriggerService(service, serviceName, req);
    }

    template <typename T>
    /**
     * @brief Call an arbitrary service which has a response type of bool, str
     *
     * Modifies the status label text depending on the result
     *
     * @param service Service to call
     * @param serviceName Name of the service to use in labels
     * @param serviceRequest Request to make to the service
     * @return true if successfully called
     * @return false otherwise
     */
    bool ControlPanel::callCustomTriggerService(ros::ServiceClient service, std::string serviceName, T serviceRequest) {
        std::string labelText = "Calling " + serviceName + " service";
        statusLabel->setText(QString(labelText.c_str()));
        if (service.call(serviceRequest)) {
            if (serviceRequest.response.success) {
                labelText = "Successfully called " + serviceName + " service";
                statusLabel->setText(QString(labelText.c_str()));
                return true;
            } else {
                labelText = serviceName + " service failed: " + serviceRequest.response.message;
                statusLabel->setText(QString(labelText.c_str()));
                return false;
            }
        } else {
            labelText = "Failed to call " + serviceName + " service" + serviceRequest.response.message;
            statusLabel->setText(QString(labelText.c_str()));
            return false;
        }
    }

    /**
     * @brief Check held leases and enable or disable buttons accordingly
     *
     * check to see if the body is already owned by the ROS node
     * the resource will be "body" and the lease_owner.client_name will begin with "ros_spot"
     * if the claim exists, treat this as a successful click of the Claim button
     * if the claim does not exist, treat this as a click of the Release button
     *
     * @param leases
     */
    void ControlPanel::leaseCallback(const spot_msgs::LeaseArray::ConstPtr &leases) {
        bool msg_has_lease = false;
        for (int i=leases->resources.size()-1; i>=0; i--) {
            const spot_msgs::LeaseResource &resource = leases->resources[i];
            bool right_resource = resource.resource.compare("body") == 0;
            bool owned_by_ros = resource.lease_owner.client_name.compare(0, 8, "ros_spot") == 0;

            if (right_resource && owned_by_ros) {
                msg_has_lease = true;
            }
        }

        if (msg_has_lease != haveLease) {
            haveLease = msg_has_lease;
            setControlButtons();
        }
    }

    /**
     * @brief Check estop state and disable control buttons if estopped
     *
     * @param estops
     */
    void ControlPanel::estopCallback(const spot_msgs::EStopStateArray::ConstPtr &estops) {
        bool softwareEstopped = false;
        bool estopped = false;
        std::string estopString("E-stops:");
        for (const auto& estop : estops->estop_states) {
            // Can't release hardware estops from the sdk
            bool stopped = false;
            if (estop.state == spot_msgs::EStopState::STATE_ESTOPPED) {
                if (estop.type == spot_msgs::EStopState::TYPE_SOFTWARE) {
                    softwareEstopped = true;
                }
                stopped = true;
            }
            std::string stoppedStr(stopped ? "On" : "Off");
            if (estop.name == "hardware_estop") {
                estopString += " hardware: " + stoppedStr;
            } else if (estop.name == "payload_estop") {
                estopString += " payload: " + stoppedStr;
            } else if (estop.name == "software_estop") {
                estopString += " software: " + stoppedStr;
            } else {
                estopString += " " + estop.name + ": " + stoppedStr;
            }
        }

        estopLabel->setText(QString(estopString.c_str()));

        if (softwareEstopped != isEStopped) {
            isEStopped = softwareEstopped;
            setControlButtons();
        }
    }

    void ControlPanel::mobilityParamsCallback(const spot_msgs::MobilityParams::ConstPtr &params) {
        if (*params == _lastMobilityParams) {
            // If we don't check this, the user will never be able to modify values since they will constantly reset
            return;
        }

        linearXSpin->setValue(params->velocity_limit.linear.x);
        linearYSpin->setValue(params->velocity_limit.linear.y);
        angularZSpin->setValue(params->velocity_limit.angular.z);

        // Set the combo box values depending on whether there is a nonzero value coming from the params. If there isn't,
        // set the value based on what it is when using the controller
        if (params->locomotion_hint > 0) {
            gaitComboBox->setCurrentIndex(gaitComboBox->findText(gaitMap.at(params->locomotion_hint).c_str()));
        } else {
            gaitComboBox->setCurrentIndex(gaitComboBox->findText(gaitMap.at(spot_msgs::SetLocomotion::Request::HINT_AUTO).c_str()));
        }
        if (params->swing_height > 0) {
            swingHeightComboBox->setCurrentIndex(swingHeightComboBox->findText(swingHeightMap.at(params->swing_height).c_str()));
        } else {
            swingHeightComboBox->setCurrentIndex(swingHeightComboBox->findText(swingHeightMap.at(spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_MEDIUM).c_str()));
        }
        if (params->terrain_params.grated_surfaces_mode > 0) {
            gratedSurfacesComboBox->setCurrentIndex(gratedSurfacesComboBox->findText(gratedSurfacesMap.at(params->terrain_params.grated_surfaces_mode).c_str()));
        } else {
            gratedSurfacesComboBox->setCurrentIndex(gratedSurfacesComboBox->findText(gratedSurfacesMap.at(spot_msgs::TerrainParams::GRATED_SURFACES_MODE_AUTO).c_str()));
        }

        _lastMobilityParams = *params;
    }

    void ControlPanel::batteryCallback(const spot_msgs::BatteryStateArray::ConstPtr &battery) {
        spot_msgs::BatteryState battState = battery->battery_states[0];
        std::string estRuntime = "Estimated runtime: " + std::to_string(battState.estimated_runtime.sec/60) + " min";
        estimatedRuntimeLabel->setText(QString(estRuntime.c_str()));

        auto temps = battState.temperatures;
        if (!temps.empty()) {
            auto minmax = std::minmax_element(temps.begin(), temps.end());
            float total = std::accumulate(temps.begin(), temps.end(), 0);
            // Don't care about float values here
            int tempMin = *minmax.first;
            int tempMax = *minmax.second;
            int tempAvg = total / temps.size();
            std::string battTemp = "Battery temp: min " + std::to_string(tempMin) + ", max " + std::to_string(tempMax) + ", avg " + std::to_string(tempAvg);
            batteryTempLabel->setText(QString(battTemp.c_str()));
        } else {
            batteryTempLabel->setText(QString("Battery temp: No battery"));
        }

        std::string status;
        switch (battState.status)
        {
        case spot_msgs::BatteryState::STATUS_UNKNOWN:
            status = "Unknown";
            break;
        case spot_msgs::BatteryState::STATUS_MISSING:
            status = "Missing";
            break;
        case spot_msgs::BatteryState::STATUS_CHARGING:
            status = "Charging";
            break;
        case spot_msgs::BatteryState::STATUS_DISCHARGING:
            status = "Discharging";
            break;
        case spot_msgs::BatteryState::STATUS_BOOTING:
            status = "Booting";
            break;
        default:
            status = "Invalid";
            break;
        }

        std::string battStatusStr;
        if (battState.status == spot_msgs::BatteryState::STATUS_CHARGING || battState.status == spot_msgs::BatteryState::STATUS_DISCHARGING) {
            // TODO: use std::format in c++20 rather than this nastiness
            std::stringstream stream;
            stream << std::fixed << std::setprecision(0) << battState.charge_percentage;
            std::string pct = stream.str() + "%";
            stream.str("");
            stream.clear();
            stream << std::fixed << std::setprecision(1) << battState.voltage;
            std::string volt = stream.str() + "V";
            stream.str("");
            stream.clear();
            stream << battState.current;
            std::string amp = stream.str() + "A";
            battStatusStr = "Battery state: " + status + ", " + pct + ", " + volt + ", " + amp;
        } else {
            battStatusStr = "Battery state: " + status;
        }
        batteryStateLabel->setText(QString(battStatusStr.c_str()));
    }

    void ControlPanel::powerCallback(const spot_msgs::PowerState::ConstPtr &power) {
        std::string state;
        switch (power->motor_power_state)
        {
        case spot_msgs::PowerState::STATE_POWERING_ON:
            state = "Powering on";
            powerOnButton->setEnabled(false);
            break;
        case spot_msgs::PowerState::STATE_POWERING_OFF:
            state = "Powering off";
            powerOffButton->setEnabled(false);
            sitButton->setEnabled(false);
            standButton->setEnabled(false);
            break;
        case spot_msgs::PowerState::STATE_ON:
            state = "On";
            powerOnButton->setEnabled(false);
            powerOffButton->setEnabled(true);
            sitButton->setEnabled(true);
            standButton->setEnabled(true);
            break;
        case spot_msgs::PowerState::STATE_OFF:
            state = "Off";
            powerOnButton->setEnabled(true && haveLease);
            powerOffButton->setEnabled(false);
            sitButton->setEnabled(false);
            standButton->setEnabled(false);
            break;
        case spot_msgs::PowerState::STATE_ERROR:
            state = "Error";
            break;
        case spot_msgs::PowerState::STATE_UNKNOWN:
            state = "Unknown";
            break;
        default:
            "Invalid";
        }
        std::string motorState = "Motor state: " + state;
        motorStateLabel->setText(QString(motorState.c_str()));
    }

    void ControlPanel::motionAllowedCallback(const std_msgs::Bool &motion_allowed) {
        motionAllowed = motion_allowed.data;
        if (!motion_allowed.data) {
            stopButton->setText("Motion is disallowed");
            stopButton->setEnabled(false);
            allowMotionButton->setText("Allow motion");
        } else {
            stopButton->setText("Stop");
            if (haveLease) {
                stopButton->setEnabled(true);
            }
            allowMotionButton->setText("Disallow motion");
        }
    }

    void ControlPanel::sit() {
        callTriggerService(sitService_, "sit");
    }

    void ControlPanel::stand() {
        callTriggerService(standService_, "stand");
    }

    void ControlPanel::powerOn() {
        callTriggerService(powerOnService_, "power on");
    }

    void ControlPanel::powerOff() {
        callTriggerService(powerOffService_, "power off");
    }

    void ControlPanel::claimLease() {
        if (callTriggerService(claimLeaseService_, "claim lease"))
            claimLeaseButton->setEnabled(false);
    }

    void ControlPanel::releaseLease() {
        if (callTriggerService(releaseLeaseService_, "release lease"))
            releaseLeaseButton->setEnabled(false);
    }

    void ControlPanel::stop() {
        callTriggerService(stopService_, "stop");
    }

    void ControlPanel::allowMotion() {
        std_srvs::SetBool req;
        req.request.data = !motionAllowed;
        callCustomTriggerService(allowMotionService_, "set motion allowed", req);
    }

    void ControlPanel::hardStop() {
        callTriggerService(hardStopService_, "hard stop");
    }

    void ControlPanel::gentleStop() {
        callTriggerService(gentleStopService_, "gentle stop");
    }

    void ControlPanel::releaseStop() {
        callTriggerService(releaseStopService_, "release stop");
    }

    void ControlPanel::setMaxVel() {
        spot_msgs::SetVelocity req;
        req.request.velocity_limit.angular.z = angularZSpin->value();
        req.request.velocity_limit.linear.x = linearXSpin->value();
        req.request.velocity_limit.linear.y = linearYSpin->value();
        callCustomTriggerService(maxVelocityService_, "set velocity limits", req);
    }

    void ControlPanel::sendBodyPose() {
        ROS_INFO("Sending body pose");
        spot_msgs::PosedStand req;
        req.request.body_height = bodyHeightSpin->value();
        req.request.body_yaw = yawSpin->value();
        req.request.body_pitch = pitchSpin->value();
        req.request.body_roll = rollSpin->value();

        callCustomTriggerService(bodyPoseService_, "set body pose", req);
    }

    void ControlPanel::sendNeutralBodyPose() {
        ROS_INFO("Sending body neutral pose");
        spot_msgs::PosedStand req;
        callCustomTriggerService(bodyPoseService_, "set body pose", req);
    }

    /**
     * @brief Get the message constant integer that corresponds to the currently selected combobox item
     *
     * @param comboBox Combobox whose selection should be checked
     * @param comboBoxMap Mapping from message constants to text in the combobox
     * @return int > 0 indicating the map constant, or -1 if it couldn't be found
     */
    int comboBoxSelectionToMessageConstantInt(QComboBox* comboBox, const std::map<uint, std::string>& comboBoxMap) {
        std::string selectionText = comboBox->currentText().toStdString();
        for (const auto& item : comboBoxMap) {
            if (selectionText == item.second) {
                return item.first;
            }
        }
        return -1;
    }

    void ControlPanel::setGait() {
        spot_msgs::SetLocomotion req;
        req.request.locomotion_mode = comboBoxSelectionToMessageConstantInt(gaitComboBox, gaitMap);
        callCustomTriggerService(gaitService_, "set gait", req);
    }

    void ControlPanel::setSwingHeight() {
        spot_msgs::SetSwingHeight req;
        req.request.swing_height = comboBoxSelectionToMessageConstantInt(swingHeightComboBox, swingHeightMap);
        callCustomTriggerService(swingHeightService_, "set swing height", req);
    }

    void ControlPanel::setTerrainParams() {
        spot_msgs::SetTerrainParams req;
        req.request.terrain_params.grated_surfaces_mode = comboBoxSelectionToMessageConstantInt(gratedSurfacesComboBox, gratedSurfacesMap);
        req.request.terrain_params.ground_mu_hint = frictionSpin->value();
        callCustomTriggerService(terrainParamsService_, "set terrain params", req);
    }

    void ControlPanel::setObstacleParams() {
        spot_msgs::SetObstacleParams req;
        req.request.obstacle_params.obstacle_avoidance_padding = obstaclePaddingSpin->value();
        callCustomTriggerService(obstacleParamsService_, "set obstacle params", req);
    }

    void ControlPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    // Load all configuration data for this panel from the given Config object.
    void ControlPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }
} // end namespace spot_viz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spot_viz::ControlPanel, rviz::Panel)
// END_TUTORIAL
