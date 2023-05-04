#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include "Tasks/TrackDesiredForceTask.h"

enum holdMode
{
  simpleAdmi,
  forceConstraint
};

struct RobotHolding : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

    void addToGUI(mc_rtc::gui::StateBuilder & gui, mc_control::fsm::Controller & ctl);

    void GUIForceContacts(mc_rtc::gui::StateBuilder & gui, mc_control::fsm::Controller & ctl);
private:

    std::shared_ptr<mc_tasks::force::AdmittanceTask> rightHandAdmittancePtr_, leftHandAdmittancePtr_;

    std::shared_ptr<TrackDesiredForceTask> rightHandForceConstPtr_, leftHandForceConstPtr_;
    int mode_ = simpleAdmi;
    // int RHweight_, LHweight_;
    int weight_, stiffness_;
    
    Eigen::Vector6d RHadmittance_;
    Eigen::Vector6d RHstiffness_;
    Eigen::Vector6d RHdamping_;
    Eigen::Vector3d RHmaxVel_;
    sva::ForceVecd RHwrench_;
    // std::string RHtargetSurf_;

    Eigen::Vector6d LHadmittance_;
    Eigen::Vector6d LHstiffness_;
    Eigen::Vector6d LHdamping_;
    Eigen::Vector3d LHmaxVel_;
    sva::ForceVecd LHwrench_;
    // std::string LHtargetSurf_;
};