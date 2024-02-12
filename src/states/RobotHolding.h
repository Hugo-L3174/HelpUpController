#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/DampingTask.h>
#include <mc_tasks/ImpedanceGains.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

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
  std::shared_ptr<mc_tasks::force::ImpedanceTask> rightHandImpedancePtr_, leftHandImpedancePtr_;
  std::shared_ptr<mc_tasks::force::DampingTask> rightHandDampingPtr_, leftHandDampingPtr_;
  std::shared_ptr<mc_tasks::force::ComplianceTask> rightHandCompliPtr_, leftHandCompliPtr_;
  std::shared_ptr<mc_tasks::force::CoPTask> rightHandCoPTaskPtr_, leftHandCoPTaskPtr_;

  // int RHweight_, LHweight_;
  int weight_, stiffness_;

  Eigen::Vector6d RHadmittance_;
  Eigen::Vector6d RHstiffness_;
  Eigen::Vector6d RHdamping_;
  Eigen::Vector3d RHmaxVel_;
  sva::ForceVecd RHwrench_;
  std::string RHtargetRobot_;
  std::string RHtargetFrame_;
  sva::PTransformd RHtargetOffset_ = sva::PTransformd::Identity();
  mc_tasks::force::ImpedanceGains RHimpGains_;

  Eigen::Vector6d LHadmittance_;
  Eigen::Vector6d LHstiffness_;
  Eigen::Vector6d LHdamping_;
  Eigen::Vector3d LHmaxVel_;
  sva::ForceVecd LHwrench_;
  std::string LHtargetRobot_;
  std::string LHtargetFrame_;
  sva::PTransformd LHtargetOffset_ = sva::PTransformd::Identity();
  mc_tasks::force::ImpedanceGains LHimpGains_;

  // externalwrench gains
  sva::MotionVecd RHgains_, LHgains_ = sva::MotionVecd::Zero();

  // mc_filter::LowPass<sva::ForceVecd> extWrenchRHLowPass_, extWrenchLHLowPass_;
};
