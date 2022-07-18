#include "RobotHolding.h"

#include "../HelpUpController.h"

void RobotHolding::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void RobotHolding::start(mc_control::fsm::Controller & ctl_)
{
  if (config_.has("weight")) weight_ = static_cast<int>(config_("weight"));
  else weight_ = 1000;

  if (config_.has("stiffness")) stiffness_ = static_cast<int>(config_("stiffness"));
  else stiffness_ = 5;



  // if (config_.has("dimWeight")) dimWeight_ = config_("dimWeight");
  // else dimWeight_.setOnes();
  auto RHadmiConf = config_("AdmittanceRHand");

  RHadmittance_ = RHadmiConf("admittance");
  RHstiffness_ = RHadmiConf("stiffness");
  RHdamping_ = RHadmiConf("damping");
  RHmaxVel_ = RHadmiConf("maxVel");
  RHwrench_ = RHadmiConf("wrench");

  auto LHadmiConf = config_("AdmittanceLHand");  

  LHadmittance_ = LHadmiConf("admittance");
  LHstiffness_ = LHadmiConf("stiffness");
  LHdamping_ = LHadmiConf("damping");
  LHmaxVel_ = LHadmiConf("maxVel");
  LHwrench_ = LHadmiConf("wrench");
  
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  // right hand admittance
  rightHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("RightHand", ctl.robots(), ctl.robots().robotIndex());
  auto admittance = sva::ForceVec(RHadmittance_);
  rightHandAdmittancePtr_->admittance(admittance);

  auto stiffness = sva::MotionVec(RHstiffness_);
  auto damping = sva::MotionVec(RHdamping_);
  rightHandAdmittancePtr_->setGains(stiffness, damping);

  rightHandAdmittancePtr_->maxLinearVel(RHmaxVel_);

  auto wrench = sva:: ForceVec(RHwrench_);
  rightHandAdmittancePtr_->targetWrench(wrench);

  // setting right hand target surface
  // auto target = ctl.realRobot("human").surfacePose("RightShoulder");
  auto target = ctl.robot("human").surfacePose("RightShoulder");
  rightHandAdmittancePtr_->targetPose(target);


  // left hand admittance
  leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("LeftHand", ctl.robots(), ctl.robots().robotIndex());
  admittance = sva::ForceVec(LHadmittance_);
  leftHandAdmittancePtr_->admittance(admittance);

  stiffness = sva::MotionVec(LHstiffness_);
  damping = sva::MotionVec(LHdamping_);
  leftHandAdmittancePtr_->setGains(stiffness, damping);

  leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);

  wrench = sva:: ForceVec(LHwrench_);
  leftHandAdmittancePtr_->targetWrench(wrench);

  // setting left hand target surface
  // target = ctl.realRobot("human").surfacePose("Back");
  target = ctl.robot("human").surfacePose("Back");
  leftHandAdmittancePtr_->targetPose(target);


  // rightHandAdmittancePtr_->weight(weight_);
  // leftHandAdmittancePtr_->weight(weight_);

  // rightHandAdmittancePtr_->stiffness(stiffness_);
  // leftHandAdmittancePtr_->stiffness(stiffness_);

  // adding tasks to solver
  ctl.solver().addTask(rightHandAdmittancePtr_);
  ctl.solver().addTask(leftHandAdmittancePtr_);
}

bool RobotHolding::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  
  // updating target positions each iteration
  // auto target = ctl.realRobot("human").surfacePose("RightShoulder");
  auto target = ctl.realRobot("human").surfacePose("RightShoulder");
  rightHandAdmittancePtr_->targetPose(target);

  // target = ctl.realRobot("human").surfacePose("Back");
  target = ctl.realRobot("human").surfacePose("Back");
  leftHandAdmittancePtr_->targetPose(target);


  output("OK");
  return true;
}

void RobotHolding::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  ctl.solver().removeTask(rightHandAdmittancePtr_);
  ctl.solver().removeTask(leftHandAdmittancePtr_);
}


EXPORT_SINGLE_STATE("RobotHolding", RobotHolding)
