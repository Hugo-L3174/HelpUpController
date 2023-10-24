#include "RobotTakePoseObserved.h"

#include "../HelpUpController.h"

void RobotTakePoseObserved::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void RobotTakePoseObserved::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  RHandTrajectoryTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(ctl.solver(), config_("RHandTrajectory"));
  LHandTrajectoryTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(ctl.solver(), config_("LHandTrajectory"));

  ctl.solver().addTask(RHandTrajectoryTask_);
  ctl.solver().addTask(LHandTrajectoryTask_);
  // ctl.logger().addLogEntry("bspline_trajectory_hrp4_LeftHandFlat", [this]() -> const sva::PTransformd { return balanceCompPoint_->computationTime();});
}

bool RobotTakePoseObserved::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto RHtargetFrame = config_("RHandTrajectory")("targetFrame");
  auto LHtargetFrame = config_("LHandTrajectory")("targetFrame");

  // get world frame objective
  auto updatedObjectiveRH = ctl.robot(RHtargetFrame("robot")).frame(RHtargetFrame("frame")).position();
  // update offset of objective (transform between control frame and observed frame)
  auto controlledFrameRH = ctl.robot(config_("RHandTrajectory")("robot")).frame(config_("RHandTrajectory")("frame")).position();
  auto observedFrameRH = ctl.realRobot(config_("RHandTrajectory")("robot")).frame(config_("RHandTrajectory")("frame")).position();
  // controlledFrameRH.inv() * observedFrameRH;
  auto offsetRH = controlledFrameRH * observedFrameRH.inv();
  mc_rtc::log::info("Error between controlled and observed RH:");
  // mc_rtc::log::info("rotation: {}", controlledFrameRH.translation() - observedFrameRH.translation());
  mc_rtc::log::info("translation: {}", controlledFrameRH.translation() - observedFrameRH.translation());
  mc_rtc::log::info("Prev objective RH was {}", updatedObjectiveRH.translation().transpose());
  updatedObjectiveRH = updatedObjectiveRH * offsetRH;
  mc_rtc::log::info("New objective RH is {}", updatedObjectiveRH.translation().transpose());
  // update task objective
  RHandTrajectoryTask_->target(updatedObjectiveRH);

  // same with left hand
  auto updatedObjectiveLH = ctl.robot(LHtargetFrame("robot")).frame(LHtargetFrame("frame")).position();
  // update offset of objective (transform between control frame and observed frame)
  auto controlledFrameLH = ctl.robot(config_("LHandTrajectory")("robot")).frame(config_("LHandTrajectory")("frame")).position();
  auto observedFrameLH = ctl.realRobot(config_("LHandTrajectory")("robot")).frame(config_("LHandTrajectory")("frame")).position();
  // controlledFrameLH.inv() * observedFrameLH;
  auto offsetLH = controlledFrameLH * observedFrameLH.inv();
  mc_rtc::log::info("Error between controlled and observed LH: ");
  mc_rtc::log::info("translation: {}", controlledFrameLH.translation() - observedFrameLH.translation());
  mc_rtc::log::info("Prev objective LH was {}", updatedObjectiveLH.translation().transpose());
  updatedObjectiveLH = updatedObjectiveLH * offsetLH;
  mc_rtc::log::info("New objective LH is {}", updatedObjectiveLH.translation().transpose());
  // update task objective
  LHandTrajectoryTask_->target(updatedObjectiveLH);
  
  // output("OK");
  return false;
}

void RobotTakePoseObserved::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

EXPORT_SINGLE_STATE("RobotTakePoseObserved", RobotTakePoseObserved)
