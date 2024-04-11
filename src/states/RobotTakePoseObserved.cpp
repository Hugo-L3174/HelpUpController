#include "RobotTakePoseObserved.h"

#include <mc_control/fsm/Controller.h>

void RobotTakePoseObserved::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
  config_("scaleCoMHeightBefore", scaleCoMHeightBefore_);
  config_("scaleCoMLateralBefore", scaleCoMLateralBefore_);
}

void RobotTakePoseObserved::start(mc_control::fsm::Controller & ctl)
{
  ctl.datastore().get<bool>("HelpUp::scaleRobotCoMZ") = scaleCoMHeightBefore_;
  ctl.datastore().get<bool>("HelpUp::scaleRobotCoMLateral") = scaleCoMLateralBefore_;
  if(config_.has("RHandTrajectory"))
  {
    RHandTrajectoryTask_ =
        mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(ctl.solver(), config_("RHandTrajectory"));
    // config offset needs to be added to helpup modifs
    if(config_("RHandTrajectory")("targetFrame").has("translation"))
    {
      config_("RHandTrajectory")("targetFrame")("translation", RHobjectiveOffset_.translation());
    }
    if(config_("RHandTrajectory")("targetFrame").has("rotation"))
    {
      config_("RHandTrajectory")("targetFrame")("rotation", RHobjectiveOffset_.rotation());
    }

    critRH_.configure(*RHandTrajectoryTask_, ctl.solver().dt(),
                      config_("RHandTrajectory")("completion", mc_rtc::Configuration{}));
    ctl.solver().addTask(RHandTrajectoryTask_);
  }

  if(config_.has("LHandTrajectory"))
  {
    LHandTrajectoryTask_ =
        mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(ctl.solver(), config_("LHandTrajectory"));
    if(config_("LHandTrajectory")("targetFrame").has("translation"))
    {
      config_("LHandTrajectory")("targetFrame")("translation", LHobjectiveOffset_.translation());
    }
    if(config_("LHandTrajectory")("targetFrame").has("rotation"))
    {
      config_("LHandTrajectory")("targetFrame")("rotation", LHobjectiveOffset_.rotation());
    }

    critLH_.configure(*LHandTrajectoryTask_, ctl.solver().dt(),
                      config_("LHandTrajectory")("completion", mc_rtc::Configuration{}));
    ctl.solver().addTask(LHandTrajectoryTask_);
  }
  // ctl.logger().addLogEntry("bspline_trajectory_hrp4_RightHand_eval", [this]() -> const double { return errorRH_;});
  // ctl.logger().addLogEntry("bspline_trajectory_hrp4_LeftHand_eval", [this]() -> const double { return errorLH_;});
}

bool RobotTakePoseObserved::run(mc_control::fsm::Controller & ctl)
{
  auto RHtargetFrame = config_("RHandTrajectory")("targetFrame");
  auto LHtargetFrame = config_("LHandTrajectory")("targetFrame");

  // get world frame objective
  auto updatedObjectiveRH =
      RHobjectiveOffset_ * ctl.robot(RHtargetFrame("robot")).frame(RHtargetFrame("frame")).position();
  // update offset of objective (transform between control frame and observed frame)
  auto controlledFrameRH = ctl.robot().frame(config_("RHandTrajectory")("frame")).position();
  auto observedFrameRH = ctl.realRobot().frame(config_("RHandTrajectory")("frame")).position();
  auto offsetRH = controlledFrameRH * observedFrameRH.inv();
  // updatedObjectiveRH = offsetRH * updatedObjectiveRH;
  // update task objective
  RHandTrajectoryTask_->target(updatedObjectiveRH);

  // same with left hand
  auto updatedObjectiveLH =
      LHobjectiveOffset_ * ctl.robot(LHtargetFrame("robot")).frame(LHtargetFrame("frame")).position();
  // update offset of objective (transform between control frame and observed frame)
  auto controlledFrameLH = ctl.robot().frame(config_("LHandTrajectory")("frame")).position();
  auto observedFrameLH = ctl.realRobot().frame(config_("LHandTrajectory")("frame")).position();
  auto offsetLH = controlledFrameLH * observedFrameLH.inv();
  // updatedObjectiveLH = offsetLH * updatedObjectiveLH;
  // update task objective
  LHandTrajectoryTask_->target(updatedObjectiveLH);

  // // build criterion using error between real robot frame and objective (not used here)
  errorRH_ = sva::transformError(observedFrameRH, updatedObjectiveRH).vector().norm();
  errorLH_ = sva::transformError(observedFrameLH, updatedObjectiveLH).vector().norm();

  if(critRH_.completed(*RHandTrajectoryTask_) && critLH_.completed(*LHandTrajectoryTask_))
  {
    output("OK");
    return true;
  }
  return false;
}

void RobotTakePoseObserved::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(RHandTrajectoryTask_);
  ctl.solver().removeTask(LHandTrajectoryTask_);
}

EXPORT_SINGLE_STATE("RobotTakePoseObserved", RobotTakePoseObserved)
