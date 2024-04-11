#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/MetaTaskLoader.h>

struct RobotTakePoseObserved : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration config_;

  bool scaleCoMHeightBefore_ = true;
  bool scaleCoMLateralBefore_ = true;

  sva::PTransformd RHobjectiveOffset_ = sva::PTransformd::Identity();
  sva::PTransformd LHobjectiveOffset_ = sva::PTransformd::Identity();

  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> RHandTrajectoryTask_;
  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> LHandTrajectoryTask_;

  double errorRH_, errorLH_ = 0;
  // double critRH_, critLH_;
  mc_control::CompletionCriteria critRH_, critLH_;
};
