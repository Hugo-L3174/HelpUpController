#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/TransformTask.h>
#include <gram_savitzky_golay/gram_savitzky_golay.h>
#include <gram_savitzky_golay/sva_filters.h>

struct XsensBodyConfiguration
{
  std::string segmentName{};
  std::string bodyName{};
  sva::PTransformd offset = sva::PTransformd::Identity();
};

struct XsensHuman : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::map<std::string, XsensBodyConfiguration> bodyConfigurations_;
  std::map<std::string, std::unique_ptr<mc_tasks::TransformTask>> tasks_;

  std::shared_ptr<gram_sg::MotionVecdFilter> velocityFilter_;

  // std::shared_ptr<gram_sg::SavitzkyGolayFilter> accFilter_;

  double stiffness_ = 10;
  double weight_ = 1000;
  std::string robot_{};
  bool liveMode_ = true;
};
