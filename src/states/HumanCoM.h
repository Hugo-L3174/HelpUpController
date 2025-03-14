#pragma once

#include <mc_control/fsm/State.h>

struct HumanCoM : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  int weight_;
  Eigen::Vector3d dimWeight_;
  int finalStiff_;
  double z_objective_;
  bool specific_Z_ = false;
};
