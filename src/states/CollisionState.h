#pragma once

#include <mc_control/fsm/State.h>

struct CollisionState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  // adding/removing collision sets from custom robots configs
  void addRemoveCollisions(mc_control::fsm::Controller & ctl, bool add, bool before);
};
