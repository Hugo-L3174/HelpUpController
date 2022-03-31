#include "HelpUpController_Initial.h"

#include "../HelpUpController.h"

void HelpUpController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HelpUpController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

bool HelpUpController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  output("OK");
  return true;
}

void HelpUpController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

EXPORT_SINGLE_STATE("HelpUpController_Initial", HelpUpController_Initial)
