#include "HumanCoM.h"

#include "../HelpUpController.h"

void HumanCoM::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
  if (config_.has("weight")) weight_ = static_cast<int>(config_("weight"));
  else weight_ = 1000;

  if (config_.has("dimWeight")) dimWeight_ = config_("dimWeight");
  else dimWeight_.setOnes();
}

void HumanCoM::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  auto humCoMTask = ctl.getComTaskHum();
  humCoMTask->weight(weight_);
  humCoMTask->dimWeight(dimWeight_);

  ctl.solver().addTask(humCoMTask);
}

bool HumanCoM::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  output("OK");
  return true;
}

void HumanCoM::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  ctl.solver().removeTask(ctl.getComTaskHum());
}

EXPORT_SINGLE_STATE("HumanCoM", HumanCoM)
