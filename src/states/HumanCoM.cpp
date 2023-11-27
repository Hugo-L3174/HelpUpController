#include "HumanCoM.h"

#include "../HelpUpController.h"

void HumanCoM::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
  if(config_.has("weight"))
    weight_ = static_cast<int>(config_("weight"));
  else
    weight_ = 1000;

  if(config_.has("dimWeight"))
    dimWeight_ = config_("dimWeight");
  else
    dimWeight_.setOnes();

  if(config_.has("stiffness"))
    finalStiff_ = config_("stiffness");
  else
    finalStiff_ = 5;

  if(config_.has("z_objective"))
  {
    z_objective_ = config_("z_objective");
    specific_Z_ = true;
  }
}

void HumanCoM::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  auto humCoMTask = ctl.getComTaskHum();
  humCoMTask->weight(weight_);
  humCoMTask->dimWeight(dimWeight_);
  humCoMTask->stiffness(5);

  if(specific_Z_) ctl.override_CoMz = z_objective_;

  ctl.solver().addTask(humCoMTask);
}

bool HumanCoM::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto humCoMTask = ctl.getComTaskHum();

  // if (humCoMTask->eval().norm() <= 0.05) humCoMTask->stiffness(finalStiff_);
  // else humCoMTask->stiffness(5);

  output("OK");
  return true;
}

void HumanCoM::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  ctl.override_CoMz = std::nullopt;

  ctl.solver().removeTask(ctl.getComTaskHum());
}

EXPORT_SINGLE_STATE("HumanCoM", HumanCoM)
