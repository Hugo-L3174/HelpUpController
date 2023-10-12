#include "ChangeBalanceConfig.h"

#include "../HelpUpController.h"

void ChangeBalanceConfig::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
  if(config_.has("completion"))
  {
    hasCompletion_ = !config("completion").empty();
    config("completion")("dcmEval", dcmThreshold_);
    
  }
  mc_rtc::log::info("there is a completion criteria: {}", hasCompletion_);
  mc_rtc::log::info("dcm objective: {}", dcmThreshold_.transpose()); 
  if(config_.has("StabilizerConfig")) 
  {
    StabilizerConfig_ = config_("StabilizerConfig");
  }
  
}

void ChangeBalanceConfig::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto stabTask = ctl.datastore().call<std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask>>("RobotStabilizer::getTask");

  ctl.datastore().call<void, Eigen::Vector3d, bool>("RobotStabilizer::setDCMThreshold", dcmThreshold_, hasCompletion_);

  if(config_.has("above"))
  {
    ctl.datastore().call("RobotStabilizer::setAboveObjective", config_("above"));
  }

  if (config_.has("StabilizerConfig"))
  {
    contactState_.clear();
    const auto & contacts = StabilizerConfig_("contacts");
    for (auto contactName : contacts)
    {
      ContactState s = contactName;
      contactState_.push_back(s);
      mc_rtc::log::info("contact added: {}", s);
    }
    
    stabTask->setContacts(contactState_);
    // stabTask->updateContacts();
  }
  // stabTask->configure()
  
  

  
}

bool ChangeBalanceConfig::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto finished = ctl.datastore().call<bool>("RobotStabilizer::isBalanced");
  if (finished)
  {
    output("OK");
    return true;
  }

  return false;
}

void ChangeBalanceConfig::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

}

EXPORT_SINGLE_STATE("ChangeBalanceConfig", ChangeBalanceConfig)