#include "ResetPoses.h"

#include "../HelpUpController.h"

void ResetPoses::configure(const mc_rtc::Configuration & config)
{
  config("chairOffset", chairOffset_);
  config("robotOffset", robotOffset_);
}

void ResetPoses::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

bool ResetPoses::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto HipsPose = sva::PTransformd::Identity();
  const std::string & segmentName = "Pelvis";
  
  if(ctl.datastore().has("XsensPlugin"))
  {
    HipsPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName);

    // Keeping rotation yaw but setting roll and pitch to zero (basically keeping chair horizontal)
    HipsPose.rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(sva::PTransformd::Identity().rotation(), HipsPose.rotation());
    // Initializing at zero height (easier)
    HipsPose.translation().z() = sva::PTransformd::Identity().translation().z();
    
    if(ctl.robots().hasRobot("chair")){

      // Adjust chair position relative to human model
      ctl.robots().robot("chair").posW(HipsPose * chairOffset_);

      // Adjust main robot position relative to chair
      ctl.robots().robot().posW(ctl.robots().robot("chair").posW() * robotOffset_); 
    }

    output("OK");
    return true;
  }
  else
  {
    mc_rtc::log::info("[{}] Waiting for datastore initialization...", name());
    output("NOK");
    return false;
  }

  
}

void ResetPoses::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

EXPORT_SINGLE_STATE("ResetPoses", ResetPoses)
