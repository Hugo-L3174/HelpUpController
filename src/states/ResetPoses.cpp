#include "ResetPoses.h"

#include "../HelpUpController.h"

void ResetPoses::configure(const mc_rtc::Configuration & config)
{
  config("chairOffset", chairOffset_);
  config("robotOffset", robotOffset_);
  config("pandaOffset", pandaOffset_);
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
    HipsPose.rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
        sva::PTransformd::Identity().rotation(), HipsPose.rotation());
    // Initializing at zero height (easier)
    HipsPose.translation().z() = sva::PTransformd::Identity().translation().z();

    // adjust human position in mujoco
    mc_rtc::log::info("[ResetPoses state] Resetting human mujoco position");

    // if (ctl.datastore().has(fmt::format("{}::SetPosW", ctl.robots().robot("human").name())))
    // {
    //   // Explicit arguments in call to have automatic cast to const
    //   ctl.datastore().call<void, const sva::PTransformd &>(fmt::format("{}::SetPosW",
    //   ctl.robots().robot("human").name()), HipsPose);
    // }

    if(ctl.robots().hasRobot("chair"))
    {
      // Adjust chair position relative to human model
      mc_rtc::log::info("[ResetPoses state] Resetting chair control position");
      ctl.robots().robot("chair").posW(HipsPose * chairOffset_);

      // Adjust observed chair position relative to human model
      mc_rtc::log::info("[ResetPoses state] Resetting observed chair control position");
      ctl.realRobots().robot("chair").posW(HipsPose * chairOffset_);

      // adjust chair position in mujoco
      mc_rtc::log::info("[ResetPoses state] Resetting chair mujoco position");
      ctl.datastore().call<void, const sva::PTransformd &>(
          fmt::format("{}::SetPosW", ctl.robots().robot("chair").name()), HipsPose * chairOffset_);

      if(ctl.robots().hasRobot("panda"))
      {
        // Adjust panda position relative to chair model
        mc_rtc::log::info("[ResetPoses state] Resetting panda control position");
        ctl.robots().robot("panda").posW(pandaOffset_ * ctl.robots().robot("chair").posW());

        // Adjust observed pabda position relative to chair model
        mc_rtc::log::info("[ResetPoses state] Resetting observed panda control position");
        ctl.realRobots().robot("panda").posW(pandaOffset_ * ctl.robots().robot("chair").posW());

        // adjust panda position in mujoco
        mc_rtc::log::info("[ResetPoses state] Resetting chair mujoco position");
        ctl.datastore().call<void, const sva::PTransformd &>(
            fmt::format("{}::SetPosW", ctl.robots().robot("panda").name()),
            pandaOffset_ * ctl.robots().robot("chair").posW());
      }

      // Adjust main robot position relative to chair
      mc_rtc::log::info("[ResetPoses state] Resetting main robot control position");
      ctl.robots().robot().posW(robotOffset_ * ctl.robots().robot("chair").posW());

      // Adjust observed main robot position relative to chair
      mc_rtc::log::info("[ResetPoses state] Resetting observed main robot control position");
      ctl.realRobots().robot().posW(robotOffset_ * ctl.robots().robot("chair").posW());

      // adjust main robot position in mujoco
      mc_rtc::log::info("[ResetPoses state] Resetting main robot mujoco position");
      ctl.datastore().call<void, const sva::PTransformd &>(fmt::format("{}::SetPosW", ctl.robots().robot().name()),
                                                           robotOffset_ * ctl.robots().robot("chair").posW());
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
