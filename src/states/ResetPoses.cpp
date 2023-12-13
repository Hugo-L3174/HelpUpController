#include "ResetPoses.h"

// #include "../HelpUpController.h"
#include <mc_control/fsm/Controller.h>

void ResetPoses::configure(const mc_rtc::Configuration & config)
{
  config("chairOffset", chairOffset_);
  config("robotOffset", robotOffset_);
  config("pandaOffset", pandaOffset_);
}

void ResetPoses::start(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);
}

bool ResetPoses::run(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);

  if(!ctl.datastore().has("HRP4IsReady"))
  {
    mc_rtc::log::warning("Waiting for HRP4");
    return false;
  }

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
      auto dsEntry = fmt::format("{}::SetPosW", ctl.robots().robot("chair").name());
      if(ctl.datastore().has(dsEntry))
      {
        ctl.datastore().call<void, const sva::PTransformd &>(dsEntry, HipsPose * chairOffset_);
      }

      if(ctl.robots().hasRobot("panda"))
      {
        // Adjust panda position relative to chair model
        mc_rtc::log::info("[ResetPoses state] Resetting panda control position");
        ctl.robots().robot("panda").posW(pandaOffset_ * ctl.robots().robot("chair").posW());

        // Adjust observed pabda position relative to chair model
        mc_rtc::log::info("[ResetPoses state] Resetting observed panda control position");
        ctl.realRobots().robot("panda").posW(pandaOffset_ * ctl.robots().robot("chair").posW());

        // adjust panda position in mujoco
        mc_rtc::log::info("[ResetPoses state] Resetting panda mujoco position");
        dsEntry = fmt::format("{}::SetPosW", ctl.robots().robot("panda").name());
        if(ctl.datastore().has(dsEntry))
        {
          ctl.datastore().call<void, const sva::PTransformd &>(dsEntry,
                                                               pandaOffset_ * ctl.robots().robot("chair").posW());
        }
      }

      if(ctl.robots().hasRobot("human"))
      {
        auto HumanHipsPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName);
        // Adjust human position relative to human model
        mc_rtc::log::info("[ResetPoses state] Resetting human control position");
        ctl.robots().robot("human").posW(HumanHipsPose);

        // Adjust observed human position relative to human model
        mc_rtc::log::info("[ResetPoses state] Resetting observed human control position");
        ctl.realRobots().robot("human").posW(HumanHipsPose);

        // adjust human position in mujoco
        // mc_rtc::log::info("[ResetPoses state] Resetting human mujoco position");
        // auto dsEntry = fmt::format("{}::SetPosW", ctl.robots().robot("human").name());
        // if(ctl.datastore().has(dsEntry))
        // {
        //   ctl.datastore().call<void, const sva::PTransformd &>(dsEntry, HumanHipsPose);
        // }
      }

      // Adjust main robot position relative to chair
      mc_rtc::log::info("[ResetPoses state] Resetting main robot control position");
      ctl.robots().robot().posW(robotOffset_ * ctl.robots().robot("chair").posW());

      // Adjust observed main robot position relative to chair
      mc_rtc::log::info("[ResetPoses state] Resetting observed main robot control position");
      ctl.realRobots().robot().posW(robotOffset_ * ctl.robots().robot("chair").posW());

      // adjust main robot position in mujoco
      mc_rtc::log::info("[ResetPoses state] Resetting main robot mujoco position");
      dsEntry = fmt::format("{}::SetPosW", ctl.robots().robot().name());
      if(ctl.datastore().has(dsEntry))
      {
        ctl.datastore().call<void, const sva::PTransformd &>(dsEntry,
                                                             robotOffset_ * ctl.robots().robot("chair").posW());
      }
    }
  }
  else
  {
    mc_rtc::log::info("[{}] Waiting for datastore initialization...", name());
    output("NOK");
    return false;
  }

  ctl.datastore().call<void>("LoadHRP4ObserverPipeline");

  output("OK");
  return true;
}

void ResetPoses::teardown(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);
}

EXPORT_SINGLE_STATE("ResetPoses", ResetPoses)
