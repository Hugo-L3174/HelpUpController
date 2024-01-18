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
  auto HumanHipsPose = sva::PTransformd::Identity();
  const std::string & segmentName = "Pelvis";

  if(ctl.datastore().has("XsensPlugin"))
  {
    HipsPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName);
    HumanHipsPose = HipsPose;
    // Keeping rotation yaw but setting roll and pitch to zero (basically keeping chair horizontal)
    HipsPose.rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
        sva::PTransformd::Identity().rotation(), HipsPose.rotation());
    // Initializing at zero height (easier)
    HipsPose.translation().z() = sva::PTransformd::Identity().translation().z();

    if(ctl.robots().hasRobot("chair"))
    {
      adjustPoses(ctl, HipsPose, "chair",
                  chairOffset_); // ref pose and offset are inverted because we want to offset the starting position

      if(ctl.robots().hasRobot("panda"))
      {
        adjustPoses(ctl, pandaOffset_, "panda", ctl.robots().robot("chair").posW());
      }

      if(ctl.robots().hasRobot("human"))
      {
        // careful: adjusting mujoco position might make robot fall in physics bc of retargetting
        adjustPoses(ctl, sva::PTransformd::Identity(), "human", HumanHipsPose);
      }

      // Adjust main robot position relative to chair
      adjustPoses(ctl, robotOffset_, ctl.robot().name(), ctl.robots().robot("chair").posW());

      // After everything is finalized relative to the chair oriented from the hips pose, re adjust chair height from
      // human hips pose (useful for contact distance)
      sva::PTransformd chairHeightOffset = sva::PTransformd::Identity();
      chairHeightOffset.translation().z() =
          HumanHipsPose.translation().z() - 0.4
          - 0.15; // chair surface from origin is 0.4 high, hips to back of legs is around 0.15
      adjustPoses(ctl, chairHeightOffset, "chair", ctl.robots().robot("chair").posW());

      // TODO:
      // investiguer external wrenches qui agissent quand même
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

void ResetPoses::adjustPoses(mc_control::fsm::Controller & ctl,
                             sva::PTransformd offset,
                             std::string robotName,
                             sva::PTransformd refPose)
{
  // Adjust main robot position relative to chair
  mc_rtc::log::info("[ResetPoses state] Resetting {} control position", robotName);
  ctl.robots().robot(robotName).posW(offset * refPose);

  // Adjust observed main robot position relative to reference robot
  mc_rtc::log::info("[ResetPoses state] Resetting observed {} control position", robotName);
  ctl.realRobots().robot(robotName).posW(offset * refPose);

  // adjust robot position in mujoco
  auto dsEntry = fmt::format("{}::SetPosW", ctl.robots().robot(robotName).name());
  if(ctl.datastore().has(dsEntry))
  {
    mc_rtc::log::info("[ResetPoses state] Resetting {} mujoco position", robotName);
    ctl.datastore().call<void, const sva::PTransformd &>(dsEntry, offset * refPose);
  }
}

void ResetPoses::teardown(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);
}

EXPORT_SINGLE_STATE("ResetPoses", ResetPoses)
