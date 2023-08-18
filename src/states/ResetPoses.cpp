#include "ResetPoses.h"

#include "../HelpUpController.h"

void ResetPoses::configure(const mc_rtc::Configuration & config)
{

}

void ResetPoses::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

bool ResetPoses::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  while (/* condition */)
  {
    // Adjust chair position relative to human model
    if(ctl.robots().hasRobot("chair")){
      mc_rtc::log::info("Human pos is {}", ctl.robots().robot("human").posW().translation().transpose());
      mc_rtc::log::info("Human pos is {}", ctl.datastore().get<sva::PTransformd>("ReplayPlugin::GetSegmentPose::HipsLink").translation().transpose());
      ctl.robots().robot("chair").posW(ctl.robots().robot("human").posW() * sva::PTransformd(Eigen::Vector3d(0.05, 0.12, -0.65)));
      mc_rtc::log::info("Chair pos is {}", ctl.robots().robot("chair").posW().translation().transpose());

      // Adjust main robot position relative to chair
      ctl.robots().robot().posW(ctl.robots().robot("chair").posW() * sva::PTransformd(sva::RotZ(M_PI/2.0), Eigen::Vector3d(-0.17, -0.3, 0.75))); 
      mc_rtc::log::info("Robot pos is {}", ctl.robots().robot().posW().translation().transpose());
    }

  }
  output("OK");
  return true;
}

void ResetPoses::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
}

EXPORT_SINGLE_STATE("ResetPoses", ResetPoses)
