#include "AdjustHumanSurfaces.h"
#include <mc_control/fsm/Controller.h>

void AdjustHumanSurfaces::configure(const mc_rtc::Configuration & config) {}

void AdjustHumanSurfaces::start(mc_control::fsm::Controller & ctl) {}

bool AdjustHumanSurfaces::run(mc_control::fsm::Controller & ctl)
{
  auto & human = ctl.robot("human");
  auto & outputHuman = ctl.outputRobot("human");
  auto & robot = ctl.robot();

  // get current end-effector poses: as they have made contact before through damping we know they are at the actual
  // surface position on the person
  auto X_0_realPoseRH = robot.frame("RightHand").position();
  auto X_0_realPoseLH = robot.frame("LeftHand").position();
  // adjust the human surfaces positions from these real contacts
  auto X_0_b = human.bodyPosW("TorsoLink");

  auto X_b_RHs = X_0_realPoseRH * X_0_b.inv();
  auto X_b_LHs = X_0_realPoseLH * X_0_b.inv();

  mc_rtc::log::info("Updating transform for surface {} from {} to {}", "RightShoulder",
                    human.surface("RightShoulder").X_b_s().translation().transpose(),
                    X_b_RHs.translation().transpose());
  mc_rtc::log::info("Updating transform for surface {} from {} to {}", "Back",
                    human.surface("Back").X_b_s().translation().transpose(), X_b_LHs.translation().transpose());

  human.surface("RightShoulder").X_b_s(X_b_RHs);
  human.frame("RightShoulder").X_p_f(X_b_RHs);
  human.surface("Back").X_b_s(X_b_LHs);
  human.frame("Back").X_p_f(X_b_LHs);
  human.forwardKinematics();
  human.forwardVelocity();
  human.forwardAcceleration();
  outputHuman.surface("RightShoulder").X_b_s(X_b_RHs);
  outputHuman.frame("RightShoulder").X_p_f(X_b_RHs);
  outputHuman.surface("Back").X_b_s(X_b_LHs);
  outputHuman.frame("Back").X_p_f(X_b_LHs);
  outputHuman.forwardKinematics();
  outputHuman.forwardVelocity();
  outputHuman.forwardAcceleration();

  mc_rtc::log::info("Right shoulder surface is now {}",
                    human.surface("RightShoulder").X_b_s().translation().transpose());
  mc_rtc::log::info("Right shoulder frame is now {}", human.frame("RightShoulder").X_b_f().translation().transpose());

  output("OK");
  return true;
}

void AdjustHumanSurfaces::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("AdjustHumanSurfaces", AdjustHumanSurfaces)
