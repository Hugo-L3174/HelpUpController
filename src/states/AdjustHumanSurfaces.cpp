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
  auto X_0_torso = human.bodyPosW("TorsoLink");

  auto X_b_RHs = X_0_realPoseRH * X_0_torso.inv();
  auto X_b_LHs = X_0_realPoseLH * X_0_torso.inv();

  mc_rtc::log::info("Updating transform for surface {} from {} to {}", "RightShoulder",
                    human.surface("RightShoulder").X_b_s().translation().transpose(),
                    X_b_RHs.translation().transpose());
  mc_rtc::log::info("Updating transform for surface {} from {} to {}", "Back",
                    human.surface("Back").X_b_s().translation().transpose(), X_b_LHs.translation().transpose());

  // updating rear surfaces to align with chair (we know the person is seated)
  auto & chair = ctl.robot("chair");
  auto X_0_chairTop = chair.frame("Top").position();
  auto X_0_rleg = human.bodyPosW("RLegLink");
  auto X_0_lleg = human.bodyPosW("LLegLink");

  auto X_origRCheek_Top = X_0_chairTop * human.frame("RCheek").position().inv();
  auto X_origLCheek_Top = X_0_chairTop * human.frame("LCheek").position().inv();

  auto X_newRCheek_Top = X_origRCheek_Top;
  X_newRCheek_Top.translation().z() = 0; // we want to keep the same z as the chair surface

  auto X_newLCheek_Top = X_origLCheek_Top;
  X_newLCheek_Top.translation().z() = 0; // we want to keep the same z as the chair surface

  auto X_b_RLs = X_newRCheek_Top.inv() * X_0_chairTop * X_0_rleg.inv();
  auto X_b_LLs = X_newLCheek_Top.inv() * X_0_chairTop * X_0_lleg.inv();

  // update everything
  human.surface("RightShoulder").X_b_s(X_b_RHs);
  human.frame("RightShoulder").X_p_f(X_b_RHs);
  human.surface("Back").X_b_s(X_b_LHs);
  human.frame("Back").X_p_f(X_b_LHs);
  human.surface("RCheek").X_b_s(X_b_RLs);
  human.frame("RCheek").X_p_f(X_b_RLs);
  human.surface("LCheek").X_b_s(X_b_LLs);
  human.frame("LCheek").X_p_f(X_b_LLs);
  human.forwardKinematics();
  human.forwardVelocity();
  human.forwardAcceleration();

  outputHuman.surface("RightShoulder").X_b_s(X_b_RHs);
  outputHuman.frame("RightShoulder").X_p_f(X_b_RHs);
  outputHuman.surface("Back").X_b_s(X_b_LHs);
  outputHuman.frame("Back").X_p_f(X_b_LHs);
  outputHuman.surface("RCheek").X_b_s(X_b_RLs);
  outputHuman.frame("RCheek").X_p_f(X_b_RLs);
  outputHuman.surface("LCheek").X_b_s(X_b_LLs);
  outputHuman.frame("LCheek").X_p_f(X_b_LLs);
  outputHuman.forwardKinematics();
  outputHuman.forwardVelocity();
  outputHuman.forwardAcceleration();

  output("OK");
  return true;
}

void AdjustHumanSurfaces::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("AdjustHumanSurfaces", AdjustHumanSurfaces)
