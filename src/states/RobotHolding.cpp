#include "RobotHolding.h"

#include "../HelpUpController.h"

void RobotHolding::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void RobotHolding::start(mc_control::fsm::Controller & ctl_)
{
  if (config_.has("weight")) weight_ = static_cast<int>(config_("weight"));
  else weight_ = 1000;

  if (config_.has("stiffness")) stiffness_ = static_cast<int>(config_("stiffness"));
  else stiffness_ = 5;



  // if (config_.has("dimWeight")) dimWeight_ = config_("dimWeight");
  // else dimWeight_.setOnes();
  auto RHadmiConf = config_("AdmittanceRHand");

  RHadmittance_ = RHadmiConf("admittance");
  RHstiffness_ = RHadmiConf("stiffness");
  RHdamping_ = RHadmiConf("damping");
  RHmaxVel_ = RHadmiConf("maxVel");
  RHwrench_ = RHadmiConf("wrench");

  auto LHadmiConf = config_("AdmittanceLHand");  

  LHadmittance_ = LHadmiConf("admittance");
  LHstiffness_ = LHadmiConf("stiffness");
  LHdamping_ = LHadmiConf("damping");
  LHmaxVel_ = LHadmiConf("maxVel");
  LHwrench_ = LHadmiConf("wrench");
  
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  // right hand admittance
  rightHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("RightHandFlat", ctl.robots(), ctl.robots().robotIndex());
  auto Radmittance = sva::ForceVec(RHadmittance_);
  rightHandAdmittancePtr_->admittance(Radmittance);

  auto Rstiffness = sva::MotionVec(RHstiffness_);
  auto Rdamping = sva::MotionVec(RHdamping_);
  rightHandAdmittancePtr_->setGains(Rstiffness, Rdamping);

  rightHandAdmittancePtr_->maxLinearVel(RHmaxVel_);

  auto Rwrench = sva:: ForceVec(RHwrench_);
  rightHandAdmittancePtr_->targetWrench(Rwrench);

  // setting right hand target surface
  // auto target = ctl.realRobot("human").surfacePose("RightShoulder");
  auto Rtarget = ctl.robot("human").surfacePose("Chest");
  rightHandAdmittancePtr_->targetPose(Rtarget);


  // left hand admittance
  leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("LeftHandFlat", ctl.robots(), ctl.robots().robotIndex());
  auto Ladmittance = sva::ForceVec(LHadmittance_);
  leftHandAdmittancePtr_->admittance(Ladmittance);

  auto Lstiffness = sva::MotionVec(LHstiffness_);
  auto Ldamping = sva::MotionVec(LHdamping_);
  leftHandAdmittancePtr_->setGains(Lstiffness, Ldamping);

  leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);

  auto Ldesired_wrench = sva:: ForceVec(LHwrench_);
  leftHandAdmittancePtr_->targetWrench(Ldesired_wrench);

  // setting left hand target surface
  // target = ctl.realRobot("human").surfacePose("Back");
  auto Ltarget = ctl.robot("human").surfacePose("Back");
  leftHandAdmittancePtr_->targetPose(Ltarget);


  // rightHandAdmittancePtr_->weight(weight_);
  // leftHandAdmittancePtr_->weight(weight_);

  // rightHandAdmittancePtr_->stiffness(stiffness_);
  // leftHandAdmittancePtr_->stiffness(stiffness_);

  // adding tasks to solver
  addToGUI(*ctl.gui(), ctl);
  ctl.solver().addTask(rightHandAdmittancePtr_);
  ctl.solver().addTask(leftHandAdmittancePtr_);
}

bool RobotHolding::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  
  // updating target positions each iteration
  auto target = ctl.robot("human").surfacePose("RightShoulder"); //Chest
  rightHandAdmittancePtr_->targetPose(target);

  target = ctl.robot("human").surfacePose("Back");
  leftHandAdmittancePtr_->targetPose(target);

  output("OK");
  return true;
}

void RobotHolding::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  ctl.solver().removeTask(rightHandAdmittancePtr_);
  ctl.solver().removeTask(leftHandAdmittancePtr_);
}

void RobotHolding::addToGUI(mc_rtc::gui::StateBuilder & gui, mc_control::fsm::Controller & ctl_)
{
  using namespace mc_rtc::gui;
  using Style = mc_rtc::gui::plot::Style;
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  ///// GUI MARKERS
  constexpr double ARROW_HEAD_DIAM = 0.015;
  constexpr double ARROW_HEAD_LEN = 0.05;
  constexpr double ARROW_SHAFT_DIAM = 0.015;
  constexpr double FORCE_SCALE = 0.003;

  ArrowConfig CoMForceArrowConfig;
  CoMForceArrowConfig.shaft_diam = 1 * ARROW_SHAFT_DIAM;
  CoMForceArrowConfig.head_diam = 1 * ARROW_HEAD_DIAM;
  CoMForceArrowConfig.head_len = 1 * ARROW_HEAD_LEN;
  CoMForceArrowConfig.scale = 1.;
  CoMForceArrowConfig.start_point_scale = 0.02;
  CoMForceArrowConfig.end_point_scale = 0.;

  ArrowConfig netWrenchForceArrowConfig = CoMForceArrowConfig;
  netWrenchForceArrowConfig.color = Color::Red;

  ArrowConfig refCoMForceArrowConfig = CoMForceArrowConfig;
  refCoMForceArrowConfig = Color::Yellow;

  ForceConfig copForceConfig(Color::Green);
  copForceConfig.start_point_scale = 0.02;
  copForceConfig.end_point_scale = 0.;

  constexpr double COM_POINT_SIZE = 0.02;
  constexpr double DCM_POINT_SIZE = 0.015;

  // gui.addElement({"Tasks", name_, "Markers", "CoM-DCM"},
  //                Arrow(
  //                    "CoM_CoM", CoMArrowConfig, [this]() -> Eigen::Vector3d { return zmpTarget_; },
  //                    [this]() -> Eigen::Vector3d { return comTarget_; }),
  //                Point3D("Measured_CoM", PointConfig(Color::Green, COM_POINT_SIZE), [this]() { return measuredCoM_; }),
  //                Point3D("CoM_DCM", PointConfig(Color::Yellow, DCM_POINT_SIZE), [this]() { return dcmTarget_; }),
  //                Point3D("Measured_DCM", PointConfig(Color::Green, DCM_POINT_SIZE),
  //                        [this]() -> Eigen::Vector3d { return measuredCoM_ + measuredCoMd_ / omega_; }));

  gui.addElement(
      {"Tasks", leftHandAdmittancePtr_->name(), "Markers", "Net wrench"},
      Arrow(
          "Measured_LeftHandForce", netWrenchForceArrowConfig, [this]() -> Eigen::Vector3d { return leftHandAdmittancePtr_->surfacePose().translation(); },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {
            return leftHandAdmittancePtr_->surfacePose().translation() + FORCE_SCALE * leftHandAdmittancePtr_->measuredWrench().force();
          }));

  // for(const auto footTask : footTasks)
  // {
  //   auto footT = footTask.second;
  //   gui.addElement({"Tasks", name_, "Markers", "Foot wrenches"},
  //                  Point3D("Stabilizer_" + footT->surface() + "CoP", PointConfig(Color::Magenta, 0.01),
  //                          [footT]() { return footT->targetCoPW(); }),
  //                  Force(
  //                      "Measured_" + footT->surface() + "CoPForce", copForceConfig,
  //                      [footT, this]() {
  //                        return robot().indirectSurfaceForceSensor(footT->surface()).worldWrenchWithoutGravity(robot());
  //                      },
  //                      [footT]() { return sva::PTransformd(footT->measuredCoPW()); }));
  // }


}

EXPORT_SINGLE_STATE("RobotHolding", RobotHolding)
