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

  if (config_.has("mode"))
  {
    auto mode = static_cast<std::string>(config_("mode"));
    if (mode.compare("forceConstrained") == 0)
    {
      mode_ = forceConstraint;
    }
  } 
  
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  auto RHForceConf = config_("ForceRHand");

  RHadmittance_ = RHForceConf("admittance");
  RHstiffness_ = RHForceConf("stiffness");
  RHdamping_ = RHForceConf("damping");
  RHmaxVel_ = RHForceConf("maxVel");
  RHwrench_ = RHForceConf("wrench");

  auto LHForceConf = config_("ForceLHand");  

  LHadmittance_ = LHForceConf("admittance");
  LHstiffness_ = LHForceConf("stiffness");
  LHdamping_ = LHForceConf("damping");
  LHmaxVel_ = LHForceConf("maxVel");
  LHwrench_ = LHForceConf("wrench");


  switch (mode_)
  {
  case simpleAdmi:
    // right hand admittance
    rightHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("RightHandFlat", ctl.robots(), ctl.robots().robotIndex());
    rightHandAdmittancePtr_->admittance(sva::ForceVec(RHadmittance_));
    rightHandAdmittancePtr_->setGains(sva::MotionVec(RHstiffness_), sva::MotionVec(RHdamping_));
    rightHandAdmittancePtr_->maxLinearVel(RHmaxVel_);
    rightHandAdmittancePtr_->targetWrench(sva:: ForceVec(RHwrench_));
    // setting right hand target surface
    // auto target = ctl.realRobot("human").surfacePose("RightShoulder");
    rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("Chest"));

    // left hand admittance
    leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("LeftHandFlat", ctl.robots(), ctl.robots().robotIndex());
    leftHandAdmittancePtr_->admittance(sva::ForceVec(LHadmittance_));
    leftHandAdmittancePtr_->setGains(sva::MotionVec(LHstiffness_), sva::MotionVec(LHdamping_));
    leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);
    leftHandAdmittancePtr_->targetWrench(sva:: ForceVec(LHwrench_));
    // setting left hand target surface
    // target = ctl.realRobot("human").surfacePose("Back");
    leftHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("Back"));
    // rightHandAdmittancePtr_->weight(weight_);
    // leftHandAdmittancePtr_->weight(weight_);

    // rightHandAdmittancePtr_->stiffness(stiffness_);
    // leftHandAdmittancePtr_->stiffness(stiffness_);

    // adding tasks to solver
    addToGUI(*ctl.gui(), ctl);
    ctl.solver().addTask(rightHandAdmittancePtr_);
    ctl.solver().addTask(leftHandAdmittancePtr_);
    break;
  


  case forceConstraint:
    rightHandForceConstPtr_ = std::make_shared<mc_tasks::ForceConstrainedTransformTask>(ctl.robot().frame("RightHandFlat")); //TODO stiffness and weight of task ? here default stiff 2 weight 500
    // Express the wrench constraint: max normal force of 20N with a soft margin of 5N on the shoulder frame
    Eigen::Vector6d dof = Eigen::Vector6d::Zero();
    dof(5) = 1.0; //selecting only z axis to constraint
    rightHandForceConstPtr_->addFrameConstraint(ctl.robot("human").frame("RightShoulder"),
                            dof,
                            sva::ForceVecd(Eigen::Vector3d::Zero(), {0, 0, 20.0}),
                            sva::ForceVecd(Eigen::Vector3d::Zero(), {0, 0, 5.0}));
    
    
    leftHandForceConstPtr_ = std::make_shared<mc_tasks::ForceConstrainedTransformTask>(ctl.robot().frame("LeftHandFlat"));
    break;
  }
  
  

}

bool RobotHolding::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  
  switch (mode_)
  {
  case simpleAdmi:
    // updating target positions each iteration
    rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("RightShoulder"));
    leftHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("Back"));
    break;
  
  case forceConstraint:

    break;
  }


  output("OK");
  return true;
}

void RobotHolding::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  switch (mode_)
  {
  case simpleAdmi:
    ctl.solver().removeTask(rightHandAdmittancePtr_);
    ctl.solver().removeTask(leftHandAdmittancePtr_);
    //TODO: remove gui addition when teardown
    break;
  
  case forceConstraint:
    break;
  }
  
}

// TODO: case of force constrained task
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
