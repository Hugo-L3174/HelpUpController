#include "RobotHolding.h"
#include <mc_solver/TasksQPSolver.h>

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
    rightHandAdmittancePtr_->targetWrench(RHwrench_);
    // setting right hand target surface
    // auto target = ctl.realRobot("human").surfacePose("RightShoulder");
    rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("Chest"));

    // left hand admittance
    leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("LeftHandFlat", ctl.robots(), ctl.robots().robotIndex());
    leftHandAdmittancePtr_->admittance(sva::ForceVec(LHadmittance_));
    leftHandAdmittancePtr_->setGains(sva::MotionVec(LHstiffness_), sva::MotionVec(LHdamping_));
    leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);
    leftHandAdmittancePtr_->targetWrench(LHwrench_);
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
    // ctl.addContact({ctl.robot().name(), "human", "RightHandFlat", "RightShoulder"});
    // ctl.addContact({ctl.robot().name(), "human", "LeftHandFlat", "Back"});
    // Trigger the contact update now so the contact is in the solver
    ctl.updateContacts();

    // Todo: generalize lambda (pb of string argument)
    mc_rbdyn::Contact contactRH = [this, &ctl]() {
      for(const auto & c : ctl.solver().contacts())
      {
        if(c.r1Surface()->name() == "RightHandFlat")
        {
          return c;
        }
      }
      mc_rtc::log::error_and_throw("Unreachable");
    }();

    mc_rbdyn::Contact contactLH = [this, &ctl]() {
      for(const auto & c : ctl.solver().contacts())
      {
        if(c.r1Surface()->name() == "LeftHandFlat")
        {
          return c;
        }
      }
      mc_rtc::log::error_and_throw("Unreachable");
    }();

    // Adding RH contact task
    auto cid = contactRH.contactId(ctl.robots());
    rightHandForceConstPtr_ = std::make_shared<TrackDesiredForceTask>(ctl.solver(), cid); 
    rightHandForceConstPtr_->setTargetWrench(RHwrench_);
    tasks_solver(ctl.solver()).addTask(rightHandForceConstPtr_.get());

    // Adding LH
    cid = contactLH.contactId(ctl.robots());
    leftHandForceConstPtr_ = std::make_shared<TrackDesiredForceTask>(ctl.solver(), cid); 
    leftHandForceConstPtr_->setTargetWrench(LHwrench_);
    tasks_solver(ctl.solver()).addTask(leftHandForceConstPtr_.get());
    
    // Add gui elements
    GUIForceContacts(*ctl.gui(), ctl);


    // Add hands admittance tasks
    // right hand admittance
    rightHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("RightHandFlat", ctl.robots(), ctl.robots().robotIndex());
    rightHandAdmittancePtr_->admittance(sva::ForceVec(RHadmittance_));
    rightHandAdmittancePtr_->setGains(sva::MotionVec(RHstiffness_), sva::MotionVec(RHdamping_));
    rightHandAdmittancePtr_->maxLinearVel(RHmaxVel_);
    rightHandAdmittancePtr_->targetWrench(RHwrench_);
    rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("RightShoulder"));
    // left hand admittance
    leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("LeftHandFlat", ctl.robots(), ctl.robots().robotIndex());
    leftHandAdmittancePtr_->admittance(sva::ForceVec(LHadmittance_));
    leftHandAdmittancePtr_->setGains(sva::MotionVec(LHstiffness_), sva::MotionVec(LHdamping_));
    leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);
    leftHandAdmittancePtr_->targetWrench(LHwrench_);
    leftHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("Back"));
    // adding admittance tasks to solver
    addToGUI(*ctl.gui(), ctl);
    ctl.solver().addTask(rightHandAdmittancePtr_);
    ctl.solver().addTask(leftHandAdmittancePtr_);

    
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

void RobotHolding::GUIForceContacts(mc_rtc::gui::StateBuilder & gui, mc_control::fsm::Controller & ctl)
{ 
  mc_rbdyn::Contact contactRH = [this, &ctl]() {
      for(const auto & c : ctl.solver().contacts())
      {
        if(c.r1Surface()->name() == "RightHandFlat")
        {
          return c;
        }
      }
      mc_rtc::log::error_and_throw("Unreachable");
    }();

    mc_rbdyn::Contact contactLH = [this, &ctl]() {
      for(const auto & c : ctl.solver().contacts())
      {
        if(c.r1Surface()->name() == "LeftHandFlat")
        {
          return c;
        }
      }
      mc_rtc::log::error_and_throw("Unreachable");
    }();

  ctl.gui()->addElement(
      {"Force Contacts"},
      mc_rtc::gui::NumberInput(
          "right weight", [this]() { return rightHandForceConstPtr_->weight(); }, [this](double w) { rightHandForceConstPtr_->weight(w); }),
      mc_rtc::gui::NumberInput(
          "left weight", [this]() { return leftHandForceConstPtr_->weight(); }, [this](double w) { leftHandForceConstPtr_->weight(w); }),
      mc_rtc::gui::ArrayInput(
          "Right Target wrench", [this]() { return RHwrench_; },
          [this](const sva::ForceVecd & fv) {
            RHwrench_ = fv;
            rightHandForceConstPtr_->setTargetWrench(fv);
          }),
      mc_rtc::gui::ArrayInput(
          "Left Target wrench", [this]() { return LHwrench_; },
          [this](const sva::ForceVecd & fv) {
            LHwrench_ = fv;
            leftHandForceConstPtr_->setTargetWrench(fv);
          }),
      mc_rtc::gui::ArrayLabel("QP wrench RH", [this, contactRH, &ctl]() { return ctl.solver().desiredContactForce(contactRH); }),
      mc_rtc::gui::ArrayLabel("QP wrench LH", [this, contactLH, &ctl]() { return ctl.solver().desiredContactForce(contactLH); }));
}

EXPORT_SINGLE_STATE("RobotHolding", RobotHolding)
