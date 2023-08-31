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

  // if (config_.has("mode"))
  // {
  //   auto mode = static_cast<std::string>(config_("mode"));
  //   if (mode.compare("forceConstrained") == 0)
  //   {
  //     mode_ = forceConstraint;
  //   }
  // } 
  
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  auto RHForceConf = config_("ForceRHand");

  RHadmittance_ = RHForceConf("admittance");
  RHstiffness_ = RHForceConf("stiffness");
  RHdamping_ = RHForceConf("damping");
  RHmaxVel_ = RHForceConf("maxVel");
  RHwrench_ = RHForceConf("wrench");
  RHForceConf("surface", RHsurf_);
  RHForceConf("target", RHtarget_);

  auto LHForceConf = config_("ForceLHand");  

  LHadmittance_ = LHForceConf("admittance");
  LHstiffness_ = LHForceConf("stiffness");
  LHdamping_ = LHForceConf("damping");
  LHmaxVel_ = LHForceConf("maxVel");
  LHwrench_ = LHForceConf("wrench");
  LHForceConf("surface", LHsurf_);
  LHForceConf("target", LHtarget_);


  rightHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> (RHsurf_, ctl.robots(), ctl.robots().robotIndex());
  rightHandAdmittancePtr_->admittance(sva::ForceVec(RHadmittance_));
  rightHandAdmittancePtr_->setGains(sva::MotionVec(RHstiffness_), sva::MotionVec(RHdamping_));
  rightHandAdmittancePtr_->maxLinearVel(RHmaxVel_);
  rightHandAdmittancePtr_->targetWrench(RHwrench_);
  rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose(RHtarget_));


  leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> (LHsurf_, ctl.robots(), ctl.robots().robotIndex());
  leftHandAdmittancePtr_->admittance(sva::ForceVec(LHadmittance_));
  leftHandAdmittancePtr_->setGains(sva::MotionVec(LHstiffness_), sva::MotionVec(LHdamping_));
  leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);
  leftHandAdmittancePtr_->targetWrench(LHwrench_);
  leftHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose(LHtarget_));

  // adding tasks to solver
  addToGUI(*ctl.gui(), ctl);
  ctl.solver().addTask(rightHandAdmittancePtr_);
  ctl.solver().addTask(leftHandAdmittancePtr_);

  

  // case forceConstraint:
  //   // ctl.addContact({ctl.robot().name(), "human", "RightHandFlat", "RightShoulder"});
  //   // ctl.addContact({ctl.robot().name(), "human", "LeftHandFlat", "Back"});
  //   // Trigger the contact update now so the contact is in the solver
  //   ctl.updateContacts();

  //   // Todo: generalize lambda (pb of string argument)
  //   mc_rbdyn::Contact contactRH = [this, &ctl]() {
  //     for(const auto & c : ctl.solver().contacts())
  //     {
  //       if(c.r1Surface()->name() == "RightHandFlat")
  //       {
  //         return c;
  //       }
  //     }
  //     mc_rtc::log::error_and_throw("Unreachable");
  //   }();

  //   mc_rbdyn::Contact contactLH = [this, &ctl]() {
  //     for(const auto & c : ctl.solver().contacts())
  //     {
  //       if(c.r1Surface()->name() == "LeftHandFlat")
  //       {
  //         return c;
  //       }
  //     }
  //     mc_rtc::log::error_and_throw("Unreachable");
  //   }();

  //   // Adding RH contact task
  //   auto cid = contactRH.contactId(ctl.robots());
  //   rightHandForceConstPtr_ = std::make_shared<TrackDesiredForceTask>(ctl.solver(), cid); 
  //   rightHandForceConstPtr_->setTargetWrench(RHwrench_);
  //   tasks_solver(ctl.solver()).addTask(rightHandForceConstPtr_.get());

  //   // Adding LH
  //   cid = contactLH.contactId(ctl.robots());
  //   leftHandForceConstPtr_ = std::make_shared<TrackDesiredForceTask>(ctl.solver(), cid); 
  //   leftHandForceConstPtr_->setTargetWrench(LHwrench_);
  //   tasks_solver(ctl.solver()).addTask(leftHandForceConstPtr_.get());
    
  //   // Add gui elements
  //   GUIForceContacts(*ctl.gui(), ctl);


  //   // Add hands admittance tasks
  //   // right hand admittance
  //   rightHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("RightHandFlat", ctl.robots(), ctl.robots().robotIndex());
  //   rightHandAdmittancePtr_->admittance(sva::ForceVec(RHadmittance_));
  //   rightHandAdmittancePtr_->setGains(sva::MotionVec(RHstiffness_), sva::MotionVec(RHdamping_));
  //   rightHandAdmittancePtr_->maxLinearVel(RHmaxVel_);
  //   rightHandAdmittancePtr_->targetWrench(RHwrench_);
  //   rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("RightShoulder"));
  //   // left hand admittance
  //   leftHandAdmittancePtr_ = std::make_shared<mc_tasks::force::AdmittanceTask> ("LeftHandFlat", ctl.robots(), ctl.robots().robotIndex());
  //   leftHandAdmittancePtr_->admittance(sva::ForceVec(LHadmittance_));
  //   leftHandAdmittancePtr_->setGains(sva::MotionVec(LHstiffness_), sva::MotionVec(LHdamping_));
  //   leftHandAdmittancePtr_->maxLinearVel(LHmaxVel_);
  //   leftHandAdmittancePtr_->targetWrench(LHwrench_);
  //   leftHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose("Back"));
  //   // adding admittance tasks to solver
  //   addToGUI(*ctl.gui(), ctl);
  //   ctl.solver().addTask(rightHandAdmittancePtr_);
  //   ctl.solver().addTask(leftHandAdmittancePtr_);

  
  

}

bool RobotHolding::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  
  rightHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose(RHtarget_));
  leftHandAdmittancePtr_->targetPose(ctl.robot("human").surfacePose(LHtarget_));

  Eigen::Vector6d gain;
  gain << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  // gain for setExternalWrenches is used only if we use the measured forces (might vibrate, thus gains)
  auto handGains = sva::MotionVecd(gain);
  std::vector<sva::MotionVecd> gainsVec;
  gainsVec.push_back(handGains);
  gainsVec.push_back(handGains);

  std::vector<std::string> surfVec;
  surfVec.push_back(LHsurf_);
  surfVec.push_back(RHsurf_);

  std::vector<sva::ForceVecd> wrenchVec;
  wrenchVec.push_back(ctl.getLHWrenchComputed());
  wrenchVec.push_back(ctl.getRHWrenchComputed());

  if (ctl.datastore().has("RobotStabilizer::setExternalWrenches"))
  {
    ctl.datastore().call("RobotStabilizer::setExternalWrenches", 
                          static_cast<const std::vector<std::string> &>(surfVec), 
                          static_cast<const std::vector<sva::ForceVecd> &>(wrenchVec),
                          static_cast<const std::vector<sva::MotionVecd> &>(gainsVec)
                        );
  }
  

  // Completion
  // (TODO: find a cleaner way than just forcing next state in gui)
  // const auto & dcm = stabilizerTask_->measuredDCM();
  // if((((dcm - comTarget_).cwiseAbs() - dcmThreshold_).array() < 0.).all())
  // {
  //   output("OK");
  //   return true;
  // }
  return false;
 
}

void RobotHolding::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration DefaultExternalWrenchConf_;
  ctl.datastore().call("RobotStabilizer::setExternalWrenchConfiguration", 
                        static_cast<const mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration &>(DefaultExternalWrenchConf_)
                      );
  ctl.solver().removeTask(rightHandAdmittancePtr_);
  ctl.solver().removeTask(leftHandAdmittancePtr_);
  //TODO: remove gui addition when teardown
  
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
        if(c.r1Surface()->name() == RHsurf_)
        {
          return c;
        }
      }
      mc_rtc::log::error_and_throw("Unreachable");
    }();

    mc_rbdyn::Contact contactLH = [this, &ctl]() {
      for(const auto & c : ctl.solver().contacts())
      {
        if(c.r1Surface()->name() == LHsurf_)
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
