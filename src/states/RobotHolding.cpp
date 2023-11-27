#include "RobotHolding.h"

#include <mc_solver/TasksQPSolver.h>

#include "../HelpUpController.h"

void RobotHolding::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void RobotHolding::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  if(config_.has("RightHandAdmi"))
  {
    rightHandAdmittancePtr_ =
        mc_tasks::MetaTaskLoader::load<mc_tasks::force::AdmittanceTask>(ctl.solver(), config_("RightHandAdmi"));
    auto RHtarget = config_("RightHandAdmi")("targetFrame");
    RHtarget("robot", RHtargetRobot_);
    RHtarget("frame", RHtargetFrame_);
    RHtarget("offset", RHtargetOffset_);
    rightHandAdmittancePtr_->velFilterGain(0.1); // see admittance implem, we want the ref vel to be made of a higher
                                                 // prop of the wrench error (clamped!) than of the prev refVel
    // rightHandAdmittancePtr_->targetPose(RHtargetOffset_ * ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).position());
    ctl.solver().addTask(rightHandAdmittancePtr_);
  }

  if(config_.has("LeftHandAdmi"))
  {
    leftHandAdmittancePtr_ =
        mc_tasks::MetaTaskLoader::load<mc_tasks::force::AdmittanceTask>(ctl.solver(), config_("LeftHandAdmi"));
    auto LHtarget = config_("LeftHandAdmi")("targetFrame");
    LHtarget("robot", LHtargetRobot_);
    LHtarget("frame", LHtargetFrame_);
    LHtarget("offset", LHtargetOffset_);
    leftHandAdmittancePtr_->velFilterGain(0.1);
    // rightHandAdmittancePtr_->targetPose(RHtargetOffset_ * ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).position());
    ctl.solver().addTask(leftHandAdmittancePtr_);
  }

  if(config_.has("CoPRH"))
  {
    rightHandCoPTaskPtr_ = mc_tasks::MetaTaskLoader::load<mc_tasks::force::CoPTask>(ctl.solver(), config_("CoPRH"));
    auto RHtarget = config_("CoPRH")("targetFrame");
    RHtarget("robot", RHtargetRobot_);
    RHtarget("frame", RHtargetFrame_);
    RHtarget("offset", RHtargetOffset_);
    ctl.solver().addTask(rightHandCoPTaskPtr_);
  }

  if(config_.has("RightHandImped"))
  {
    rightHandImpedancePtr_ =
        mc_tasks::MetaTaskLoader::load<mc_tasks::force::ImpedanceTask>(ctl.solver(), config_("RightHandImped"));
    auto RHtarget = config_("RightHandImped")("targetFrame");
    RHtarget("robot", RHtargetRobot_);
    RHtarget("frame", RHtargetFrame_);
    RHtarget("offset", RHtargetOffset_);
    rightHandImpedancePtr_->targetPose(RHtargetOffset_ * ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).position());
    ctl.solver().addTask(rightHandImpedancePtr_);
  }

  if(config_.has("LeftHandImped"))
  {
    leftHandImpedancePtr_ =
        mc_tasks::MetaTaskLoader::load<mc_tasks::force::ImpedanceTask>(ctl.solver(), config_("LeftHandImped"));
    auto LHtarget = config_("LeftHandImped")("targetFrame");
    LHtarget("robot", LHtargetRobot_);
    LHtarget("frame", LHtargetFrame_);
    LHtarget("offset", LHtargetOffset_);
    leftHandImpedancePtr_->targetPose(LHtargetOffset_ * ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).position());
    ctl.solver().addTask(leftHandImpedancePtr_);
  }

  // adding tasks to solver
  // addToGUI(*ctl.gui(), ctl);
}

bool RobotHolding::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);

  // rightHandImpedancePtr_->targetPose(ctl.robot("human").surfacePose(RHtarget_));
  // rightHandImpedancePtr_->targetWrench(ctl.getRHWrenchComputed());

  // leftHandImpedancePtr_->targetPose(ctl.robot("human").surfacePose(LHtarget_));
  // leftHandImpedancePtr_->targetWrench(ctl.getLHWrenchComputed());

  // Eigen::Vector6d gain;
  // gain << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  // gain for setExternalWrenches is used only if we use the measured forces (might vibrate, thus gains)
  // auto handGains = sva::MotionVecd(gain);

  std::vector<sva::MotionVecd> gainsVec;
  std::vector<std::string> surfVec;
  std::vector<sva::ForceVecd> wrenchVec;

  if(config_.has("RightHandAdmi"))
  {
    // rightHandAdmittancePtr_->targetPose(RHtargetOffset_ *
    // ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).position()); rightHandAdmittancePtr_->targetSurface(); RHgains_=
    // sva::MotionVecd(Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01));
    RHgains_ = sva::MotionVecd(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));
    gainsVec.push_back(RHgains_);
    surfVec.push_back(config_("RightHandAdmi")("frame"));
    // wrenchVec.push_back(rightHandAdmittancePtr_->targetWrench());
    wrenchVec.push_back(sva::ForceVecd::Zero());
  }
  if(config_.has("LeftHandAdmi"))
  {
    // leftHandAdmittancePtr_->targetPose(LHtargetOffset_ * ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).position());
    // LHgains_= sva::MotionVecd(Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01));
    LHgains_ = sva::MotionVecd(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));
    gainsVec.push_back(LHgains_);
    surfVec.push_back(config_("LeftHandAdmi")("frame"));
    // wrenchVec.push_back(leftHandAdmittancePtr_->targetWrench());
    wrenchVec.push_back(sva::ForceVecd::Zero());
  }

  if(config_.has("CoPRH"))
  {
    rightHandCoPTaskPtr_->targetSurface(ctl.robot(RHtargetRobot_).robotIndex(), RHtargetFrame_, RHtargetOffset_);
    // gainsVec.push_back(RHgains_);
    // surfVec.push_back(config_("CoPRH")("frame"));
    // wrenchVec.push_back( rightHandCoPTaskPtr_->targetWrench());
  }

  if(config_.has("RightHandImped"))
  {
    rightHandImpedancePtr_->targetPose(RHtargetOffset_ * ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).position());
    RHgains_ = sva::MotionVecd(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));
    gainsVec.push_back(RHgains_);
    surfVec.push_back(config_("RightHandImped")("frame"));
    wrenchVec.push_back(sva::ForceVecd::Zero());
  }
  if(config_.has("LeftHandImped"))
  {
    leftHandImpedancePtr_->targetPose(LHtargetOffset_ * ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).position());
    LHgains_ = sva::MotionVecd(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));
    gainsVec.push_back(LHgains_);
    surfVec.push_back(config_("LeftHandImped")("frame"));
    wrenchVec.push_back(sva::ForceVecd::Zero());
  }

  ctl.datastore().call("RobotStabilizer::setExternalWrenches", static_cast<const std::vector<std::string> &>(surfVec),
                       static_cast<const std::vector<sva::ForceVecd> &>(wrenchVec),
                       static_cast<const std::vector<sva::MotionVecd> &>(gainsVec));

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
  ctl.datastore().call(
      "RobotStabilizer::setExternalWrenchConfiguration",
      static_cast<const mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration &>(DefaultExternalWrenchConf_));
  // ctl.solver().removeTask(rightHandAdmittancePtr_);
  // ctl.solver().removeTask(leftHandAdmittancePtr_);

  ctl.solver().removeTask(rightHandImpedancePtr_);
  ctl.solver().removeTask(leftHandImpedancePtr_);
  // TODO: remove gui addition when teardown
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
  //                Point3D("Measured_CoM", PointConfig(Color::Green, COM_POINT_SIZE), [this]() { return measuredCoM_;
  //                }), Point3D("CoM_DCM", PointConfig(Color::Yellow, DCM_POINT_SIZE), [this]() { return dcmTarget_; }),
  //                Point3D("Measured_DCM", PointConfig(Color::Green, DCM_POINT_SIZE),
  //                        [this]() -> Eigen::Vector3d { return measuredCoM_ + measuredCoMd_ / omega_; }));

  // gui.addElement(
  //     {"Tasks", leftHandAdmittancePtr_->name(), "Markers", "Net wrench"},
  //     Arrow(
  //         "Measured_LeftHandForce", netWrenchForceArrowConfig, [this]() -> Eigen::Vector3d { return
  //         leftHandAdmittancePtr_->surfacePose().translation(); }, [this, FORCE_SCALE]() -> Eigen::Vector3d {
  //           return leftHandAdmittancePtr_->surfacePose().translation() + FORCE_SCALE *
  //           leftHandAdmittancePtr_->measuredWrench().force();
  //         }));

  // for(const auto footTask : footTasks)
  // {
  //   auto footT = footTask.second;
  //   gui.addElement({"Tasks", name_, "Markers", "Foot wrenches"},
  //                  Point3D("Stabilizer_" + footT->surface() + "CoP", PointConfig(Color::Magenta, 0.01),
  //                          [footT]() { return footT->targetCoPW(); }),
  //                  Force(
  //                      "Measured_" + footT->surface() + "CoPForce", copForceConfig,
  //                      [footT, this]() {
  //                        return
  //                        robot().indirectSurfaceForceSensor(footT->surface()).worldWrenchWithoutGravity(robot());
  //                      },
  //                      [footT]() { return sva::PTransformd(footT->measuredCoPW()); }));
  // }
}

EXPORT_SINGLE_STATE("RobotHolding", RobotHolding)
