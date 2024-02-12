#include "RobotHolding.h"

#include <mc_solver/TasksQPSolver.h>

#include <mc_control/fsm/Controller.h>

void RobotHolding::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void RobotHolding::start(mc_control::fsm::Controller & ctl)
{

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
    // critically damped impedance has D = 2*sqrt(K*M)
    // if damping not manually given, compute it from the others
    // update: multiplying D by 2 again because too low otherwise (other spring damper systems conflict)
    if(!config_("RightHandImped")("gains").has("damper"))
    {
      Eigen::Vector6d damping;
      for(int i = 0; i < 6; i++)
      {
        // damping(i) = 2 * 2*sqrt(rightHandImpedancePtr_->gains().spring().vector()(i) *
        // rightHandImpedancePtr_->gains().mass().vector()(i));
        damping(i) = 2 * rightHandImpedancePtr_->gains().spring().vector()(i);
      }
      rightHandImpedancePtr_->gains().damper() = damping;
    }
    // rightHandImpedancePtr_->targetWrench(sva::ForceVecd::Zero());
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
    // critically damped impedance has D = 2*sqrt(K*M)
    // if damping not manually given, compute it from the others
    if(!config_("LeftHandImped")("gains").has("damper"))
    {
      Eigen::Vector6d damping;
      for(int i = 0; i < 6; i++)
      {
        // damping(i) = 2 * 2*sqrt(leftHandImpedancePtr_->gains().spring().vector()(i) *
        // leftHandImpedancePtr_->gains().mass().vector()(i));
        damping(i) = 2 * leftHandImpedancePtr_->gains().spring().vector()(i);
      }
      leftHandImpedancePtr_->gains().damper() = damping;
    }
    // leftHandImpedancePtr_->targetWrench(sva::ForceVecd::Zero());
    ctl.solver().addTask(leftHandImpedancePtr_);
  }

  // adding tasks to solver
  // addToGUI(*ctl.gui(), ctl);
}

bool RobotHolding::run(mc_control::fsm::Controller & ctl)
{

  // getting ref vel from human back, to be transformed in objective frame
  auto backRefVel = ctl.robot("human").frame("Back").velocity();
  auto X_0_back = ctl.robot("human").frame("Back").position();
  // getting ref acc same principle
  auto backRefAcc = sva::PTransformd(X_0_back.rotation()).inv()
                    * ctl.robot("human").bodyAccB("TorsoLink"); // is this transformed correctly? should it be body
                                                                // into world into frame instead of body into frame?
  backRefAcc.linear() += backRefVel.angular().cross(backRefVel.linear()); // coriolis term

  std::vector<sva::MotionVecd> gainsVec;
  std::vector<std::string> surfVec;
  std::vector<sva::ForceVecd> wrenchVec;
  auto & changeMode = ctl.datastore().get<bool>("HelpUp::ChangeMode");
  if(changeMode)
  {
    if(ctl.datastore().call<bool>("HelpUp::ForceMode"))
    {
      mc_rtc::log::info("[{}] : changing force mode to computed wrenches", name());
      if(config_.has("RightHandImped"))
      {
        rightHandImpedancePtr_->gains().wrench() = config_("RightHandImped")("gains")("wrench");
      }
      if(config_.has("LeftHandImped"))
      {
        leftHandImpedancePtr_->gains().wrench() = config_("LeftHandImped")("gains")("wrench");
      }
    }
    else
    {
      mc_rtc::log::info("[{}] : changing force mode to follow only", name());
      if(config_.has("RightHandImped"))
      {
        rightHandImpedancePtr_->gains().wrench() = Eigen::Vector6d::Zero().eval();
      }
      if(config_.has("LeftHandImped"))
      {
        leftHandImpedancePtr_->gains().wrench() = Eigen::Vector6d::Zero().eval();
      }
    }
    changeMode = false;
  }

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
    auto X_0_RHtarget = RHtargetOffset_ * ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).position();
    // getting ref vel from objective frame (already in world frame)
    auto RHRefVel = ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).velocity();
    // getting ref acc
    auto X_pBody_RHtarget = ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).X_b_f();
    auto pBodyRHName = ctl.robot(RHtargetRobot_).frame(RHtargetFrame_).body();
    // no accessor for frame acceleration -> getting body acc of parent (/!\ it is a SPATIAL acc, so need to add
    // centrifugal term), transforming it back to target frame, then orient it in world frame to check: is bodyAccB in
    // world frame? it says inertial frame but it should be the same thing
    auto RHRefAcc = sva::PTransformd(X_0_RHtarget.inv().rotation()) * X_pBody_RHtarget
                    * ctl.robot(RHtargetRobot_).bodyAccB(pBodyRHName);
    // centrifugal term: appears because this is not acc of the body but of a point attached to the body
    RHRefAcc.linear() += RHRefVel.angular().cross(RHRefVel.linear());

    // setting reference trajectory of target frame
    rightHandImpedancePtr_->targetPose(X_0_RHtarget);
    rightHandImpedancePtr_->targetVel(RHRefVel);
    rightHandImpedancePtr_->targetAccel(RHRefAcc);

    // setting target wrenches to apply on target frame
    if(ctl.datastore().call<bool>("HelpUp::ForceMode"))
    {
      rightHandImpedancePtr_->targetWrench(ctl.datastore().call<sva::ForceVecd>("HelpUp::ComputedRHWrench"));
      RHgains_ = sva::MotionVecd(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));
      gainsVec.push_back(RHgains_);
      surfVec.push_back(config_("RightHandImped")("frame"));
      wrenchVec.push_back(ctl.datastore().call<sva::ForceVecd>("HelpUp::ComputedRHWrench"));
    }
    else
    {
      rightHandImpedancePtr_->targetWrench(sva::ForceVecd(Eigen::Vector3d(0., 0., 0.), Eigen::Vector3d(0., 0., 0.)));
      RHgains_ = sva::MotionVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      gainsVec.push_back(RHgains_);
      surfVec.push_back(config_("RightHandImped")("frame"));
      wrenchVec.push_back(sva::ForceVecd::Zero());
    }
  }
  if(config_.has("LeftHandImped"))
  {
    auto X_0_LHtarget = LHtargetOffset_ * ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).position();
    // getting ref vel from objective frame (already in world frame)
    auto LHRefVel = ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).velocity();
    // getting ref acc
    auto X_pBody_LHtarget = ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).X_b_f();
    auto pBodyLHName = ctl.robot(LHtargetRobot_).frame(LHtargetFrame_).body();
    // no accessor for frame acceleration -> getting body acc of parent (/!\ it is a SPATIAL acc, so need to add
    // centrifugal term), transforming it back to target frame, then orient it in world frame
    auto LHRefAcc = sva::PTransformd(X_0_LHtarget.inv().rotation()) * X_pBody_LHtarget
                    * ctl.robot(LHtargetRobot_).bodyAccB(pBodyLHName);
    LHRefAcc.linear() += LHRefVel.angular().cross(LHRefVel.linear()); // centrifugal term

    // setting reference trajectory of target frame
    leftHandImpedancePtr_->targetPose(X_0_LHtarget);
    leftHandImpedancePtr_->targetVel(LHRefVel);
    leftHandImpedancePtr_->targetAccel(LHRefAcc);

    // setting target wrenches to apply on target frame
    if(ctl.datastore().call<bool>("HelpUp::ForceMode"))
    {
      leftHandImpedancePtr_->targetWrench(ctl.datastore().call<sva::ForceVecd>("HelpUp::ComputedLHWrench"));
      LHgains_ = sva::MotionVecd(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1));
      gainsVec.push_back(LHgains_);
      surfVec.push_back(config_("LeftHandImped")("frame"));
      wrenchVec.push_back(ctl.datastore().call<sva::ForceVecd>("HelpUp::ComputedLHWrench"));
    }
    else
    {
      leftHandImpedancePtr_->targetWrench(sva::ForceVecd(Eigen::Vector3d(0., 0., 0.), Eigen::Vector3d(0., 0., 0.)));
      LHgains_ = sva::MotionVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      gainsVec.push_back(LHgains_);
      surfVec.push_back(config_("LeftHandImped")("frame"));
      wrenchVec.push_back(sva::ForceVecd::Zero());
    }
  }

  if(ctl.datastore().call<bool>("HelpUp::ForceMode"))
  {
    ctl.datastore().call("RobotStabilizer::setExternalWrenches", static_cast<const std::vector<std::string> &>(surfVec),
                         static_cast<const std::vector<sva::ForceVecd> &>(wrenchVec),
                         static_cast<const std::vector<sva::MotionVecd> &>(gainsVec));
  }
  else
  {
    ctl.datastore().call("RobotStabilizer::setExternalWrenches", static_cast<const std::vector<std::string> &>(surfVec),
                         static_cast<const std::vector<sva::ForceVecd> &>(wrenchVec),
                         static_cast<const std::vector<sva::MotionVecd> &>(gainsVec));
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

void RobotHolding::teardown(mc_control::fsm::Controller & ctl)
{

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

void RobotHolding::addToGUI(mc_rtc::gui::StateBuilder & gui, mc_control::fsm::Controller & ctl)
{
  using namespace mc_rtc::gui;
  using Style = mc_rtc::gui::plot::Style;

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
