#include "RobotStabilizer.h"

// #include "../HelpUpController.h"
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace constants = mc_rtc::constants;
using ContactState = mc_tasks::lipm_stabilizer::ContactState;

void RobotStabilizer::configure(const mc_rtc::Configuration & config)
{
  if(config.has("completion"))
  {
    hasCompletion_ = !config("completion").empty();
    config("completion")("dcmEval", dcmThreshold_);
  }
  config("optionalGUI", optionalGUI_);

  config_.load(config);
}

void RobotStabilizer::start(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);

  if(!config_.has("StabilizerConfig"))
  {
    config_.add("StabilizerConfig");
  }
  config_("StabilizerConfig").add("type", "lipm_stabilizer");

  config_("stiffness", K_);
  D_ = config_("damping", 2 * std::sqrt(K_));

  // create stabilizer task from config
  // stabilizerTask_ = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
  //                   ctl.solver().robots(),
  //                   ctl.solver().realRobots(),
  //                   ctl.robot().robotIndex(),
  //                   ctl.timeStep
  //                   );

  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(
      ctl.solver(), config_("StabilizerConfig"));

  robot_ = stabilizerTask_->robot().name();
  auto & robot = ctl.robot(robot_);
  anchorFrameFunction_ = config_("anchorFrameFunction", "KinematicAnchorFrame::" + robot_);
  ctl.solver().addTask(stabilizerTask_);

  // Initialize stabilizer targets. Defaults to current CoM/CoP
  config_("comHeight", stabilizerTask_->config().comHeight);
  // Reset linear inverted pendulum model, used here to compute stabilizer references
  double lambda = constants::GRAVITY / stabilizerTask_->config().comHeight;
  pendulum_.reset(lambda, robot.com(), robot.comVelocity(), robot.comAcceleration());

  if(config_.has("ExternalWrenchConfig"))
  {
    config_("ExternalWrenchConfig")("addExpectedCoMOffset", ExternalWrenchConf_.addExpectedCoMOffset);
    config_("ExternalWrenchConfig")("modifyCoMErr", ExternalWrenchConf_.modifyCoMErr);
    config_("ExternalWrenchConfig")("com_offset_cutoff", ExternalWrenchConf_.comOffsetLowPassCoMCutoffPeriod);
    config_("ExternalWrenchConfig")("modifyZMPErr", ExternalWrenchConf_.modifyZMPErr);
    config_("ExternalWrenchConfig")("com_offset_com_cutoff", ExternalWrenchConf_.comOffsetLowPassCutoffPeriod);
    config_("ExternalWrenchConfig")("modifyZMPErrD", ExternalWrenchConf_.modifyZMPErrD);
    config_("ExternalWrenchConfig")("substractMeasuredValue", ExternalWrenchConf_.subtractMeasuredValue);
    config_("ExternalWrenchConfig")("ext_wrench_sum_cutoff", ExternalWrenchConf_.extWrenchSumLowPassCutoffPeriod);
    config_("ExternalWrenchConfig")("excludeFromDCMBiasEst", ExternalWrenchConf_.excludeFromDCMBiasEst);
  }
  stabilizerTask_->externalWrenchConfiguration(ExternalWrenchConf_);

  if(config_.has("DCMBiasEstimatorConfig"))
  {
    config_("DCMBiasEstimatorConfig")("withDCMBias", DCMBiasConf_.withDCMBias);
    config_("DCMBiasEstimatorConfig")("withDCMFilter", DCMBiasConf_.withDCMFilter);
    config_("DCMBiasEstimatorConfig")("correctCoMPos", DCMBiasConf_.correctCoMPos);
  }
  stabilizerTask_->dcmBiasEstimatorConfiguration(DCMBiasConf_);

  if(config_.has("above"))
  {
    setAboveObjective(config_("above"), ctl);
  }
  else if(config_.has("com"))
  {
    targetCoM(config_("com"));
  }
  else
  {
    targetCoM(robot.com());
  }

  // Fixme: the stabilizer needs the observed state immediatly
  if(ctl.datastore().has(anchorFrameFunction_))
  {
    mc_rtc::log::warning("[{}] a datastore callback for \"{}\" already exist on the datastore, using it instead",
                         name(), anchorFrameFunction_);
    ownsAnchorFrameCallback_ = false;
  }
  else
  {
    ctl.datastore().make_call(anchorFrameFunction_,
                              [this](const mc_rbdyn::Robot & robot) { return stabilizerTask_->anchorFrame(robot); });
    ownsAnchorFrameCallback_ = true;
  }

  if(optionalGUI_ && stabilizerTask_->inDoubleSupport())
  {
    ctl.gui()->addElement(
        {"FSM", name(), "Move"}, mc_rtc::gui::ElementsStacking::Horizontal,
        mc_rtc::gui::Button("Left foot", [this]()
                            { targetCoP(stabilizerTask_->contactAnklePose(ContactState::Left).translation()); }),
        mc_rtc::gui::Button("Center",
                            [this]()
                            {
                              targetCoP(sva::interpolate(stabilizerTask_->contactAnklePose(ContactState::Left),
                                                         stabilizerTask_->contactAnklePose(ContactState::Right), 0.5)
                                            .translation());
                            }),
        mc_rtc::gui::Button("Right foot", [this]()
                            { targetCoP(stabilizerTask_->contactAnklePose(ContactState::Right).translation()); }));
    ctl.gui()->addElement({"FSM", name(), "Move"},
                          mc_rtc::gui::ArrayInput(
                              "CoM Target", [this]() -> const Eigen::Vector3d & { return comTarget_; },
                              [this](const Eigen::Vector3d & com) { targetCoM(com); }),
                          mc_rtc::gui::ArrayInput(
                              "Move CoM", []() -> Eigen::Vector3d { return Eigen::Vector3d::Zero(); },
                              [this](const Eigen::Vector3d & com) { targetCoM(comTarget_ + com); }));
  }

  ctl.gui()->addElement({"FSM", name(), "Gains"},
                        mc_rtc::gui::NumberInput(
                            "CoM stiffness", [this]() { return K_; }, [this](const double & s) { K_ = s; }),
                        mc_rtc::gui::NumberInput(
                            "CoM damping", [this]() { return D_; }, [this](const double & d) { D_ = d; }),
                        mc_rtc::gui::NumberInput(
                            "CoM stiffness & damping", [this]() { return K_; },
                            [this](const double & g)
                            {
                              K_ = g;
                              D_ = 2 * std::sqrt(K_);
                            }));

#define LOG_MEMBER(NAME, MEMBER) MC_RTC_LOG_HELPER(name() + NAME, MEMBER)
  auto & logger = ctl.logger();
  LOG_MEMBER("_stiffness", K_);
  LOG_MEMBER("_damping", D_);
  LOG_MEMBER("_targetCoM", comTarget_);
  LOG_MEMBER("_targetCoP", copTarget_);
#undef LOG_MEMBER

  // Provide accessor callbacks on the datastore
  ctl.datastore().make_call("RobotStabilizer::getCoMTarget",
                            [this]() -> const Eigen::Vector3d & { return comTarget_; });
  ctl.datastore().make_call("RobotStabilizer::setCoMTarget",
                            [this](const Eigen::Vector3d & com) { this->targetCoM(com); });
  ctl.datastore().make_call("RobotStabilizer::setStiffness", [this](double K) { this->K_ = K; });
  ctl.datastore().make_call("RobotStabilizer::setDamping", [this](double D) { this->D_ = D; });
  ctl.datastore().make_call("RobotStabilizer::getStiffness", [this]() { return K_; });
  ctl.datastore().make_call("RobotStabilizer::getDamping", [this]() { return D_; });
  ctl.datastore().make_call("RobotStabilizer::getConfiguration",
                            [this]() -> mc_rbdyn::lipm_stabilizer::StabilizerConfiguration
                            { return stabilizerTask_->config(); });
  ctl.datastore().make_call("RobotStabilizer::setConfiguration",
                            [this](const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & conf)
                            { stabilizerTask_->configure(conf); });
  ctl.datastore().make_call("RobotStabilizer::setPelvisWeight", [this](double w) { stabilizerTask_->pelvisWeight(w); });
  ctl.datastore().make_call("RobotStabilizer::setPelvisStiffness",
                            [this](double s) { stabilizerTask_->pelvisStiffness(s); });
  ctl.datastore().make_call("RobotStabilizer::setTorsoWeight", [this](double w) { stabilizerTask_->torsoWeight(w); });
  ctl.datastore().make_call("RobotStabilizer::setTorsoStiffness",
                            [this](double s) { stabilizerTask_->torsoStiffness(s); });
  ctl.datastore().make_call("RobotStabilizer::setCoMWeight", [this](double w) { stabilizerTask_->comWeight(w); });
  ctl.datastore().make_call("RobotStabilizer::setCoMStiffness",
                            [this](const Eigen::Vector3d & s) { stabilizerTask_->comStiffness(s); });
  ctl.datastore().make_call("RobotStabilizer::setExternalWrenches",
                            [this](const std::vector<std::string> & surfaceNames,
                                   const std::vector<sva::ForceVecd> & targetWrenches,
                                   const std::vector<sva::MotionVecd> & gains)
                            { stabilizerTask_->setExternalWrenches(surfaceNames, targetWrenches, gains); });
  ctl.datastore().make_call("RobotStabilizer::setExternalWrenchConfiguration",
                            [this](const mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration & extWrenchConfig)
                            { stabilizerTask_->externalWrenchConfiguration(extWrenchConfig); });
  ctl.datastore().make_call("RobotStabilizer::setDCMBiasConfiguration",
                            [this](const mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration & dcmBiasConfig)
                            { stabilizerTask_->dcmBiasEstimatorConfiguration(dcmBiasConfig); });
  ctl.datastore().make_call("RobotStabilizer::getCoPAdmittance",
                            [this]() { return stabilizerTask_->config().copAdmittance; });
  ctl.datastore().make_call("RobotStabilizer::setCoPAdmittance", [this](const Eigen::Vector2d & copAdmittance)
                            { stabilizerTask_->copAdmittance(copAdmittance); });
  ctl.datastore().make_call("RobotStabilizer::getTask", [this]() { return stabilizerTask_; });
  ctl.datastore().make_call("RobotStabilizer::setDCMThreshold", [this](Eigen::Vector3d dcmThreshold, bool hasCompletion)
                            { setDCMThreshold(dcmThreshold, hasCompletion); });
  ctl.datastore().make_call("RobotStabilizer::setAboveObjective",
                            [this, &ctl](const mc_rtc::Configuration aboveConf) { setAboveObjective(aboveConf, ctl); });
  ctl.datastore().make_call("RobotStabilizer::isBalanced", [this]() { return isBalanced_; });
  ctl.datastore().make<bool>("RobotStabilizer::ManualMode", manual_);
  ctl.datastore().make_call("RobotStabilizer::getMeasuredDCM", [this]() { return stabilizerTask_->measuredDCM(); });
}

void RobotStabilizer::setDCMThreshold(Eigen::Vector3d dcmThreshold, bool hasCompletion)
{
  dcmThreshold_ = dcmThreshold;
  hasCompletion_ = hasCompletion;
}

void RobotStabilizer::setAboveObjective(mc_rtc::Configuration aboveConf, mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto & robot = ctl.robot(robot_);
  const std::string above = aboveConf;
  if(above == "LeftAnkle")
  {
    targetCoP(stabilizerTask_->contactAnklePose(ContactState::Left).translation());
  }
  else if(above == "RightAnkle")
  {
    targetCoP(stabilizerTask_->contactAnklePose(ContactState::Right).translation());
  }
  else if(above == "CenterAnkles")
  {
    targetCoP(sva::interpolate(stabilizerTask_->contactAnklePose(ContactState::Left),
                               stabilizerTask_->contactAnklePose(ContactState::Right), 0.5)
                  .translation());
  }
  else if(above == "LeftSurface")
  {
    mc_rtc::log::info("targetting left surface");
    targetCoP(robot.surfacePose(stabilizerTask_->footSurface(ContactState::Left)).translation());
  }
  else if(above == "RightSurface")
  {
    targetCoP(robot.surfacePose(stabilizerTask_->footSurface(ContactState::Right)).translation());
  }
  else if(above == "CenterSurfaces")
  {
    targetCoP(sva::interpolate(ctl.robot().surfacePose(stabilizerTask_->footSurface(ContactState::Left)),
                               ctl.robot().surfacePose(stabilizerTask_->footSurface(ContactState::Right)), 0.5)
                  .translation());
  }
  else if(robot.hasSurface(above))
  {
    targetCoP(robot.surfacePose(above).translation());
  }
  else
  {
    mc_rtc::log::error_and_throw(
        "[RobotStabilizer] Requested standing above {} but this is neither one of the state target "
        "(LeftAnkle, RightAnkle, CenterAnkles, LeftSurface, RightSurface, CenterSurfaces), nor a valid robot surface "
        "name",
        above);
  }
}

void RobotStabilizer::targetCoP(const Eigen::Vector3d & cop)
{
  comTarget_ = cop + Eigen::Vector3d{0., 0., stabilizerTask_->config().comHeight};
  copTarget_ = cop;
}

void RobotStabilizer::targetCoM(const Eigen::Vector3d & com)
{
  double copHeight = 0;
  if(stabilizerTask_->inDoubleSupport())
  {
    copHeight = (stabilizerTask_->contactAnklePose(ContactState::Left).translation().z()
                 + stabilizerTask_->contactAnklePose(ContactState::Right).translation().z())
                / 2;
  }
  else if(stabilizerTask_->inContact(ContactState::Left))
  {
    copHeight = stabilizerTask_->contactAnklePose(ContactState::Left).translation().z();
  }
  else
  {
    copHeight = stabilizerTask_->contactAnklePose(ContactState::Right).translation().z();
  }

  comTarget_ = com;
  copTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), copHeight};
}

void RobotStabilizer::targetDCM(const Eigen::Vector3d & dcm)
{
  dcmTarget_ = dcm;
}

bool RobotStabilizer::run(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);

  // getting current measured pendulum state
  const Eigen::Vector3d & com_ = pendulum_.com();
  const Eigen::Vector3d & comd_ = pendulum_.comd();

  // computing desired com acceleration from error between stabilizer state target and current pendulum com movement
  Eigen::Vector3d comdd = K_ * (comTarget_ - com_) - D_ * comd_;
  // computing vertical acceleration + weird pendulum stiffness factor (from vhip)
  Eigen::Vector3d n = constants::vertical;
  double lambda = n.dot(comdd + constants::gravity) / n.dot(com_ - copTarget_);
  // computing zmp (2d)
  Eigen::Vector3d zmp = com_ - (constants::gravity + comdd) / lambda;
  // integrate pendulum movement over timestep -> updates pendulum com, comd, comdd, zmp, zmpd
  pendulum_.integrateIPM(zmp, lambda, ctl.timeStep);

  // Update stabilizer task target with computed pendulum dynamics
  stabilizerTask_->target(pendulum_.com(), pendulum_.comd(), pendulum_.comdd(), pendulum_.zmp());
  // stabilizerTask_->staticTarget(comTarget_);

  // if(!hasCompletion_)
  // {
  //   output("OK");
  //   return true;
  // }
  const auto & dcm = stabilizerTask_->measuredDCM();
  if((((dcm - comTarget_).cwiseAbs() - dcmThreshold_).array() < 0.).all())
  {
    isBalanced_ = true;
    output("OK");
    return true;
  }
  isBalanced_ = false;
  return false;
}

void RobotStabilizer::teardown(mc_control::fsm::Controller & ctl)
{
  // auto & ctl = static_cast<HelpUpController &>(ctl_);

  ctl.solver().removeTask(stabilizerTask_);
  ctl.logger().removeLogEntries(this);

  // ctl.datastore().remove("RobotStabilizer::getCoMTarget");
  // ctl.datastore().remove("RobotStabilizer::setCoMTarget");
  // ctl.datastore().remove("RobotStabilizer::getStiffness");
  // ctl.datastore().remove("RobotStabilizer::setStiffness");
  // ctl.datastore().remove("RobotStabilizer::getDamping");
  // ctl.datastore().remove("RobotStabilizer::setDamping");
  // ctl.datastore().remove("RobotStabilizer::getConfiguration");
  // ctl.datastore().remove("RobotStabilizer::setConfiguration");
  // ctl.datastore().remove("RobotStabilizer::setPelvisWeight");
  // ctl.datastore().remove("RobotStabilizer::setPelvisStiffness");
  // ctl.datastore().remove("RobotStabilizer::setTorsoWeight");
  // ctl.datastore().remove("RobotStabilizer::setTorsoStiffness");
  // ctl.datastore().remove("RobotStabilizer::setCoMWeight");
  // ctl.datastore().remove("RobotStabilizer::setCoMStiffness");
  // ctl.datastore().remove("RobotStabilizer::setExternalWrenches");
  // ctl.datastore().remove("RobotStabilizer::getCoPAdmittance");
  // ctl.datastore().remove("RobotStabilizer::setCoPAdmittance");
  // ctl.datastore().remove("RobotStabilizer::setExternalWrenchConfiguration");
  // ctl.datastore().remove("RobotStabilizer::getTask");
  // ctl.datastore().remove("RobotStabilizer::setDCMThreshold");
  // ctl.datastore().remove("RobotStabilizer::setAboveObjective");
  // ctl.datastore().remove("RobotStabilizer::isBalanced");
  if(ownsAnchorFrameCallback_)
  {
    ctl.datastore().remove(anchorFrameFunction_);
  }
}

EXPORT_SINGLE_STATE("RobotStabilizer", RobotStabilizer)
