#include "HelpUpController.h"
#include <mc_control/SimulationContactPair.h>
#include <mc_observers/ObserverPipeline.h>
#include <mc_panda/devices/Robot.h>
#include <mc_rtc/gui/plot.h>
#include <mc_solver/CoMIncPlaneConstr.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include "MCStabilityPolytope.h"
#include "config.h"

// allows to overwrite robot module with a controller configuration to constraint some joint limits
static inline mc_rbdyn::RobotModulePtr patch_rm(mc_rbdyn::RobotModulePtr rm, const mc_rtc::Configuration & config)
{
  auto limits = config("Limits", mc_rtc::Configuration{})(rm->name, mc_rtc::Configuration{})
                    .
                operator std::map<std::string, mc_rtc::Configuration>();
  for(const auto & [joint, overwrite] : limits)
  {
    if(overwrite.has("lower"))
    {
      rm->_bounds[0].at(joint)[0] = overwrite("lower").operator double();
    }
    if(overwrite.has("upper"))
    {
      rm->_bounds[1].at(joint)[0] = overwrite("upper").operator double();
    }
  }
  return rm;
}

HelpUpController::HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(patch_rm(rm, config),
                              dt,
                              config,
                              mc_control::ControllerParameters{}.load_robot_config_into({}).overwrite_config(true)),
  robotPolytope_(robot().name()), humanPolytope_("human"), lowPassPolyCenter_(dt, cutoffPeriodPolyCenter_),
  accLowPass_(dt, cutoffPeriod_), lowPassLB_(dt, cutoffPeriodForceShoes_), lowPassRB_(dt, cutoffPeriodForceShoes_),
  lowPassLF_(dt, cutoffPeriod_), lowPassRF_(dt, cutoffPeriodForceShoes_)
{
  // Load entire controller configuration file
  config_.load(config);
  // trajectories_ = std::make_shared<TrajectoryModel> (/*path*/);
  config_("wrenchDistributionTarget", wrenchDistributionTarget_);
  config_("handContactsForBalance", handContactsForBalance_);
  auto measuredPerson = config_("measuredPerson");
  measuredPerson("hipHeight", hipHeight_);
  measuredPerson("mass", humanMass_);
  measuredPerson("withSuit", withSuit_);
  measuredPerson("withShoes", withShoes_);
  measuredPerson("withLegs", withLegs_);
  measuredPerson("withWrists", withWrists_);

  humanPolytope_.load(config("StabilityPolytope")("human"));
  robotPolytope_.load(config("StabilityPolytope")(robot().name()));

  if(withSuit_) humanMass_ += vestMass_;
  if(withLegs_) humanMass_ += (2 * legsMass_);
  if(withShoes_) humanMass_ += (2 * shoesMass_);
  if(withWrists_) humanMass_ += (2 * wristsMass_);

  /* Observers
   */
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot & robot) { // robot() is the main robot (hrp4)
                          return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
                        });

  comIncPlaneConstraintPtr_ =
      std::make_shared<mc_solver::CoMIncPlaneConstr>(realRobots(), realRobots().robotIndex(), dt);
  comIncPlaneConstraintHumPtr_ =
      std::make_shared<mc_solver::CoMIncPlaneConstr>(robots(), robots().robotIndex("human"), dt);

  // init DCM objective
  DCMobjective_ = robot("human").com();
  robDCMobjective_ = robot().com();
  robDCMTarget_ = robot().com();
  robMeasuredDCM_ = robot().com();

  comDesired_ = robot().com();
  comDesiredHum_ = robot("human").com();

  // surfaces pointers for live contact sets
  const auto & human_surfaces = robot("human").surfaces();
  RFootSurf = human_surfaces.at("RightSole");
  LFootSurf = human_surfaces.at("LeftSole");
  RCheekSurf = human_surfaces.at("RCheek");
  LCheekSurf = human_surfaces.at("LCheek");
  RBackLegSurf = human_surfaces.at("RLegBack");
  LBackLegSurf = human_surfaces.at("LLegBack");
  RightShoulderSurf = human_surfaces.at("RightShoulder");
  BackSurf = human_surfaces.at("Back");

  TopSurf = robot("chair").surfaces().at("Top");
  GroundSurf = robot("ground").surfaces().at("AllGround");

  // RHandSurf = realRobot("e2dr").surfaces().at("RightHand");
  // LHandSurf = realRobot("e2dr").surfaces().at("LeftHand");
  // RHandSurf = realRobot().surfaces().at("RightHand");
  // LHandSurf = realRobot().surfaces().at("LeftHand");

  RCheekChair = std::make_shared<mc_control::SimulationContactPair>(RCheekSurf, TopSurf);
  LCheekChair = std::make_shared<mc_control::SimulationContactPair>(LCheekSurf, TopSurf);

  RBackLegChair = std::make_shared<mc_control::SimulationContactPair>(RBackLegSurf, TopSurf);
  LBackLegChair = std::make_shared<mc_control::SimulationContactPair>(LBackLegSurf, TopSurf);
  RFootGround = std::make_shared<mc_control::SimulationContactPair>(RFootSurf, GroundSurf);
  LFootGround = std::make_shared<mc_control::SimulationContactPair>(LFootSurf, GroundSurf);
  // RHandShoulder = std::make_shared<mc_control::SimulationContactPair>(RHandSurf, RightShoulderSurf);
  // LHandBack = std::make_shared<mc_control::SimulationContactPair>(LHandSurf, BackSurf);

  if(config_.has("Omega"))
  {
    OmegaZAcc_ = config_("Omega")("WithVerticalAcc");
  }

  if(config_.has("filteredDerivation"))
  {
    FilteredDerivation_ = config_("filteredDerivation");
  }

  // initialize DCM tracker for the human and for the main robot
  humanDCMTracker_ = std::make_shared<DCM_VRPtracker>(dt, cutoffPeriod_, humanMass_, DCMpropgain_, DCMinteggain_);
  robotDCMTracker_ = std::make_shared<DCM_VRPtracker>(dt, cutoffPeriod_, robot().mass(), DCMpropgain_, DCMinteggain_);

  datastore().make_call("HelpUp::ForceMode", [this]() { return computedForceMode_; });
  datastore().make<bool>("HelpUp::ChangeMode", true);
  datastore().make_call("HelpUp::ComputedLHWrench", [this]() { return getLHWrenchComputed(); });
  datastore().make_call("HelpUp::ComputedRHWrench", [this]() { return getRHWrenchComputed(); });
  // scale robot height to human or not (false at first)
  datastore().make<bool>("HelpUp::scaleRobotCoMZ", false);
  datastore().make<bool>("HelpUp::scaleRobotCoMLateral", false);

  // accLowPass_.dt(dt);
  // accLowPass_.cutoffPeriod(cutoffPeriod_);

  addLogEntries();
  addGuiElements();
  addTasksToSolver();

  mc_rtc::log::success("HelpUpController init done ");
}

bool HelpUpController::run()
{

  LFShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetLFForce");
  LBShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetLBForce");
  RFShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetRFForce");
  RBShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetRBForce");

  // XXX datastore log entries to display force target and measured at contact
  // CAREFUL here should add offset if any because not replay of real hands positions
  if(datastore().has("ReplayPlugin::LHWrench") && datastore().has("ReplayPlugin::TargetLH"))
  {
    LHForceLog_ = datastore().get<sva::ForceVecd>("ReplayPlugin::LHWrench");
    RHForceLog_ = datastore().get<sva::ForceVecd>("ReplayPlugin::RHWrench");
    LHtargetForceLog_ = datastore().get<sva::ForceVecd>("ReplayPlugin::TargetLH");
    RHtargetForceLog_ = datastore().get<sva::ForceVecd>("ReplayPlugin::TargetRH");
  }

  if(LFShoe_.vector().array().isNaN().any())
  {
    mc_rtc::log::error("[ForceShoes] Left front forces are NaN, using zeros instead");
    LFShoe_ = sva::ForceVecd::Zero();
  }
  if(LBShoe_.vector().array().isNaN().any())
  {
    mc_rtc::log::error("[ForceShoes] Left back forces are NaN, using zeros instead");
    LBShoe_ = sva::ForceVecd::Zero();
  }
  if(RFShoe_.vector().array().isNaN().any())
  {
    mc_rtc::log::error("[ForceShoes] Right front forces are NaN, using zeros instead");
    RFShoe_ = sva::ForceVecd::Zero();
  }
  if(RBShoe_.vector().array().isNaN().any())
  {
    mc_rtc::log::error("[ForceShoes] Right back forces are NaN, using zeros instead");
    RBShoe_ = sva::ForceVecd::Zero();
  }

  // Attempt at filtering shoes force sensors: should be in plugin
  lowPassLF_.update(LFShoe_);
  lowPassLB_.update(LBShoe_);
  lowPassRF_.update(RFShoe_);
  lowPassRB_.update(RBShoe_);

  LFShoe_ = lowPassLF_.eval();
  LBShoe_ = lowPassLB_.eval();
  RFShoe_ = lowPassRF_.eval();
  RBShoe_ = lowPassRB_.eval();

  // Sensors are left handed ? Y axis is inverted so we correct it here
  Eigen::Matrix3d Rr;
  Rr << 1, 0, 0, 0, -1, 0, 0, 0, 1;
  sva::PTransformd sensorFrame(Rr, Eigen::Vector3d::Zero());
  LFShoe_ = sensorFrame.dualMul(LFShoe_);
  LBShoe_ = sensorFrame.dualMul(LBShoe_);
  RFShoe_ = sensorFrame.dualMul(RFShoe_);
  RBShoe_ = sensorFrame.dualMul(RBShoe_);

  xsensCoMpos_ = datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMpos");
  xsensCoMvel_ = datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMvel");
  // xsensCoMacc_ = datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMacc");

  rawxsensCoMacc_ = datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMacc");
  accLowPass_.update(rawxsensCoMacc_);
  xsensCoMacc_ = accLowPass_.eval();

  humanDCMTracker_->setCoMDyn(xsensCoMpos_, xsensCoMvel_, xsensCoMacc_);
  // pass the measured forces and their respective frames to the tracker
  std::vector<std::pair<sva::PTransformd, sva::ForceVecd>> forceContacts;
  forceContacts.emplace_back(robot("human").frame("RFsensor").position(), RFShoe_);
  forceContacts.emplace_back(robot("human").frame("LFsensor").position(), LFShoe_);
  forceContacts.emplace_back(robot("human").frame("RBsensor").position(), RBShoe_);
  forceContacts.emplace_back(robot("human").frame("LBsensor").position(), LBShoe_);
  humanDCMTracker_->setAppliedForces(forceContacts);

  robotDCMTracker_->setCoMDyn(robot().com(), robot().comVelocity(), robot().comAcceleration());

  humanDCMTracker_->updateTrackedValues();
  robotDCMTracker_->updateTrackedValues();

  // Assume positions have been reset when first poly is computed (only ran once)
  if(firstPolyHumOK_ && !pandaTaskAdded_ && robots().hasRobot("panda"))
  {
    // pandaTransform_ = std::make_shared<mc_tasks::TransformTask>(robot("panda").frame("HumanBack"), 1, 1000);
    pandaForceConstrainedTransform_ =
        std::make_shared<mc_tasks::ForceConstrainedTransformTask>(robot("panda").frame("HumanBack"), 1, 1000);

    pandaForceConstrainedTransform_->defaultAdmittanceGain(0.01);
    pandaForceConstrainedTransform_->defaultSafetyDamping(0.01);
    // For some reason the 3 last from the dof vector actuate the 3 first of the ForceVecd
    // CAREFUL contraints are falsely constructed in the QP, selector matrix is resized too much if there are zeroes,
    // check dofToSelector function
    Eigen::Vector6d dof = -Eigen::Vector6d::Ones();
    pandaForceConstrainedTransform_->addFrameConstraint(robot("panda").frame("BoxCenter"), dof,
                                                        sva::ForceVecd({10, 10, 10}, {20.0, 20.0, 20.0}),
                                                        sva::ForceVecd({0.0, 0.0, 0.0}, {5.0, 5.0, 5.0}));

    dof = Eigen::Vector6d::Ones();
    pandaForceConstrainedTransform_->addFrameConstraint(robot("panda").frame("BoxCenter"), dof,
                                                        sva::ForceVecd({10, 10, 10}, {20.0, 20.0, 20.0}),
                                                        sva::ForceVecd({0.0, 0.0, 0.0}, {5.0, 5.0, 5.0}));
    // pandaTransform_->stiffness(sva::MotionVecd(Eigen::Vector3d(100, 100, 100), Eigen::Vector3d(10, 10, 10)));
    // solver().addTask(pandaTransform_);
    solver().addTask(pandaForceConstrainedTransform_);
    pandaTaskAdded_ = true;
  }

  // set target every run
  if(pandaTaskAdded_ && robots().hasRobot("panda"))
  {
    pandaForceConstrainedTransform_->targetSurface(robot("human").robotIndex(), "Back",
                                                   sva::PTransformd(Eigen::Vector3d(0, 0, -0.05)));
    // pandaTransform_->targetSurface(robot("human").robotIndex(), "Back", sva::PTransformd(Eigen::Vector3d(0, 0,
    // -0.05)));
    auto X_0_back = robot("human").frame("Back").position();
    auto backRefVel = robot("human").frame("Back").velocity();
    // transform from world frame to back frame
    backRefVel = sva::PTransformd(X_0_back.rotation()) * backRefVel;
    // pandaTransform_->refVelB(backRefVel);
    pandaForceConstrainedTransform_->refVelB(backRefVel);
    // getting ref acc same principle
    // auto backRefAcc = sva::PTransformd(X_back_0.rotation()) * robot("human").bodyAccB("TorsoLink");
    // backRefAcc.linear() += backRefVel.angular().cross(backRefVel.linear()); // coriolis term
    // pandaTransform_->refAccel(backRefAcc);
  }

  computePolytope(robMeasuredDCM_, firstPolyRobOK_, mainRob);

  /* We update the objective only if the first polytope at least was computed
  Then it is updated every control iteration using the last computed polytope
  */
  if(firstPolyRobOK_)
  {
    // XXX this causes drift as the com follows the measured value (in choreonoid)
    // for now we use the stabilizer computed DCM and not the VRP tracker
    robMeasuredDCM_ = datastore().call<Eigen::Vector3d>("RobotStabilizer::getMeasuredDCM");
    updateObjective(robotPolytope_, robMeasuredDCM_, robDCMobjective_, mainRob);
  }
  else
  {
    // This is to initialize the low pass near the com and not at zero (which makes the com objective move violently in
    // the beginning otherwise) combined to chebichev coeff and cutoff period of lowpass
    lowPassPolyCenter_.reset(robot().com());
  }

  // Same process with human
  computePolytope(humanDCMTracker_->getDCM(), firstPolyHumOK_, human);

  if(firstPolyHumOK_)
  {
    updateObjective(humanPolytope_, humanDCMTracker_->getDCM(), DCMobjective_, human);
  }

  if(contactSetHum_->numberOfContacts() / 4 > 2)
  {
    // If more than two contacts: then the rear is still in contact with the chair
    // Then we should use the model version using the CoM acceleration
    modelMode_ = true;
    humanDCMTracker_->setModelMode(true);
  }
  else
  {
    // two contacts or less: only feet contacts -> we can switch to force measurements
    modelMode_ = false;
    humanDCMTracker_->setModelMode(false);
  }

  humanDCMTracker_->updateObjectiveValues(DCMobjective_);
  robotDCMTracker_->updateObjectiveValues(robDCMobjective_);

  auto desiredCoMWrench = humanDCMTracker_->getMissingForces();
  distributeAssistiveHandsWrench(desiredCoMWrench, robot(), wrenchDistributionTarget_("targetRobot"),
                                 wrenchDistributionTarget_("helpSurfaceLH"),
                                 wrenchDistributionTarget_("helpSurfaceRH"));
  t_ += solver().dt();
  bool ok = mc_control::fsm::Controller::run();
  return ok;
}

void HelpUpController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  // Overwrite the low default interaction thresholds: this causes problems when running in mujoco
  if(robots().hasRobot("panda"))
  {

    mc_panda::Robot * robot_ptr = mc_panda::Robot::get(robot("panda")); // nullptr if not a panda
    if(robot_ptr)
    {
      mc_rtc::log::info("Robot {} has a Robot-device", robot("panda").name());
      robot_ptr->setJointImpedance(
          {{3000, 3000, 3000, 2500, 2500, 2000,
            2000}}); // values taken from
                     // https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L18
      // robot_ptr->setCollisionBehavior( //values taken from
      // https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
      //   {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      //   {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      //   {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      //   {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
      robot_ptr->setCollisionBehavior( // TODO: be careful with this mode!
          {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
          {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
          {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
          {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});
    }
  }
  /**
   * The observer pipeline for HRP4 will be initialized once the
   * position of all robots have been reset in ResetPoses state
   */
  datastore().make_call("LoadHRP4ObserverPipeline",
                        [this]()
                        {
                          if(config_.has("HRP4ObserverPipeline"))
                          {
                            auto pipelineConfig = config_("HRP4ObserverPipeline");
                            mc_rtc::log::info("Adding observer pipeline HRP4ObserverPipeline with config:\n{}",
                                              pipelineConfig.dump(true, true));
                            observerPipelines_.emplace_back(*this);
                            auto & pipeline = observerPipelines_.back();
                            pipeline.create(pipelineConfig, timeStep);
                            if(pipelineConfig("log", true))
                            {
                              pipeline.addToLogger(logger());
                            }
                            if(pipelineConfig("gui", false))
                            {
                              pipeline.addToGUI(*gui());
                            }
                            pipeline.reset();
                          }
                        });

  if(datastore().has("UDPPlugin"))
  { // When using the UDP plugin wait until hrp4 is ready
    datastore().make_call("UDPPlugin::" + robot().name() + "::reset",
                          [this]()
                          {
                            mc_rtc::log::warning("TestUDPPugin::reset called, resetting posture task");
                            datastore().make<bool>("HRP4IsReady", true);
                            getPostureTask(robot().name())->reset();
                          });
  }
  else
  { // In simulation we do not need to wait
    datastore().make<bool>("HRP4IsReady", true);
  }

  // Update dynamics constraints
  // dynamicsConstraint = mc_solver::DynamicsConstraint(robots(),
  //                                                    robot().robotIndex(),
  //                                                    solver().dt(), {0.1, 0.01, 0.5});
  // // Must be added to the solver before controller reset
  // solver().addConstraintSet(dynamicsConstraint);

  // // Add human dynamics constraints
  // if(robots().hasRobot("human")){
  //   humanDynamicsConstraint_ = mc_solver::DynamicsConstraint(robots(),
  //                                                      robots().robot("human").robotIndex(),
  //                                                      solver().dt(), {0.1, 0.01, 0.5});
  //   solver().addConstraintSet(humanDynamicsConstraint_);

  mc_rtc::log::success("running reset");

  // Update constraints and resets posture tasks
  // solver().updateConstrSize();
}

void HelpUpController::computePolytope(const Eigen::Vector3d & currentPos, bool & firstPolyOK, whatRobot rob)
{
  switch(rob)
  {
    case mainRob:
      updateContactSet(robot().robotIndex());
      updateContactForces();
      if(contactSet_->numberOfContacts() > 0)
      {
        robotPolytope_.update(contactSet_, currentPos);
      }
      firstPolyOK = robotPolytope_.computed();
      break;

    case human:
      updateRealHumContacts();
      if(contactSetHum_->numberOfContacts() > 0)
      {
        humanPolytope_.update(contactSetHum_, currentPos);
      }
      firstPolyOK = humanPolytope_.computed();
      break;
    default:
      break;
  }
}

void HelpUpController::updateObjective(MCStabilityPolytope & polytope_,
                                       Eigen::Vector3d currentPos,
                                       Eigen::Vector3d & objective,
                                       whatRobot rob)
{

  // Update objective to stabilizer or human assistance
  switch(rob)
  {
    case mainRob:
    {
      auto planes = polytope_.constraintPlanes();
      Eigen::Vector3d chebichev = polytope_.chebichevCenter();
      Eigen::Vector3d bary = polytope_.baryCenter();
      // lowpass filtering of center of polytope (very noisy from every computation)
      lowPassPolyCenter_.update(bary);
      bary = lowPassPolyCenter_.eval();
      Eigen::Vector3d filteredObjective = (1 - chebichevCoef_) * currentPos + chebichevCoef_ * bary;
      // filteredObjective = robDCMTarget_.translation();
      // lowPassPolyCenter_.update(chebichev);
      // chebichev = lowPassPolyCenter_.eval();
      // Eigen::Vector3d filteredObjective = (1 - chebichevCoef_) * currentPos + chebichevCoef_ * chebichev;
      auto heightScalingValues =
          config_.find("scaling_heights").value_or(mc_rtc::Configuration{}).find(robot().name()).value();
      if(datastore().get<bool>("HelpUp::scaleRobotCoMZ"))
      {
        double min = heightScalingValues("min");
        double max = heightScalingValues("max");
        filteredObjective.z() = std::clamp(robot("human").com().z(), min, max);
      }
      else
      {
        double nominal = heightScalingValues("nominal");
        filteredObjective.z() = nominal;
      }
      if(datastore().get<bool>("HelpUp::scaleRobotCoMLateral"))
      {
        auto lateralCoM =
            std::clamp(humanDCMTracker_->getDCM().x(), robot().frame("LeftFoot").position().translation().x(),
                       robot().frame("RightFoot").position().translation().x());
        filteredObjective.x() = lateralCoM;
      }
      else
      {
        filteredObjective.x() =
            sva::interpolate(robot().frame("LeftFoot").position(), robot().frame("RightFoot").position(), 0.5)
                .translation()
                .x();
      }
      objective = polytope_.objectiveInPolytope(filteredObjective);
      // objective = polytope_.objectiveInPolytope(objective);
      if(datastore().has("RobotStabilizer::setCoMTarget"))
      {
        // if the mode is not set to manual objective, update automatically using balance regions
        auto manual = datastore().get<bool>("RobotStabilizer::ManualMode");
        if(manual == false)
        {
          datastore().call<void, const Eigen::Vector3d &>("RobotStabilizer::setCoMTarget", objective);
        }
      }
      break;
    }

    case human:
    {
      Eigen::Vector3d prevObjective = objective;
      // writing objective by reference in DCMobjective_ var used in VRP control law
      objective = polytope_.objectiveInPolytope(currentPos);
      if(polytope_.configToChange_)
      {
        // XXX needed only if using uncolored polyhedron implementation
        // polytope_.removeFromGUI(*gui());
        // polytope_.addToGUI(*gui());
      }
      // prevent polytope projection to lower objective (from polytope form, closest point might be lower)
      if(objective.z() < currentPos.z())
      {
        objective.z() = currentPos.z();
      }
      // we do not authorize objective to go down while person is not standing
      // standing com typically about 10cm top of hips, we keep a 5cm margin
      // if(objective.z() < prevObjective.z() && objective.z() < hipHeight_ + 0.05)
      // {
      //   objective.z() = prevObjective.z();
      // }

      break;
    }

    default:
      break;
  }
}

bool HelpUpController::addTasksToSolver()
{
  // solver().addTask(comTask_); //We now use the stabilizer task to manage the com and not a simple com task
  // human com is done in custom state

  // solver().addTask(stabTask_);

  // solver().addConstraintSet(*comIncPlaneConstraintPtr_);
  planes_ = {};
  planes(planes_, mainRob); // todo update planes

  // solver().addConstraintSet(*comIncPlaneConstraintHumPtr_); // not added with real human
  planesHum_ = {};
  planes(planesHum_, human);

  postureTask->stiffness(0.1); // main robot (hrp4)
  posture_tasks_["human"]->stiffness(0.1);

  return true;
}

// bool HelpUpController::removeTasksFromSolver()
// {
//   solver().removeTask(comTask_);
//   return true;
// }

void HelpUpController::updateCombinedCoM()
{
  double total_mass = realRobot().mass() + realRobot("human").mass();
  combinedCoM_ = ((robot().com() * robot().mass()) + (robot("human").com() * robot("human").mass())) / total_mass;
  // (maybe better with com.cpp method from rbdyn)
}

void HelpUpController::addLogEntries()
{
  robotPolytope_.addToLogger(logger());
  humanPolytope_.addToLogger(logger());
  // logger().addLogEntry("polytope_computationTime", [this]() -> const int { return

  auto & logger = this->logger();
  humanDCMTracker_->addLogEntries("human", logger);
  robotDCMTracker_->addLogEntries("robot", logger);

  logger.addLogEntry("HelpUp_robot measured DCM", [this]() -> const Eigen::Vector3d & { return robMeasuredDCM_; });
  // MC_RTC_LOG_HELPER("HelpUp_robot measured DCM", robMeasuredDCM_);
  logger.addLogEntry("HelpUp_robot DCM objective", [this]() -> const Eigen::Vector3d & { return robDCMobjective_; });
  logger.addLogEntry("HelpUp_filtered cheb center",
                     [this]() -> const Eigen::Vector3d & { return lowPassPolyCenter_.eval(); });
  logger.addLogEntry("Xsens_Raw acc", [this]() -> const Eigen::Vector3d & { return rawxsensCoMacc_; });
  logger.addLogEntry("Xsens_Filtered acc", [this]() -> const Eigen::Vector3d & { return xsensCoMacc_; });
  logger.addLogEntry("ForceShoes_LFShoeMeasure", [this]() -> const sva::ForceVecd & { return LFShoe_; });
  logger.addLogEntry("ForceShoes_LBShoeMeasure", [this]() -> const sva::ForceVecd & { return LBShoe_; });
  logger.addLogEntry("ForceShoes_RFShoeMeasure", [this]() -> const sva::ForceVecd & { return RFShoe_; });
  logger.addLogEntry("ForceShoes_RBShoeMeasure", [this]() -> const sva::ForceVecd & { return RBShoe_; });
  logger.addLogEntry("HelpUp_ComputedLHwrench", [this]() -> const sva::ForceVecd & { return LHwrench_; });
  logger.addLogEntry("HelpUp_ComputedRHwrench", [this]() -> const sva::ForceVecd & { return RHwrench_; });
  logger.addLogEntry("HelpUp_ComputedAssistanceForces",
                     [this]() -> const sva::ForceVecd & { return RedistribWrench_; });

  logger.addLogEntry("HelpUp_human_nbContacts",
                     [this]()
                     {
                       return contactSetHum_->numberOfContacts()
                              / 4; // contacts are added as 4 points with their own friction cone, see
                                   // https://hal.archives-ouvertes.fr/hal-02108449/document
                     });

  logger.addLogEntry("HelpUp_human_RCheekDist", [this]() { return RCheekChair->pair.getDistance(); });
  logger.addLogEntry("HelpUp_human_LCheekDist", [this]() { return LCheekChair->pair.getDistance(); });
}

void HelpUpController::addGuiElements()
{
  robotPolytope_.addToGUI(*gui());
  humanPolytope_.addToGUI(*gui());

  humanDCMTracker_->addGuiElements(*gui());

  constexpr double DCM_POINT_SIZE = 0.015;
  constexpr double COM_POINT_SIZE = 0.02;
  constexpr double ARROW_HEAD_DIAM = 0.015;
  constexpr double ARROW_HEAD_LEN = 0.01;
  constexpr double ARROW_SHAFT_DIAM = 0.01;
  constexpr double FORCESHOES_SCALE = 0.002;
  constexpr double FORCE_HANDS_SCALE = 0.005;

  const std::map<char, mc_rtc::gui::Color> COLORS = {
      {'r', mc_rtc::gui::Color{1.0, 0.0, 0.0}}, {'g', mc_rtc::gui::Color{0.0, 1.0, 0.0}},
      {'b', mc_rtc::gui::Color{0.0, 0.0, 1.0}}, {'y', mc_rtc::gui::Color{1.0, 0.5, 0.0}},
      {'c', mc_rtc::gui::Color{0.0, 0.5, 1.0}}, {'m', mc_rtc::gui::Color{1.0, 0.0, 0.5}}};

  mc_rtc::gui::ArrowConfig forceArrowConfig;
  forceArrowConfig.shaft_diam = 1 * ARROW_SHAFT_DIAM;
  forceArrowConfig.head_diam = 1 * ARROW_HEAD_DIAM;
  forceArrowConfig.head_len = 1 * ARROW_HEAD_LEN;
  forceArrowConfig.scale = 1.;
  forceArrowConfig.start_point_scale = 0.01;
  forceArrowConfig.end_point_scale = 0.;

  mc_rtc::gui::ArrowConfig VRPforceArrowConfig = forceArrowConfig;
  VRPforceArrowConfig.color = COLORS.at('r');

  mc_rtc::gui::ArrowConfig DCMforceArrowConfig = forceArrowConfig;
  DCMforceArrowConfig.color = COLORS.at('c');

  mc_rtc::gui::ArrowConfig ShoesforceArrowConfig = forceArrowConfig;
  ShoesforceArrowConfig.color = COLORS.at('y');

  mc_rtc::gui::ArrowConfig TargetforceArrowConfig = forceArrowConfig;
  TargetforceArrowConfig.color = COLORS.at('b');
  TargetforceArrowConfig.start_point_scale = 0;
  TargetforceArrowConfig.color.a = 0.3;

  mc_rtc::gui::ArrowConfig MissingforceArrowConfig = forceArrowConfig;
  MissingforceArrowConfig.color = COLORS.at('g');

  gui()->addElement({}, mc_rtc::gui::Input("Chebichev Coeff [0-1]", chebichevCoef_));
  if(robots().hasRobot("panda"))
  {
    gui()->addElement({},
                      mc_rtc::gui::Button("Panda high stiffness mode",
                                          [this]()
                                          {
                                            // pandaTransform_->stiffness(
                                            //     sva::MotionVecd(Eigen::Vector3d(100, 100, 100), Eigen::Vector3d(50,
                                            //     50, 50)));
                                            // XXX lowering stiffness here: should not be necessary if position AND
                                            // velocity/acc commands should also solve the threshold problem of the
                                            // force constrained task
                                            pandaForceConstrainedTransform_->stiffness(
                                                sva::MotionVecd(Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(5, 5, 5)));
                                            // pandaAdmi_->stiffness(
                                            //        sva::MotionVecd(Eigen::Vector3d(100, 100, 100),
                                            //        Eigen::Vector3d(50, 50, 50)));
                                          }),
                      mc_rtc::gui::Button("Panda low stiffness mode",
                                          [this]() { // pandaTransform_->stiffness(1);
                                            pandaForceConstrainedTransform_->stiffness(1);
                                            // pandaAdmi_->stiffness(1);
                                          }));
  }
  gui()->addElement({},
                    mc_rtc::gui::Checkbox(
                        "Scale CoM height", [this]() { return datastore().get<bool>("HelpUp::scaleRobotCoMZ"); },
                        [this]() {
                          datastore().get<bool>("HelpUp::scaleRobotCoMZ") =
                              !datastore().get<bool>("HelpUp::scaleRobotCoMZ");
                        }),
                    mc_rtc::gui::Checkbox(
                        "Scale CoM lateral", [this]() { return datastore().get<bool>("HelpUp::scaleRobotCoMLateral"); },
                        [this]() {
                          datastore().get<bool>("HelpUp::scaleRobotCoMLateral") =
                              !datastore().get<bool>("HelpUp::scaleRobotCoMLateral");
                        }),
                    mc_rtc::gui::Button("Add computed force mode",
                                        [this]()
                                        {
                                          computedForceMode_ = true;
                                          datastore().get<bool>("HelpUp::ChangeMode") = true;
                                        }),
                    mc_rtc::gui::Button("Follow only mode",
                                        [this]()
                                        {
                                          computedForceMode_ = false;
                                          datastore().get<bool>("HelpUp::ChangeMode") = true;
                                        }));

  gui()->addElement(
      {"Plugin", "ForceShoes", "Values"},
      mc_rtc::gui::Arrow(
          "LFShoe", ShoesforceArrowConfig,
          [this]() -> Eigen::Vector3d { return robot("human").surfacePose("LFsensor").translation(); },
          [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
          {
            sva::PTransformd X_LFsensor_0 = robot("human").surfacePose("LFsensor").inv();
            Eigen::Vector3d F_LFsensor_world = X_LFsensor_0.forceDualMul(LFShoe_);
            return robot("human").surfacePose("LFsensor").translation() + FORCESHOES_SCALE * F_LFsensor_world;
          }),
      mc_rtc::gui::Arrow(
          "LBShoe", ShoesforceArrowConfig,
          [this]() -> Eigen::Vector3d { return robot("human").surfacePose("LBsensor").translation(); },
          [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
          {
            sva::PTransformd X_LBsensor_0 = robot("human").surfacePose("LBsensor").inv();
            Eigen::Vector3d F_LBsensor_world = X_LBsensor_0.forceDualMul(LBShoe_);
            return robot("human").surfacePose("LBsensor").translation() + FORCESHOES_SCALE * F_LBsensor_world;
          }),
      mc_rtc::gui::Arrow(
          "RFShoe", ShoesforceArrowConfig,
          [this]() -> Eigen::Vector3d { return robot("human").surfacePose("RFsensor").translation(); },
          [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
          {
            sva::PTransformd X_RFsensor_0 = robot("human").surfacePose("RFsensor").inv();
            Eigen::Vector3d F_RFsensor_world = X_RFsensor_0.forceDualMul(RFShoe_);
            return robot("human").surfacePose("RFsensor").translation() + FORCESHOES_SCALE * F_RFsensor_world;
          }),
      mc_rtc::gui::Arrow(
          "RBShoe", ShoesforceArrowConfig,
          [this]() -> Eigen::Vector3d { return robot("human").surfacePose("RBsensor").translation(); },
          [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
          {
            sva::PTransformd X_RBsensor_0 = robot("human").surfacePose("RBsensor").inv();
            Eigen::Vector3d F_RBsensor_world = X_RBsensor_0.forceDualMul(RBShoe_);
            return robot("human").surfacePose("RBsensor").translation() + FORCESHOES_SCALE * F_RBsensor_world;
          }),
      mc_rtc::gui::Arrow(
          "CoMForce", ShoesforceArrowConfig, [this]() -> Eigen::Vector3d { return xsensCoMpos_; },
          [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
          { return xsensCoMpos_ + FORCESHOES_SCALE * humanDCMTracker_->getAppliedForcesSum().force(); }));

  if(datastore().has("ReplayPlugin::LHWrench") && datastore().has("ReplayPlugin::TargetLH"))
  {
    gui()->addElement(
        {"Plugin", "Replay", "RobotPushHuman"},
        mc_rtc::gui::Arrow(
            "LHand", ShoesforceArrowConfig,
            [this]() -> Eigen::Vector3d { return robot("human").surfacePose("Back").translation(); },
            [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
            {
              sva::PTransformd X_0_LHsensorReplayPose =
                  robot("hrp4").forceSensor("LeftHandForceSensor").X_0_s(robot("hrp4"))
                  * robot("hrp4").surfacePose("LeftHand").inv() * robot("human").surfacePose("Back");
              Eigen::Vector3d F_LH_world = X_0_LHsensorReplayPose.inv().forceDualMul(LHForceLog_);
              return robot("human").surfacePose("Back").translation() + FORCESHOES_SCALE * F_LH_world;
            }),
        mc_rtc::gui::Arrow(
            "RHand", ShoesforceArrowConfig,
            [this]() -> Eigen::Vector3d { return robot("human").surfacePose("RightShoulder").translation(); },
            [this, FORCE_HANDS_SCALE]() -> Eigen::Vector3d
            {
              sva::PTransformd X_0_RHsensorReplayPose =
                  robot("hrp4").forceSensor("RightHandForceSensor").X_0_s(robot("hrp4"))
                  * robot("hrp4").surfacePose("RightHand").inv() * robot("human").surfacePose("RightShoulder");
              Eigen::Vector3d F_RH_world = X_0_RHsensorReplayPose.inv().forceDualMul(RHForceLog_);
              return robot("human").surfacePose("RightShoulder").translation() + FORCE_HANDS_SCALE * F_RH_world;
            }),
        mc_rtc::gui::Arrow(
            "LHandObjective", TargetforceArrowConfig,
            [this]() -> Eigen::Vector3d
            { return robot(wrenchDistributionTarget_("targetRobot")).surfacePose("Back").translation(); },
            [this, FORCE_HANDS_SCALE]() -> Eigen::Vector3d
            {
              sva::PTransformd X_LHcontact_0 = robot("human").surfacePose("Back").inv();
              Eigen::Vector3d F_LH_world = X_LHcontact_0.forceDualMul(LHtargetForceLog_);
              return robot(wrenchDistributionTarget_("targetRobot")).surfacePose("Back").translation()
                     + FORCE_HANDS_SCALE * F_LH_world;
            }),
        mc_rtc::gui::Arrow(
            "RHandObjective", TargetforceArrowConfig,
            [this]() -> Eigen::Vector3d
            { return robot(wrenchDistributionTarget_("targetRobot")).surfacePose("RightShoulder").translation(); },
            [this, FORCE_HANDS_SCALE]() -> Eigen::Vector3d
            {
              sva::PTransformd X_RHcontact_0 = robot("human").surfacePose("RightShoulder").inv();
              Eigen::Vector3d F_RH_world = X_RHcontact_0.forceDualMul(RHtargetForceLog_);
              return robot(wrenchDistributionTarget_("targetRobot")).surfacePose("RightShoulder").translation()
                     + FORCE_HANDS_SCALE * F_RH_world;
            }));
  }

  if(robots().hasRobot("panda"))
  {
    gui()->addElement({"Panda"}, mc_rtc::gui::Arrow(
                                     "Force Sensor", ShoesforceArrowConfig,
                                     [this]() -> Eigen::Vector3d
                                     { return robot("panda").frame("panda_link8").position().translation(); },
                                     [this, FORCESHOES_SCALE]() -> Eigen::Vector3d
                                     {
                                       return robot("panda").frame("panda_link8").position().translation()
                                              + 3 * FORCESHOES_SCALE
                                                    * robot("panda").forceSensor("LeftHandForceSensor").force();
                                     }));
  }

  gui()->addElement({"Target"}, mc_rtc::gui::Transform(
                                    "DCMobjective", [this]() -> const sva::PTransformd & { return robDCMTarget_; },
                                    [this](const sva::PTransformd & p) { robDCMTarget_ = p; })

  );

  // gui()->addElement({"AccPoly"},
  //     mc_rtc::gui::Polygon("HRP4accBalanceRegion", mc_rtc::gui::Color{0.8, 0., 0.}, [this]() { return accelerations_;
  //     }), mc_rtc::gui::Polygon("HumanaccBalanceRegion", mc_rtc::gui::Color{0., 0.8, 0.}, [this]() { return
  //     humaccelerations_; })
  // );

  // gui()->addPlot(
  //   "Applied force",
  //   mc_rtc::gui::plot::X("t", [this]() { return t_; }),
  //   // mc_rtc::gui::plot::Y("RH Force", [this]() { return
  //   realRobot().forceSensor("RightHandForceSensor").force().z(); }, mc_rtc::gui::Color::Red),
  //   // mc_rtc::gui::plot::Y("LH Force", [this]() { return realRobot().forceSensor("LeftHandForceSensor").force().z();
  //   }, mc_rtc::gui::Color::Green) mc_rtc::gui::plot::Y("RH Force", [this]() { return
  //   realRobot().forceSensor("RArm_6AF").force().z(); }, mc_rtc::gui::Color::Red), mc_rtc::gui::plot::Y("LH Force",
  //   [this]() { return realRobot().forceSensor("LArm_6AF").force().z(); }, mc_rtc::gui::Color::Green)
  // );
}

void HelpUpController::planes(const std::vector<mc_rbdyn::Plane> & constrPlanes, whatRobot rob)
{
  switch(rob)
  {
    case mainRob:
      planes_.clear();
      planes_ = constrPlanes;
      comIncPlaneConstraintPtr_->setPlanes(solver(), constrPlanes, {}, {}, 0.1, 0.03, 0.6, 0.0);
      break;
    case human:
      planesHum_.clear();
      planesHum_ = constrPlanes;
      comIncPlaneConstraintHumPtr_->setPlanes(solver(), constrPlanes, {}, {}, 0.1, 0.03, 0.6, 0.0);
      break;
    case combined:

      break;
  }
}

void HelpUpController::planes(const std::vector<Eigen::Vector4d> & constrPlanes, whatRobot rob)
{
  std::vector<mc_rbdyn::Plane> p;

  for(auto plane : constrPlanes)
  {
    p.push_back({{-plane(0), -plane(1), -plane(2)}, plane(3)});
  }
  planes(p, rob);
}

void HelpUpController::updateContactSet(
    unsigned int robotIndex) // todo update human outside of qp if too heavy to be under 5ms
{
  auto contacts = solver().contacts();
  updateContactSet(contacts, robotIndex);
}

void HelpUpController::updateContactSet(const std::vector<mc_rbdyn::Contact> & contacts, unsigned int robotIndex)
{
  // auto maxForces = config_("surfacesMaxForces");
  Eigen::Vector3d acceleration;
  const auto & robot = robots().robot(robotIndex);
  // this line finds either the string in the global config or in the robot specific configuration file
  auto maxForces = config_.find("surfacesMaxForces").value_or(mc_rtc::Configuration{}).find(robot.name()).value();

  // XXX can we avoid allocating a new contact set every time?
  contactSet_ = std::make_shared<ContactSet>(false);
  contactSet_->mass(robots().robot(robotIndex).mass());
  contactSet_->setFrictionSides(6);

  for(const auto & contact : contacts)
  {
    if(contact.r1Index() == robotIndex || contact.r2Index() == robotIndex)
    {
      const auto & surface_name =
          (contact.r1Index() == robotIndex) ? contact.r1Surface()->name() : contact.r2Surface()->name();
      const auto & surface = robot.surface(surface_name);

      const auto & bodyName = surface.bodyName();
      const auto & body_PT = robot.bodyPosW(bodyName);

      const auto & points = surface.points();
      int ptCpt = 0; // point counter
      double mu = contact.friction(); // get the friction coef h

      double fmax;
      if(maxForces.has(surface_name))
      {
        fmax = maxForces(surface_name);
      }
      else
      {
        mc_rtc::log::warning("Surface " + surface_name + " NOT found in the config for max force");
        fmax = robot.mass() * 10;
      }

      double fmin = 0; // todo set the same for min forces?
      ContactType type;
      if(config_.has("constrainedSurfaces"))
      {
        auto constrainedSurfaces = config_("constrainedSurfaces");
        if(constrainedSurfaces.has(surface_name))
        {
          type = ContactType::constrained;
        }
        else
        {
          type = ContactType::support;
        }
      }
      else
      {
        type = ContactType::support;
      }

      for(const auto & point : points)
      {
        auto pos = body_PT.rotation().transpose() * point.translation() + body_PT.translation();

        Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
        homTrans.block<3, 3>(0, 0) = body_PT.rotation().transpose() * point.rotation().transpose();
        homTrans.block<3, 1>(0, 3) = pos;

        const auto ptName = surface.name() + "_" + std::to_string(ptCpt);

        // Not adding hand contacts to robot balance polytope (stabilizer issues)
        if(handContactsForBalance_)
        {
          contactSet_->addContact(ptName, homTrans, mu, fmax, fmin, type);
        }
        else
        {
          if(!(surface_name == "LeftHand" || surface_name == "RightHand"))
          {
            contactSet_->addContact(ptName, homTrans, mu, fmax, fmin, type);
          }
        }

        ptCpt++;
      }
    }
  }

  // // Adding hand contacts to polytope contact set even if it is not a contact control wise ()
  // RHandShoulder->update(realRobot("hrp4"), robot("human"));
  // LHandBack->update(robot("hrp4"), robot("human"));

  // Adding the accelerations

  acceleration << 0.0, 0.0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0.6, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, 0.6, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << -0.6, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, -0.6, -9.81;
  contactSet_->addCoMAcc(acceleration);
}

void HelpUpController::contactForces(const std::map<std::string, double> & contactFMax,
                                     const std::map<std::string, double> & contactFMin)
{
  contactFMax_ = contactFMax;
  contactFMin_ = contactFMin;
}

void HelpUpController::updateContactForces()
{
  for(const auto & [contactName, contactForceMax] : contactFMax_)
  {
    contactSet_->setContactFMax(contactForceMax, contactName);
  }
  for(const auto & [contactName, contactForceMin] : contactFMin_)
  {
    contactSet_->setContactFMin(contactForceMin, contactName);
  }
}

void HelpUpController::desiredCoM(Eigen::Vector3d desiredCoM, whatRobot rob)
{
  // double coef = 0.01;
  double coef = 0.1;
  Eigen::Vector3d prevCoM;
  switch(rob)
  {
    case mainRob:
      // setting com height manually, scaled as human stands up
      // hrp4 half sitting com: 0.78, human standing com: 0.87, e2dr half sitting com: 0.97
      // desiredCoM[2] = std::max((0.78*robot("human").com().z())/0.87 , 0.7);
      // desiredCoM[2] = std::max((0.95*robot("human").com().z())/0.87 , 0.9);
      // desiredCoM[2] = 0.95;
      desiredCoM[2] = 0.78;
      prevCoM = comDesired_;
      comDesired_ = (1 - coef) * prevCoM + coef * desiredCoM;

      // Instead of the general com task we use the stabilizer task to manage the robot com
      if(datastore().has("RobotStabilizer::getTask"))
      {
        auto stabTask =
            datastore().call<std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask>>("RobotStabilizer::getTask");
        stabTask->staticTarget(comDesired_);
      }

      break;
    case human:
      // this is now deprecated and shouldn't be used since human com is managed by custom state
      desiredCoM[2] = 0.87;
      prevCoM = comDesiredHum_;
      comDesiredHum_ = (1 - coef) * prevCoM + coef * desiredCoM;
      // comTaskHum_->com(comDesiredHum_);
      break;
    case combined:

      break;
  }
}

std::map<std::string, double> HelpUpController::getConfigFMax() const
{
  auto maxForces = config_("surfacesMaxForces");
  std::map<std::string, double> configFMax;

  for(auto contact : contactSet_->get_contactNames())
  {
    // for each contact find the surface name (hopefully first part of the name
    auto underscore = contact.find("_");
    std::string surfaceName = contact.substr(0, underscore);

    double fmax = robot().mass() * 10;
    if(maxForces.has(surfaceName))
      fmax = static_cast<double>(maxForces(surfaceName));
    else
      mc_rtc::log::warning("[HelpUpController] Surface {} is not in surfacesMaxForces", surfaceName);
    // then add the maxForce
    configFMax.emplace(contact, fmax);
  }

  return configFMax;
}

std::map<std::string, double> HelpUpController::getConfigFMin() const
{
  auto minForces = config_("surfacesMinForces");
  std::map<std::string, double> configFMin;

  for(auto contact : contactSet_->get_contactNames())
  {
    // for each contact find the surface name (hopefully first part of the name
    auto underscore = contact.find("_");
    std::string surfaceName = contact.substr(0, underscore);

    double fmin = robot().mass() * 10;
    if(minForces.has(surfaceName))
      fmin = static_cast<double>(minForces(surfaceName));
    else
      mc_rtc::log::warning("[HelpUpController] Surface {} is not in surfacesMinForces", surfaceName);
    // then add the maxForce
    configFMin.emplace(contact, fmin);
  }

  return configFMin;
}

void HelpUpController::updateRealHumContacts()
{
  // mc_rtc::log::info("Updating human contacts");
  RFootGround->update(robot("human"), robot("ground"));
  LFootGround->update(robot("human"), robot("ground"));
  RCheekChair->update(robot("human"), robot("chair"));
  LCheekChair->update(robot("human"), robot("chair"));
  RBackLegChair->update(robot("human"), robot("chair"));
  LBackLegChair->update(robot("human"), robot("chair"));
  // RHandShoulder->update(robot(), robot("human"));
  // LHandBack->update(robot(), robot("human"));

  double distThreshold = 0.01;
  double speedThreshold = 1e-4;

  auto prevContactNb = contactSetHum_->numberOfContacts() / 4;

  contactSetHum_ = std::make_shared<ContactSet>(false);
  contactSetHum_->mass(humanMass_);
  contactSetHum_->setFrictionSides(6);

  // adding feet contacts in any case
  addRealHumContact("RightSole", 0, humanMass_ * 9.81, ContactType::support);
  addRealHumContact("LeftSole", 0, humanMass_ * 9.81, ContactType::support);

  if((RCheekChair->pair.getDistance()
      <= distThreshold)) // || (humanDCMTracker_->getAppliedForcesSum().force().z() < 0.9 * 9.81 * humanMass_))
  {
    addRealHumContact("RCheek", 0, humanMass_ * 9.81, ContactType::support);
  }

  if((LCheekChair->pair.getDistance()
      <= distThreshold)) // || (humanDCMTracker_->getAppliedForcesSum().force().z() < 0.9 * 9.81 * humanMass_))
  {
    addRealHumContact("LCheek", 0, humanMass_ * 9.81, ContactType::support);
  }

  // if(RBackLegChair->pair.getDistance() <= distThreshold)
  // {
  //   addRealHumContact("RLegBack", 0, humanMass_ * 9.81, ContactType::support);
  // }

  // if(LBackLegChair->pair.getDistance() <= distThreshold)
  // {
  //   addRealHumContact("LLegBack", 0, humanMass_ * 9.81, ContactType::support);
  // }

  // if (LHandBack->pair.getDistance()<=distThreshold)
  // {
  //   addRealHumContact("Back", 0, 200, ContactType::support);
  // }

  // if (RHandShoulder->pair.getDistance()<=distThreshold)
  // {
  //   addRealHumContact("RightShoulder", 0, 200, ContactType::support);
  // }

  // Adding the accelerations
  Eigen::Vector3d acceleration;

  acceleration << 0.0, 0.0, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << 0.6, 0, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << 0, 0.6, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << -0.6, 0, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << 0, -0.6, -9.81;
  contactSetHum_->addCoMAcc(acceleration);
}

void HelpUpController::addRealHumContact(std::string humanSurfName, double fmin, double fmax, ContactType type)
{
  const auto & surf = robot("human").surface(humanSurfName);
  const auto & points = surf.points();

  int ptCpt = 0; // point counter
  std::string ptName;
  double mu = 0.7; // friction coef h

  auto surf_PT = surf.X_0_s(robot("human"));

  for(const auto & point : points)
  {
    auto pos = surf_PT.rotation().transpose() * point.translation() + surf_PT.translation();

    Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
    homTrans.block<3, 3>(0, 0) = surf_PT.rotation().transpose() * point.rotation().transpose();
    homTrans.block<3, 1>(0, 3) = pos;

    ptName = humanSurfName + "_" + std::to_string(ptCpt);
    contactSetHum_->addContact(ptName, homTrans, mu, fmax, fmin, type);

    ptCpt++;
  }
}

sva::ForceVecd HelpUpController::getCurrentForceVec(const std::vector<sva::ForceVecd> & log,
                                                    double startOffset,
                                                    double freq)
{
  double acqTimeStep = 1 / freq;
  if(t_ < startOffset)
  {
    return log[0];
  }
  else if(t_ >= startOffset && int(t_ / acqTimeStep) - int(startOffset / acqTimeStep) < log.size())
  {
    // mc_rtc::log::info("acq timestep is {}, so t_/acq = {} and t/acq - start/step = {}", acqTimeStep,
    // int(t_/acqTimeStep), int(t_/acqTimeStep) - int(startOffset/acqTimeStep));
    return log[int(t_ / acqTimeStep) - int(startOffset / acqTimeStep)];
  }
  else
  {
    // mc_rtc::log::info(" else: acq timestep is {}, so t_/acq = {} and t/acq - start/step = {}", acqTimeStep,
    // int(t_/acqTimeStep), int(t_/acqTimeStep) - int(startOffset/acqTimeStep));
    return log.back();
  }
}

void HelpUpController::distributeAssistiveHandsWrench(const sva::ForceVecd & desiredWrench,
                                                      const mc_rbdyn::Robot & helperRobot,
                                                      const std::string & targetRobot,
                                                      const std::string & leftSurface,
                                                      const std::string & rightSurface)
{
  // Variables
  // ---------
  // x = [w_lh_lhc w_rh_rhc] where
  // w_lh_lhc: spatial force vector of robot left hand contact in left hand contact frame: world frame and centroidal
  // frame share the same orientation (should it be inertial frame, ie at contact instead of world?) w_rh_rhc: spatial
  // force vector of robot right hand contact in right hand contact frame

  // Objective
  // ---------
  // Weighted minimization of the following tasks:
  // w_lh_C + w_rh_C == desiredWrench  -- realize desired wrench at human CoM
  // (X_0_lhc* w_lh_0) == 0 -- minimize left hand force applied on human
  // (X_0_rhc* w_rh_0) == 0 -- minimize right hand force applied on human

  // maybe add minimization of arm torques?

  // w_l_lankle == 0 -- minimize left foot ankle torque (anisotropic weight) (=smallest value possible?)
  // w_r_rankle == 0 -- minimize right foot ankle torque (anisotropic weight)
  // Constraints
  // -----------
  // CWC X_0_lhc* w_lh_0 <= 0  -- left hand wrench within contact wrench cone
  // CWC X_0_rhc* w_rh_0 <= 0  -- right hand wrench within contact wrench cone
  // (X_0_lhc* w_lh_0).z() > minForce  -- minimum left hand contact force to maintain contact
  // (X_0_rhc* w_rh_0).z() > minForce  -- minimum right hand contact force
  // (X_0_lhc* w_lh_0).z() + (X_0_rhc* w_rh_0).z() < maxForce -- maximum force on the human

  // const auto & leftHandContact = contacts_.at(ContactState::Left);
  // Create hand contacts
  const mc_tasks::lipm_stabilizer::internal::Contact leftHandContact(helperRobot, leftSurface, 0.7);
  const mc_tasks::lipm_stabilizer::internal::Contact rightHandContact(helperRobot, rightSurface, 0.7);
  const sva::PTransformd & X_0_lhc = leftHandContact.surfacePose();
  const sva::PTransformd & X_0_rhc = rightHandContact.surfacePose();
  // sva::PTransformd X_0_vrp(desiredVRP()); // getting vrp transform
  sva::PTransformd X_0_C; // getting CoM transform
  if(targetRobot == "human")
  {
    X_0_C = sva::PTransformd(xsensCoMpos_); // getting CoM transform
  }
  else if(targetRobot == "panda")
  {
    X_0_C = sva::PTransformd(robot("panda").frame("BoxCenter").position().translation());
  }

  // vérifier transformations au com
  sva::PTransformd X_lhc_c = X_0_C * X_0_lhc.inv();
  sva::PTransformd X_rhc_c = X_0_C * X_0_rhc.inv();

  constexpr unsigned NB_VAR = 6 + 6; // 6d wrench * 2 contacts
  constexpr unsigned COST_DIM = 6 + NB_VAR; // 6 for desired wrench, nb_var to minimize our variables individually
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A.setZero(COST_DIM, NB_VAR);
  b.setZero(COST_DIM);

  // |w_lh_c + w_rh_c - desiredWrench|^2
  // We handle moments around the ZMP instead of the world origin to avoid numerical errors due to large moment values.
  // https://github.com/jrl-umi3218/mc_rtc/pull/285
  // Why zmp and not CoM ? In our case CoM seems more appropriate: desired wrench is the result of the various contacts
  // on the CoM Thus minimize difference between result of hand contact wrenches on the CoM and missing forces to apply
  // (desired wrench)

  auto A_net = A.block<6, 12>(0, 0);
  auto b_net = b.segment<6>(0);
  A_net.block<6, 6>(0, 0) =
      X_lhc_c
          .dualMatrix(); // matrix transform for wrench from hand contact to com (since variable is hand contact wrench)
  A_net.block<6, 6>(0, 6) = X_rhc_c.dualMatrix();
  // b_net = X_0_vrp.dualMul(desiredWrench).vector(); ? since desired wrench is on the com maybe just this?

  // IMPORTANT we send the NEGATIVE desired wrench because the wrench we compute is the wrench for the human
  // this means the robot must apply negative this value (reaction force) on its assisting end-effectors
  b_net = -desiredWrench.vector();

  // |wrench_lh|^2
  auto A_lhwrench = A.block<6, 6>(6, 0);
  // |wrench_rh|^2
  auto A_rhwrench = A.block<6, 6>(12, 6);
  // anisotropic weights:  taux, tauy, tauz,   fx,   fy,   fz;
  A_lhwrench.diagonal() << 1., 1., 1., 1., 1., 1.; // 1.,   1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_rhwrench.diagonal() << 1., 1., 1., 1., 1., 1.;
  // A_lhwrench *= X_0_lhc.dualMatrix(); I don't think transforms are needed since variables are in contact frame and
  // those are the ones we want to minimize A_rhwrench *= X_0_rhc.dualMatrix(); corresponding b vector is zero since we
  // want to minimize (closest to zero)

  // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
  // this is to be sure to have an equal repartition

  // double lfr = leftFootRatio_;
  // auto A_force = A.block<1, 12>(18, 0);
  // A_force.block<1, 6>(0, 0) = (1 - lfr) * X_0_lc.dualMatrix().bottomRows<1>();
  // A_force.block<1, 6>(0, 6) = -lfr * X_0_rc.dualMatrix().bottomRows<1>();

  // Apply weights
  double valueWeight = std::sqrt(10000);
  double wrenchWeight = std::sqrt(100);
  A_net *= valueWeight;
  b_net *= valueWeight;
  A_lhwrench *= wrenchWeight;
  A_rhwrench *= wrenchWeight;

  // transformation to a least squares problem
  Eigen::MatrixXd Q = A.transpose() * A;
  Eigen::VectorXd c = -A.transpose() * b;

  // The CoP constraint represent 4 linear constraints for each contact
  const int cwc_const = 12 + 4;
  // Two contacts with cwc constraints + two min force constraints + one max force constraint
  const int nb_const = 2 * cwc_const + 2 + 1;
  Eigen::Matrix<double, -1, NB_VAR> A_ineq;
  Eigen::VectorXd b_ineq;
  A_ineq.setZero(nb_const, NB_VAR);
  b_ineq.setZero(nb_const);

  auto A_lh_cwc = A_ineq.block<cwc_const, 6>(0, 0);
  auto A_rh_cwc = A_ineq.block<cwc_const, 6>(cwc_const, 6);

  // CWC * w_lh_lc <= 0
  A_lh_cwc = leftHandContact.wrenchFaceMatrix().block(0, 0, cwc_const, 6); // * X_0_lhc.dualMatrix();
  // b_ineq.segment(0,cwc_const) is already zero

  // CWC * w_rh_rc <= 0
  A_rh_cwc = rightHandContact.wrenchFaceMatrix().block(0, 0, cwc_const, 6); // * X_0_rhc.dualMatrix();
  // b_ineq.segment(cwc_const,cwc_const) is already zero

  // w_l_lc.force().z() >= min_Force
  double minHandsForce = 5.; // minimal contact force in Newtons
  A_ineq.block(nb_const - 3, 0, 1, 6) =
      -Eigen::Matrix6d::Identity().bottomRows<1>(); // selecting force only, identity bc variable is already in contact
                                                    // frame //-X_0_lhc.dualMatrix().bottomRows<1>();
  b_ineq(nb_const - 3) = -minHandsForce;
  // w_r_rc.force().z() >= min_Force
  A_ineq.block(nb_const - 2, 6, 1, 6) =
      -Eigen::Matrix6d::Identity().bottomRows<1>(); //-X_0_rhc.dualMatrix().bottomRows<1>();
  b_ineq(nb_const - 2) = -minHandsForce;
  // (X_0_lhc* w_lh_0).z() + (X_0_rhc* w_rh_0).z() < maxForce
  double maxCompression = 200.; // max force applied total on torso
  A_ineq.block(nb_const - 1, 0, 1, 6) = Eigen::Matrix6d::Identity().bottomRows<1>(); // first 6 elements are lh
  A_ineq.block(nb_const - 1, 6, 1, 6) = Eigen::Matrix6d::Identity().bottomRows<1>(); // second 6 are rh
  b_ineq(nb_const - 1) = maxCompression;

  vrpSolver_.problem(NB_VAR, 0, nb_const);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);
  bool solutionFound = vrpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ false);
  if(!solutionFound)
  {
    mc_rtc::log::error("Hands force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = vrpSolver_.result();
  sva::ForceVecd w_lh_lhc(x.segment<3>(0), x.segment<3>(3)); // wrench value at contact frame
  sva::ForceVecd w_rh_rhc(x.segment<3>(6), x.segment<3>(9));

  LHwrench_ = w_lh_lhc;
  RHwrench_ = w_rh_rhc;

  RedistribWrench_ = X_lhc_c.dualMul(w_lh_lhc) + X_rhc_c.dualMul(w_rh_rhc); // Verify coherent solution at CoM

  // sva::ForceVecd w_lh_lhc = X_0_lhc.dualMul(w_lh_0); // wrench value at contact pose
  // sva::ForceVecd w_rh_rhc = X_0_rhc.dualMul(w_rh_0);
  // Eigen::Vector2d leftCoP = (constants::vertical.cross(w_lh_lhc.couple()) / w_lh_lhc.force()(2)).head<2>();
  // Eigen::Vector2d rightCoP = (constants::vertical.cross(w_rh_rhc.couple()) / w_rh_rhc.force()(2)).head<2>();
  // footTasks[ContactState::Left]->targetCoP(leftCoP);
  // footTasks[ContactState::Left]->targetForce(w_lh_lhc.force());
  // footTasks[ContactState::Right]->targetCoP(rightCoP);
  // footTasks[ContactState::Right]->targetForce(w_rh_rhc.force());
}
