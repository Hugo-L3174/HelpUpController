#include "HelpUpController.h"
#include "config.h"

HelpUpController::HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), polytopeIndex_(0), polytopeHumIndex_(0), computed_(false), computedHum_(false), computing_(false), 
computingHum_(false), transitionning_(false), transitionningHum_(false), readyForComp_(false), readyForCompHum_(false)
, DCMerrorBuffer_(19), humanOmegaBuffer_(19)
{
  // Load entire controller configuration file
  config_.load(config);
  // trajectories_ = std::make_shared<TrajectoryModel> (/*path*/);
  
  /* Observers
  */
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) { // robot() is the main robot (hrp4)
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
  });
  
  // datastore().make_call("KinematicAnchorFrame::human" , [this](const mc_rbdyn::Robot & robot) { // robot 3 (out of 4) is the human robots().robots()[3].name()
  //   return sva::interpolate(robot.surfacePose("LeftSole"), robot.surfacePose("RightSole"), 0.5);
  // });
  

  // comIncPlaneConstraintPtr_ = std::make_shared<mc_solver::CoMIncPlaneConstr> (robots(), robots().robotIndex(), dt);*
  comIncPlaneConstraintPtr_ = std::make_shared<mc_solver::CoMIncPlaneConstr> (realRobots(), realRobots().robotIndex(), dt);
  comIncPlaneConstraintHumPtr_ = std::make_shared<mc_solver::CoMIncPlaneConstr> (robots(), robots().robotIndex("human"), dt);


  // initialize the current computation point:
  currentCompPoint_ = std::make_shared<ComputationPoint>  (-1, std::make_shared<ContactSet>(false));
  currentHumCompPoint_ = std::make_shared<ComputationPoint>  (-1, std::make_shared<ContactSet>(false));

  // init display regions
  balanceCompPoint_ = currentCompPoint_;
  balanceHumCompPoint_ = currentHumCompPoint_;
  
  comTask_ = std::make_shared<mc_tasks::CoMTask> (robots(), robots().robotIndex(), 5.0, 2e3); // Stiffness 5, weight 2000
  comTask_->damping(10.0); 
  // comTask_->com(realRobot().com());
  comDesired_ = robot().com();
  comTask_->com(robot().com()); 

  // human com task is created here but managed in custom state
  comTaskHum_ = std::make_shared<mc_tasks::CoMTask> (robots(), robots().robotIndex("human"), 5.0, 2e3); // Stiffness 5, weight 1000
  comTaskHum_->damping(10.0); 
  comDesiredHum_ = robot("human").com();
  comTaskHum_->com(robot("human").com());

  // Stabilizer task: check configuration?
  auto stabConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabTask_ = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver().robots(),
          solver().realRobots(),
          robot().robotIndex(),
          dt
          );

  stabTask_->reset();
  // stabConf.comHeight = 0.7; // We let it depend on the robot configuration
  // stabConf.torsoWeight = 10;
  // stabConf.pelvisWeight = 100;
  // stabConf.comStiffness[2]=1000;
  // stabConf.comHeight = 0.75;
  // stabConf.pelvisWeight = 1000;
  // stabConf.comStiffness[0]=1000;
  // stabConf.comStiffness[1]=2000;
  // stabConf.comStiffness[2]=2000;
  stabTask_->configure(stabConf);
  stabTask_->staticTarget(robot().com()); // be careful that the controller will not update the objective if the com is not currently in the balance region

  // surfaces pointers for live contact sets
  const auto & human_surfaces = robot("human").surfaces();
  RFootSurf = human_surfaces.at("RightSole");
  LFootSurf = human_surfaces.at("LeftSole");
  RCheekSurf = human_surfaces.at("RCheek");
  LCheekSurf = human_surfaces.at("LCheek");
  RightShoulderSurf = human_surfaces.at("RightShoulder");
  BackSurf = human_surfaces.at("Back");

  TopSurf = realRobot("chair").surfaces().at("Top");
  GroundSurf = realRobot("ground").surfaces().at("AllGround");

  // RHandSurf = realRobot("e2dr").surfaces().at("RightHand");
  // LHandSurf = realRobot("e2dr").surfaces().at("LeftHand");
  // RHandSurf = realRobot().surfaces().at("RightHand");
  // LHandSurf = realRobot().surfaces().at("LeftHand");

  RCheekChair = std::make_shared<mc_control::SimulationContactPair> (RCheekSurf, TopSurf);
  LCheekChair = std::make_shared<mc_control::SimulationContactPair>(LCheekSurf, TopSurf);
  RFootGround = std::make_shared<mc_control::SimulationContactPair>(RFootSurf, GroundSurf);
  LFootGround = std::make_shared<mc_control::SimulationContactPair>(LFootSurf, GroundSurf);
  // RHandShoulder = std::make_shared<mc_control::SimulationContactPair>(RHandSurf, RightShoulderSurf);
  // LHandBack = std::make_shared<mc_control::SimulationContactPair>(LHandSurf, BackSurf);

  // First log: standing alone ok
  mc_rtc::Configuration dataIn1( std::string(PATH) + "/etc/forces/F1.yaml");
  mc_rtc::Configuration dataIn2( std::string(PATH) + "/etc/forces/F2.yaml");
  mc_rtc::Configuration dataIn3( std::string(PATH) + "/etc/forces/F3.yaml");
  mc_rtc::Configuration dataIn4( std::string(PATH) + "/etc/forces/F4.yaml");

  // Second log: standing with help
  // mc_rtc::Configuration dataIn1( std::string(PATH) + "/etc/forces/F5.yaml");
  // mc_rtc::Configuration dataIn2( std::string(PATH) + "/etc/forces/F6.yaml");
  // mc_rtc::Configuration dataIn3( std::string(PATH) + "/etc/forces/F7.yaml");
  // mc_rtc::Configuration dataIn4( std::string(PATH) + "/etc/forces/F8.yaml");

  auto data1 = dataIn1.operator std::map<std::string, std::vector<double>>();
  auto data2 = dataIn2.operator std::map<std::string, std::vector<double>>();
  auto data3 = dataIn3.operator std::map<std::string, std::vector<double>>();
  auto data4 = dataIn4.operator std::map<std::string, std::vector<double>>();


  // auto forceVect = sva::ForceVecd(Eigen::Vector3d(data1["Tx"][0], data1["Ty"][0], data1["Tz"][0]), Eigen::Vector3d(data1["Fx"][0], data1["Fy"][0], data1["Fz"][0]));
  // mc_rtc::log::info("first force vec is : {}", forceVect);
  for (auto i = 0; i < data1["Counter"].size(); i++)
  {
    auto forceVect = sva::ForceVecd(Eigen::Vector3d(data1["Tx"][i], data1["Ty"][i], data1["Tz"][i]), Eigen::Vector3d(data1["Fx"][i], data1["Fy"][i], data1["Fz"][i]));
    LBShoeVec_.push_back(forceVect);
  }

  for (auto i = 0; i < data2["Counter"].size(); i++)
  {
    auto forceVect = sva::ForceVecd(Eigen::Vector3d(data2["Tx"][i], data2["Ty"][i], data2["Tz"][i]), Eigen::Vector3d(data2["Fx"][i], data2["Fy"][i], data2["Fz"][i]));
    LFShoeVec_.push_back(forceVect);
  }

  for (auto i = 0; i < data3["Counter"].size(); i++)
  {
    auto forceVect = sva::ForceVecd(Eigen::Vector3d(data3["Tx"][i], data3["Ty"][i], data3["Tz"][i]), Eigen::Vector3d(data3["Fx"][i], data3["Fy"][i], data3["Fz"][i]));
    RBShoeVec_.push_back(forceVect);
  }

  for (auto i = 0; i < data4["Counter"].size(); i++)
  {
    auto forceVect = sva::ForceVecd(Eigen::Vector3d(data4["Tx"][i], data4["Ty"][i], data4["Tz"][i]), Eigen::Vector3d(data4["Fx"][i], data4["Fy"][i], data4["Fz"][i]));
    RFShoeVec_.push_back(forceVect);
  }

  if (config_.has("Omega"))
  {
    OmegaZAcc_ = config_("Omega")("WithVerticalAcc");
  }
  

  // initializing filter buffers for omega and dcm error derivatives
  for (int i = 0; i < DCMerrorBuffer_.capacity(); i++)
  {
    DCMerrorBuffer_.push_back(DCMerror_);
  }
  
  for (int i = 0; i < humanOmegaBuffer_.capacity(); i++)
  {
    humanOmegaBuffer_.push_back(humanOmega());
  }


  // mc_rtc::log::info("Human pos is {}", robots().robot("human").posW().translation().transpose());
  
  // robots().robot("chair").posW(robots().robot("human").posW() * sva::PTransformd(Eigen::Vector3d(0.05, 0.12, -0.65)));
  // mc_rtc::log::info("Chair pos is {}", robots().robot("chair").posW().translation().transpose());

  // robots().robot().posW(robots().robot("chair").posW() * sva::PTransformd(sva::RotZ(M_PI/2.0), Eigen::Vector3d(-0.17, -0.3, 0.75))); 
  // mc_rtc::log::info("Robot pos is {}", robots().robot().posW().translation().transpose());

  addLogEntries();
  addGuiElements();
  addTasksToSolver();
  

  mc_rtc::log::success("HelpUpController init done ");
}


bool HelpUpController::run()
{
  // First log: offset at 6.92 to sync with xsens log
  // LFShoe_ = getCurrentForceVec(LFShoeVec_, 6.92, 50);
  // LBShoe_ = getCurrentForceVec(LBShoeVec_, 6.92, 50);
  // RFShoe_ = getCurrentForceVec(RFShoeVec_, 6.92, 50);
  // RBShoe_ = getCurrentForceVec(RBShoeVec_, 6.92, 50);

  // Second log: offset at 6.24
  // LFShoe_ = getCurrentForceVec(LFShoeVec_, 6.24, 50);
  // LBShoe_ = getCurrentForceVec(LBShoeVec_, 6.24, 50);
  // RFShoe_ = getCurrentForceVec(RFShoeVec_, 6.24, 50);
  // RBShoe_ = getCurrentForceVec(RBShoeVec_, 6.24, 50);

  LFShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetLFForce");
  LBShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetLBForce");
  RFShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetRFForce");
  RBShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetRBForce");

  // ------------------------------------- Computation polytope robot
  if (!computing_)
  {
    // update the contact set model 
    if (!readyForComp_)
    {
      computed_ = false;
      updateContactSet(robot().robotIndex()); //robots().robotIndex("hrp4")
      // Instead of keeping the max forces as given in the config we update them as the current applied force on the contact ?
      // Not a good idea, only limiting. It is better to change the max force as what we allow the robot to exert (to stay safe for the human),
      // and the min force (necessary to help the human stand?)
      // See change in updateContactForces()
      updateContactForces();
      readyForComp_ = true;
    }

    // start the computation
    if (!computed_ and readyForComp_ and !computingHum_)
    {
      polytopeIndex_++;
      polytopeReady_ = false;

      stabThread_ = std::thread([this](int polIndex, std::shared_ptr<ContactSet> contactSet){
        this->computeStabilityRegion(contactSet, hrp4, false, polIndex);
        polytopeReady_ = true;
      }, polytopeIndex_, std::make_shared<ContactSet>(*contactSet_));
      computing_ = true;
      readyForComp_ = false;
    }
  }
  else // computing_ == true => currently computing
  {
    // check if the computation is finished
    if (polytopeReady_ and stabThread_.joinable())
    {
      stabThread_.join();
      computing_ = false;
      computed_ = true;
      // add the stuff to update the polytope

      // setting future comp point as the next one
      nextCompPoint_= futureCompPoint_;
      transitionning_ = true;
    }
  }

  
  // ------------------------------------- Computation polytope human
  if (!computingHum_)
  {
    // update the contact set model 
    if (!readyForCompHum_)
    {
      computedHum_ = false;
      updateRealHumContacts();
      // updateContactSet(robots().robotIndex("human"), human); // todo: update internal contacts from estimation
      updateContactForces();
      readyForCompHum_ = true;
    }

    // start the computation
    if (!computedHum_ and readyForCompHum_ )
    {
      polytopeHumIndex_++;
      polytopeHumReady_ = false;

      if (contactSetHum_->numberOfContacts()>1)
      {
        stabThreadHum_ = std::thread([this](int polIndex, std::shared_ptr<ContactSet> contactSetHum){
          this->computeStabilityRegion(contactSetHum, human, true, polIndex);
          polytopeHumReady_ = true;
        }, polytopeHumIndex_, std::make_shared<ContactSet>(*contactSetHum_));
        computingHum_ = true;
      }
      readyForCompHum_ = false;
    }

  }
  else // computing_ == true => currently computing
  {
    // check if the computation is finished
    if (polytopeHumReady_ and stabThreadHum_.joinable())
    {
      stabThreadHum_.join();
      computingHum_ = false;
      computedHum_ = true;
      // add the stuff to update the polytope
      nextHumCompPoint_ = futureHumCompPoint_;
      transitionningHum_ = true;
    }
  }
  // // update the desired CoM Acceleration:
  // Eigen::Vector3d comPos = robot("hrp4").com();
  // Eigen::Vector3d comVel = realRobot("hrp4").comVelocity();
  // Eigen::Vector3d comAcc = realRobot("hrp4").comAcceleration();
  
  // Eigen::Vector3d gradientPotential = Eigen::Vector3d::Zero();
  // gradientPotential = currentCompPoint_->computeGradient(comPos);


  // double dt = timeStep;
  // Eigen::Vector3d gravity;
  // gravity << 0.0, 0.0, 9.81;
  // double mass = robot().mass();

  // Eigen::Vector3d newVel = Eigen::Vector3d::Zero();


  // ------------------------------------------- transition robot
  if (transitionning_)
  {
    // if we are not in next
    Eigen::Vector3d currentCoM = robot().com(); 
    nextCompPoint_->constraintPlanes(); 
    // update display regardless of com in or out
    balanceCompPoint_ = nextCompPoint_;
    if (isVertexInPlanes(currentCoM, nextCompPoint_->constraintPlanes(), 0.03))
    {
      setNextToCurrent(hrp4);
      transitionning_ = false;
    }
  }

  // ------------------------------------------- transition human
  if (transitionningHum_)
  {
    // if we are not in next
    Eigen::Vector3d currentHumCoM = robot("human").com(); 
    nextHumCompPoint_->constraintPlanes();
    // update display regardless of com in or out
    balanceHumCompPoint_ = nextHumCompPoint_;
    if (isVertexInPlanes(currentHumCoM, nextHumCompPoint_->constraintPlanes(), 0.03))
    {
      setNextToCurrent(human);
      transitionningHum_ = false;
    }
  }
  
  prevDCMerror_ = DCMerror_;
  prevOmega_ = humanOmega();
  humanOmegaBuffer_.push_back(humanOmega());
  DCMerrorBuffer_.push_back(DCMerror_);
  computeDCMerror();
  computeCommandVRP();
  sva::ForceVecd desiredCoMWrench;
  desiredCoMWrench.force() = missingForces();
  distributeHandsWrench(desiredCoMWrench);
  t_ += solver().dt();
  bool ok = mc_control::fsm::Controller::run();
  return ok;
}

void HelpUpController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
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

  //   // Human model start posture
  //   if(config_("human").has("posture")){
  //     if(config_("human")("posture").has("target")){
  //       std::map<std::string, std::vector<double>> humanPostureTarget = config_("human")("posture")("target");
  //       // Set human mbc equal to the posture target
  //       for(auto const & t : humanPostureTarget){
  //         robots().robot("human").mbc().q[robots().robot("human").jointIndexByName(t.first)] = t.second;
  //       }
  //     }
  //   }
  mc_rtc::log::success("running reset");
  // Adjust chair position relative to human model
  // if(robots().hasRobot("chair")){
  // mc_rtc::log::info("Human pos is {}", robots().robot("human").posW().translation().transpose());
  // mc_rtc::log::info("Human pos is {}", datastore().get<sva::PTransformd>("ReplayPlugin::GetSegmentPose::HipsLink").translation().transpose());
  // robots().robot("chair").posW(robots().robot("human").posW() * sva::PTransformd(Eigen::Vector3d(0.05, 0.12, -0.65)));
  // mc_rtc::log::info("Chair pos is {}", robots().robot("chair").posW().translation().transpose());

  // robots().robot().posW(robots().robot("chair").posW() * sva::PTransformd(sva::RotZ(M_PI/2.0), Eigen::Vector3d(-0.17, -0.3, 0.75))); 
  // mc_rtc::log::info("Robot pos is {}", robots().robot().posW().translation().transpose());
  // }

  // }
  // Update constraints and resets posture tasks
  // solver().updateConstrSize();
  
}

void HelpUpController::computeStabilityRegion(std::shared_ptr<ContactSet> contactset, whatRobot rob, bool save, int polIndex, std::string suffix)
{
  switch (rob)
  {
  case hrp4:
    futureCompPoint_ = std::make_shared<ComputationPoint> (polIndex, contactset);
    futureCompPoint_->computeEquilibriumRegion();
    // futureCompPoint_->chebichevCenter(); // compute and store the chebichev center to avoid having glpk being called twice at the same time
    if (save)
      {
        futureCompPoint_->save("");
      }
    break;
  
  case human:
    futureHumCompPoint_ = std::make_shared<ComputationPoint> (polIndex, contactset);
    futureHumCompPoint_->computeEquilibriumRegion();
    // futureCompPoint_->chebichevCenter();
    if (save)
      {
        futureHumCompPoint_->save("");
      }
    break;
  }
  
}


bool HelpUpController::addTasksToSolver()
{
  // solver().addTask(comTask_); //We now use the stabilizer task to manage the com and not a simple com task 
  // human com is done in custom state

  solver().addTask(stabTask_); 

  // solver().addConstraintSet(*comIncPlaneConstraintPtr_);
  planes_ = {};
  planes(planes_, hrp4); //todo update planes
  
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
  double total_mass = realRobot().mass()+realRobot("human").mass();
  combinedCoM_ = ((robot().com()*robot().mass())+(robot("human").com()*robot("human").mass()))/total_mass;
  // (maybe better with com.cpp method from rbdyn)
}

void HelpUpController::addLogEntries()
{
  logger().addLogEntry("polytopeIndex", [this] () -> const int { return polytopeIndex_; });
  logger().addLogEntry("humPolytopeIndex", [this] () -> const int { return polytopeHumIndex_; });

  logger().addLogEntry("polytope_computationTime", [this]() -> const int { return currentCompPoint_->computationTime();});
  logger().addLogEntry("humPolytope_computationTime", [this]() -> const int { return currentHumCompPoint_->computationTime();});

  logger().addLogEntry("Back_surf_pos" , [this]() -> const auto { return BackSurf->X_0_s(robot("human"));});
  logger().addLogEntry("Shoulder_surf_pos" , [this]() -> const auto { return RightShoulderSurf->X_0_s(robot("human"));});

  // Logging the desired CoM computed
  auto desiredCoM = [this](){
    return comDesired_;
  };
  logger().addLogEntry("mainRob_com_desired", desiredCoM);

  auto desiredHumCoM = [this](){
    return comDesiredHum_;
  };
  logger().addLogEntry("humCom_desired", desiredHumCoM);

  // Logging the control CoM position computed by mc_rtc
  auto controlCoM = [this](){
    return this->robot().com();
  };
  logger().addLogEntry("mainRob_com_control", controlCoM);

  // Logging the estimated CoM position of the real robot estimated by the observers
  auto realCoM = [this](){
    return this->realRobot().com();
  };
  logger().addLogEntry("mainRob_com_real", realCoM);

  // Logging the control CoM position computed by mc_rtc
  auto humcontrolCoM = [this](){
    return this->robot("human").com();
  };
  logger().addLogEntry("human_com_control", controlCoM);

  // Logging the estimated CoM position of the real robot estimated by the observers
  auto humrealCoM = [this](){
    return this->realRobot("human").com();
  };
  logger().addLogEntry("human_com_real", realCoM);

  // Logging the CoM velocity
  auto controlCoMVel = [this](){
    return this->robot().comVelocity();
  };
  logger().addLogEntry("comVel_control", controlCoMVel);

  auto realCoMVel = [this](){
    return this->realRobot().comVelocity();
  };
  logger().addLogEntry("comVel_real", realCoMVel);

  // Logging the CoM acceleration
  auto realCoMAcc = [this](){
    return this->realRobot().comAcceleration();
  };
  logger().addLogEntry("comAcc_real", realCoMAcc);

  // Logging the CoM acceleration
  auto controlCoMAcc = [this](){
    return this->robot().comAcceleration();
  };
  logger().addLogEntry("comAcc_control", controlCoMAcc);

  // auto desiredCoMVel = [this](){
  //   return this->comp_c_;
  // };
  // logger().addLogEntry("comVel_desired", desiredCoMVel);

  auto xsensPos = [this](){
    return this->xsensCoMpos_;
  };
  logger().addLogEntry("xsensCoMpose", xsensPos);

  auto xsensVel = [this](){
    return this->xsensCoMvel_;
  };
  logger().addLogEntry("xsensCoMvel", xsensVel);

  auto xsensAcc = [this](){
    return this->xsensCoMacc_;
  };
  logger().addLogEntry("xsensCoMacc", xsensAcc);


  // Logging the pose of the right hand for control and real.
  auto controlRightHand = [this](){
    return this->robot().surfacePose("RightHand");
  };
  logger().addLogEntry("RightHandPose_control", controlRightHand);

  auto realRightHand = [this](){
    return this->realRobot().surfacePose("RightHand");
  };
  logger().addLogEntry("RightHandPose_real", realRightHand);

  auto controlLeftHand = [this](){
    return this->robot().surfacePose("LeftHand");
  };
  logger().addLogEntry("LeftHandPose_control", controlLeftHand);

  auto realLeftHand = [this](){
    return this->realRobot().surfacePose("LeftHand");
  };
  logger().addLogEntry("LeftHandPose_real", realLeftHand);

  auto measuredTorquesHuman = [this](){
    return this->realRobot("human").jointTorques();
  };
  logger().addLogEntry("Measured torques human", measuredTorquesHuman);

  // auto controlTorquesHuman = [this](){
  //   return this->robot("human").jointTorque().at(robot("human").jointIndexByName("RShin_0"));
  // };
  // logger().addLogEntry("Control torques human", controlTorquesHuman);

  auto measuredTorquesHRP4 = [this](){
    return this->realRobot().jointTorques();
  };
  logger().addLogEntry("Measured torques main robot", measuredTorquesHRP4);

  auto controlTorquesHRP4 = [this](){
    return this->robot().jointTorques();
  };
  logger().addLogEntry("Control torques main robot", controlTorquesHRP4);

  auto logDCMhum = [this](){
    return humanXsensDCM();
  };
  logger().addLogEntry("DCM_XsensDCM", logDCMhum);

  auto logdotDCMerror = [this](){
    return dotDCMerror();
  };
  logger().addLogEntry("DCM_human dot DCM error", logdotDCMerror);

  auto logdotDCMerrorv1 = [this](){
    return dotDCMerrorV1();
  };
  logger().addLogEntry("DCM_human dot DCM error V1", logdotDCMerrorv1);

  // auto logVRPhum = [this](){
  //   return humanXsensVRP();
  // };
  // logger().addLogEntry("DCM_XsensVRP", logVRPhum);

  auto logVRPhumModel = [this](){
    return humanVRPmodel();
  };
  logger().addLogEntry("DCM_VRPmodel", logVRPhumModel);

  auto logVRPhumMeasured = [this](){
    return humanVRPmeasured();
  };
  logger().addLogEntry("DCM_VRPmeasured", logVRPhumMeasured);

  auto logOmega = [this](){
    return humanOmega();
  };
  logger().addLogEntry("DCM_human omega", logOmega);

  auto logOmegaSquared = [this](){
    return humanOmega()*humanOmega();
  };
  logger().addLogEntry("DCM_human omega square", logOmegaSquared);

  auto logDotOmega = [this](){
    return dotHumanOmega();
  };
  logger().addLogEntry("DCM_human dot omega", logDotOmega);

  auto commandVRP = [this](){
    return desiredVRP();
  };
  logger().addLogEntry("DCM_desired VRP command", commandVRP);

  auto LFshoe = [this](){
    return LFShoe_;
  };
  logger().addLogEntry("ForceShoes_LFShoeMeasure", LFshoe);

  auto LBshoe = [this](){
    return LBShoe_;
  };
  logger().addLogEntry("ForceShoes_LBShoeMeasure", LBshoe);

  auto RFshoe = [this](){
    return RFShoe_;
  };
  logger().addLogEntry("ForceShoes_RFShoeMeasure", RFshoe);

  auto RBshoe = [this](){
    return RBShoe_;
  };
  logger().addLogEntry("ForceShoes_RBShoeMeasure", RBshoe);

  auto CoMforces = [this](){
    return humanVRPforces();
  };
  logger().addLogEntry("ForceShoes_MeasuredSum", CoMforces);

  auto Missingforces = [this](){
    return missingForces();
  };
  logger().addLogEntry("DCM_MissingForces", Missingforces);

  auto QPLH = [this](){
    return LHwrench_;
  };
  auto QPRH = [this](){
    return RHwrench_;
  };
  logger().addLogEntry("DCM_ComputedLHwrench", QPLH);
  logger().addLogEntry("DCM_ComputedRHwrench", QPRH);

  auto QPforces = [this](){
    return RedistribWrench_;
  };
  logger().addLogEntry("DCM_ComputedAssistanceForces", QPforces);

  // auto frontToBackL = [this](){
  //   auto X_LF_0 = robot("human").surfacePose("LFsensor").inv();
  //   auto X_0_LB = robot("human").surfacePose("LBsensor");
  //   // missing transform LF->world
  //   auto LFwrenchinWorld = X_LF_0.dualMul(LFShoe_);
  //   auto LFwrenchinLB = X_0_LB.dualMul(LFwrenchinWorld);
  //   return LFwrenchinLB;
  // };
  // logger().addLogEntry("ForceShoes_LBfromfront", frontToBackL);

}

void HelpUpController::addGuiElements()
{
  constexpr double DCM_POINT_SIZE = 0.015;
  constexpr double COM_POINT_SIZE = 0.02;
  constexpr double ARROW_HEAD_DIAM = 0.015;
  constexpr double ARROW_HEAD_LEN = 0.05;
  constexpr double ARROW_SHAFT_DIAM = 0.01;
  constexpr double FORCE_SCALE = 0.0015;

  const std::map<char, mc_rtc::gui::Color> COLORS =
    {
      {'r', mc_rtc::gui::Color{1.0, 0.0, 0.0}},
      {'g', mc_rtc::gui::Color{0.0, 1.0, 0.0}},
      {'b', mc_rtc::gui::Color{0.0, 0.0, 1.0}},
      {'y', mc_rtc::gui::Color{1.0, 0.5, 0.0}},
      {'c', mc_rtc::gui::Color{0.0, 0.5, 1.0}},
      {'m', mc_rtc::gui::Color{1.0, 0.0, 0.5}}
    };

  mc_rtc::gui::ArrowConfig forceArrowConfig;
  forceArrowConfig.shaft_diam = 1 * ARROW_SHAFT_DIAM;
  forceArrowConfig.head_diam = 1 * ARROW_HEAD_DIAM;
  forceArrowConfig.head_len = 1 * ARROW_HEAD_LEN;
  forceArrowConfig.scale = 1.;
  forceArrowConfig.start_point_scale = 0.02;
  forceArrowConfig.end_point_scale = 0.;

  mc_rtc::gui::ArrowConfig VRPforceArrowConfig = forceArrowConfig;
  VRPforceArrowConfig.color = COLORS.at('r');

  mc_rtc::gui::ArrowConfig DCMforceArrowConfig = forceArrowConfig;
  DCMforceArrowConfig.color = COLORS.at('c');

  mc_rtc::gui::ArrowConfig ShoesforceArrowConfig = forceArrowConfig;
  ShoesforceArrowConfig.color = COLORS.at('y');

  mc_rtc::gui::ArrowConfig MissingforceArrowConfig = forceArrowConfig;
  MissingforceArrowConfig.color = COLORS.at('g');
  

  gui()->addElement({"CoM"},
      mc_rtc::gui::Point3D("mainCoM", mc_rtc::gui::PointConfig(COLORS.at('y'), COM_POINT_SIZE), [this]() { return robot().com(); }),
      mc_rtc::gui::Point3D("mainCoMreal", mc_rtc::gui::PointConfig(COLORS.at('m'), COM_POINT_SIZE), [this]() { return realRobot().com(); }), // Note that this is the control robot com and not the real robot com 
      mc_rtc::gui::Point3D("humanCoM", mc_rtc::gui::PointConfig(COLORS.at('y'), COM_POINT_SIZE), [this]() { return robot("human").com(); }),
      mc_rtc::gui::Point3D("humanCoMreal", mc_rtc::gui::PointConfig(COLORS.at('m'), COM_POINT_SIZE), [this]() { return realRobot("human").com(); }),
      mc_rtc::gui::Point3D("humanCoMXsens", mc_rtc::gui::PointConfig(COLORS.at('b'), COM_POINT_SIZE), [this]() { return xsensCoMpos_; })
      // mc_rtc::gui::Point3D("CoMcombined", CoMconfig2, [this]() { return combinedCoM_; })
  );

  gui()->addElement({"DCM dynamics"},
      // mc_rtc::gui::Point3D("mainDCM", mc_rtc::gui::PointConfig(COLORS.at('b'), DCM_POINT_SIZE), [this]() { return mainCtlDCM(); }),
      mc_rtc::gui::Point3D("humanDCMXsens", mc_rtc::gui::PointConfig(COLORS.at('c'), DCM_POINT_SIZE), [this]() { return humanXsensDCM(); }),
      mc_rtc::gui::Point3D("humanVRPmodel", mc_rtc::gui::PointConfig(COLORS.at('y'), DCM_POINT_SIZE), [this]() { return humanVRPmodel(); }),
      mc_rtc::gui::Point3D("humanVRPmeasured", mc_rtc::gui::PointConfig(COLORS.at('r'), DCM_POINT_SIZE), [this]() { return humanVRPmodel(); }),
      // this is the computed vrp to achieve the desired xsensFinalpos_ (not sure)
      // mc_rtc::gui::Arrow("missingForces", MissingforceArrowConfig, [this]() -> Eigen::Vector3d { return desiredVRP(); }, [this]() -> Eigen::Vector3d { return xsensCoMpos_; }),
      mc_rtc::gui::Arrow("DCM-VRP", VRPforceArrowConfig, [this]() -> Eigen::Vector3d { return /*humanXsensVRP()*/ humanVRPmodel(); }, [this]() -> Eigen::Vector3d { return humanXsensDCM(); })
  
  );

  gui()->addElement({"Force Shoes"},
      mc_rtc::gui::Arrow("LFShoe", ShoesforceArrowConfig, [this]() -> Eigen::Vector3d { return robot("human").surfacePose("LFsensor").translation(); },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {return robot("human").surfacePose("LFsensor").translation() + FORCE_SCALE * LFShoe_.force(); }),
      mc_rtc::gui::Arrow("LBShoe", ShoesforceArrowConfig, [this]() -> Eigen::Vector3d { return robot("human").surfacePose("LBsensor").translation(); },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {return robot("human").surfacePose("LBsensor").translation() + FORCE_SCALE * LBShoe_.force(); }), 
      mc_rtc::gui::Arrow("RFShoe", ShoesforceArrowConfig, [this]() -> Eigen::Vector3d { return robot("human").surfacePose("RFsensor").translation(); },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {return robot("human").surfacePose("RFsensor").translation() + FORCE_SCALE * RFShoe_.force(); }),
      mc_rtc::gui::Arrow("RBShoe", ShoesforceArrowConfig, [this]() -> Eigen::Vector3d { return robot("human").surfacePose("RBsensor").translation(); },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {return robot("human").surfacePose("RBsensor").translation() + FORCE_SCALE * RBShoe_.force(); }),
      mc_rtc::gui::Arrow("CoMForce", ShoesforceArrowConfig, [this]() -> Eigen::Vector3d { return xsensCoMpos_; },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {return xsensCoMpos_ + FORCE_SCALE * humanVRPforces().force(); })
  );
  // gui()->addElement({"Trajectories"},
  //     mc_rtc::gui::Trajectory("Front trajectory", [this]() { return trajectories_->Front_Traj_; })
  //     // mc_rtc::gui::Trajectory("Back trajectory", [this]() { return trajectories_->Back_Traj_; })
   
  // );

  gui()->addElement({"Polytopes"}, 
      mc_rtc::gui::Polygon("MainRobBalanceRegion", COLORS.at('r'), [this]() { return balanceCompPoint_->getTriangles(); }),
      mc_rtc::gui::Polygon("HumanBalanceRegion", COLORS.at('g'), [this]() { return balanceHumCompPoint_->getTriangles(); }) 
  );

  // gui()->addElement({"AccPoly"}, 
  //     mc_rtc::gui::Polygon("HRP4accBalanceRegion", mc_rtc::gui::Color{0.8, 0., 0.}, [this]() { return accelerations_; }),
  //     mc_rtc::gui::Polygon("HumanaccBalanceRegion", mc_rtc::gui::Color{0., 0.8, 0.}, [this]() { return humaccelerations_; }) 
  // );

  mc_rtc::gui::PolyhedronConfig pconfig;
  pconfig.triangle_color = mc_rtc::gui::Color(0.2, 0.2, 0.2, 0.3);
  pconfig.use_triangle_color = true;
  pconfig.show_triangle = true;
  pconfig.show_vertices = true;
  pconfig.show_edges = true;
  pconfig.fixed_edge_color = true;
  pconfig.edge_config.color = mc_rtc::gui::Color::LightGray;
  pconfig.edge_config.width = 0.03;
  static bool publish_as_vertices_triangles = false;

    
  // gui()->addElement({"Polyhedrons"},
  //                        mc_rtc::gui::Polyhedron("Polyhedron", pconfig, [this]() { 
  //                         auto in = currentCompPoint_->getTriangles();
  //                         auto res = std::vector<std::array<Eigen::Vector3d, 3>>{};
  //                         for(const auto & v : in)
  //                         {
  //                           res.push_back({v[0], v[1], v[2]});
  //                         }
  //                         return res; })                    
  // );

  // gui()->addPlot(
  //   "Applied force",
  //   mc_rtc::gui::plot::X("t", [this]() { return t_; }),
  //   // mc_rtc::gui::plot::Y("RH Force", [this]() { return realRobot().forceSensor("RightHandForceSensor").force().z(); }, mc_rtc::gui::Color::Red),
  //   // mc_rtc::gui::plot::Y("LH Force", [this]() { return realRobot().forceSensor("LeftHandForceSensor").force().z(); }, mc_rtc::gui::Color::Green)
  //   mc_rtc::gui::plot::Y("RH Force", [this]() { return realRobot().forceSensor("RArm_6AF").force().z(); }, mc_rtc::gui::Color::Red),
  //   mc_rtc::gui::plot::Y("LH Force", [this]() { return realRobot().forceSensor("LArm_6AF").force().z(); }, mc_rtc::gui::Color::Green)
  // );
  


}

void HelpUpController::planes(std::vector<mc_rbdyn::Plane> constrPlanes, whatRobot rob)
{
  switch(rob)
  {
    case hrp4 : 
      planes_.clear();
      planes_ = constrPlanes;
      comIncPlaneConstraintPtr_->setPlanes(solver(), constrPlanes, {}, {}, 0.1, 0.03, 0.6, 0.0);
      break;
    case human :
      planesHum_.clear();
      planesHum_ = constrPlanes;
      comIncPlaneConstraintHumPtr_->setPlanes(solver(), constrPlanes, {}, {}, 0.1, 0.03, 0.6, 0.0);
      break;
    case combined :

      break;
  }

}

void HelpUpController::planes(std::vector<Eigen::Vector4d> constrPlanes, whatRobot rob)
{
  //LOG_INFO("Setting the comIncPlaneConstraint planes"
  std::vector<mc_rbdyn::Plane> p;

  for (auto plane: constrPlanes)
    {
      p.push_back({{-plane(0), -plane(1), -plane(2)}, plane(3)});
    }
  planes(p, rob);
  // planesUpdated_ = true;
}

// const std::vector<mc_rbdyn::Plane> HelpUpController::planes() const
// {
//   return planes_;
// }


void HelpUpController::increasePolytopeIndex(int polyIndex)
{
  polyIndex++;
}

int HelpUpController::getPolytopeIndex(int polyIndex)
{
  return polyIndex;
}

void HelpUpController::updateContactSet(unsigned int robotIndex) // todo update human outside of qp if too heavy to be under 5ms
{
  // updateRealHumContacts();
  auto contacts = solver().contacts();
  updateContactSet(contacts, robotIndex);
}

void HelpUpController::updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex)
{
  auto maxForces = config_("surfacesMaxForces");
  Eigen::Vector3d acceleration;
  const auto & robot = robots().robot(robotIndex);

  // accelerations_.resize(0);
  contactSet_ = std::make_shared<ContactSet> (false);
  contactSet_->mass(robots().robot(robotIndex).mass());
  contactSet_->setFrictionSides(6);
  

  for (auto contact: contacts)
    {
      if (contact.r1Index() == robotIndex || contact.r2Index() == robotIndex)
      {
        auto surface_name = (contact.r1Index() == robotIndex) ? contact.r1Surface()->name() : contact.r2Surface()->name();
        const auto & surface = robot.surface(surface_name);
        
        std::string bodyName = surface.bodyName();
        auto body_PT = robot.bodyPosW(bodyName);

        auto points = surface.points();
        int ptCpt = 0; // point counter
        std::string ptName;
        double mu = contact.friction(); // get the friction coef h
        
        double fmax;
        if (maxForces.has(surface_name))
          {
            fmax = maxForces(surface_name);
          }
        else
          {
            mc_rtc::log::warning( "Surface " + surface_name + " NOT found in the config for max force");
            fmax = robot.mass()*10;
          }
        // std::cout << "force value on "<< surface_name << realRobots().robot(robotIndex).forceSensor(surface_name+"ForceSensor").force(); // in hrp4 there are RightFootForceSensor LeftFootForceSensor RightHandForceSensor LeftHandForceSensor

        double fmin = 0; // todo set the same for min forces?
        ContactType type;
        if (config_.has("constrainedSurfaces"))
          {
            auto constrainedSurfaces = config_("constrainedSurfaces");
            if (constrainedSurfaces.has(surface_name))
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

        for (auto point: points)
          {
            auto pos = body_PT.rotation().transpose()*point.translation() + body_PT.translation();

            Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
            homTrans.block<3,3>(0,0) = body_PT.rotation().transpose()*point.rotation().transpose();
            homTrans.block<3,1>(0,3) = pos;

            ptName = surface.name() + "_" + std::to_string(ptCpt);
            contactSet_->addContact(ptName, homTrans, mu, fmax, fmin, type);

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
  // acceleration << robots().robot("")
  // accelerations_.push_back()

  acceleration << 0.6, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, 0.6, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << -0.6, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, -0.6, -9.81;
  contactSet_->addCoMAcc(acceleration);

  // std::cout << "#-----------------------------------------" << std::endl;
  // std::cout << "Displaying current contact Set" << std::endl;
  // contactSet_->showContactSet();

  readyForComp_ = false;
  computed_ = false;
  
  
}

void HelpUpController::contactForces(std::map<std::string, double> contactFMax, std::map<std::string, double> contactFMin)
{
  contactFMax_ = contactFMax;
  contactFMin_ = contactFMin;
}

void HelpUpController::updateContactForces()
{
  for (auto contactForce: contactFMax_)
    {
      contactSet_->setContactFMax(contactForce.second, contactForce.first);
    }
  for (auto contactForce: contactFMin_)
    {
      contactSet_->setContactFMin(contactForce.second, contactForce.first);
    }
}

// void HelpUpController::setFutureToCurrent(std::shared_ptr<ComputationPoint> current, std::shared_ptr<ComputationPoint> future)
// {
//   current = future;
//   planes(current->constraintPlanes());
//   desiredCoM(current->objectiveCoM(2, robot().com())); // todo same for human
// }// com computed on mode 2 (comQP, to be explored)

// void HelpUpController::setFutureToNext(std::shared_ptr<ComputationPoint> future, std::shared_ptr<ComputationPoint> next)
// {
//   next = future;
// }


void HelpUpController::setNextToCurrent(whatRobot rob)
{
  Eigen::Vector3d newCoM;
  switch(rob)
  {
    case hrp4 : 
      currentCompPoint_ = nextCompPoint_;
      planes(currentCompPoint_->constraintPlanes(), rob);
      if (planes_.size()>0)
      {
        newCoM = currentCompPoint_->objectiveCoM(0, robot().com()); // Here is set to mode 2 --> optimal com (qp) Chebychev qp is better: mode 1
        desiredCoM(newCoM, rob); 
      }
      break;
      
    case human :
      currentHumCompPoint_ = nextHumCompPoint_;
      planes(currentHumCompPoint_->constraintPlanes(), rob);
      if (planesHum_.size()>0)
      {
        newCoM = currentHumCompPoint_->objectiveCoM(0, robot("human").com());
        if (override_CoMz) // true if optional is set, false if "empty" (set in custom state if needed)
        {
          newCoM[2] = *override_CoMz;
        }
        // newCoM[2] = 0.75;
        desiredCoM(newCoM, rob);
      }
      
      break;
  }
  
}

 // auto currentCoM = robot().com();
 //  double currentZ = currentCoM(2);
 //  desiredCoM_(2)=currentZ;

 //  // set the planes and the desired CoM in the controler
 //  planes(desiredPlanes_);
 //  desiredCoM(desiredCoM_);

void HelpUpController::desiredCoM(Eigen::Vector3d desiredCoM, whatRobot rob)
{
  double coef = 0.01;
  Eigen::Vector3d prevCoM;
  switch(rob)
  {
    case hrp4 : 
      // setting com height manually, scaled as human stands up
      // hrp4 half sitting com: 0.78, human standing com: 0.87, e2dr half sitting com: 0.97
      // desiredCoM[2] = std::max((0.78*robot("human").com().z())/0.87 , 0.7);
      // desiredCoM[2] = std::max((0.95*robot("human").com().z())/0.87 , 0.9);
      // desiredCoM[2] = 0.95;
      desiredCoM[2] = 0.78;
      prevCoM = comDesired_;
      comDesired_ = (1-coef)*prevCoM + coef * desiredCoM;
      // comTask_->com(comDesired_); 
      // Instead of the general com task we use the stabilizer task to manage the robot com 
      stabTask_->staticTarget(comDesired_); 
      break;
    case human :
      // this is now deprecated and shouldn't be used since human com is managed by custom state
      desiredCoM[2] = 0.87;
      prevCoM = comDesiredHum_;
      comDesiredHum_ = (1-coef)*prevCoM + coef * desiredCoM;
      comTaskHum_->com(comDesiredHum_);
      break;
    case combined :

      break;
  }
  
}

Eigen::Vector3d HelpUpController::currentCoM(std::string robotName) const
{
  // return comTask_->actual();
  return robot(robotName).com();
}

void HelpUpController::comTaskWeight(double weight, std::shared_ptr<mc_tasks::CoMTask> CoMTask)
{
  CoMTask->weight(weight);
}

void HelpUpController::comTaskStiffness(double stiffness, std::shared_ptr<mc_tasks::CoMTask> CoMTask)
{
  CoMTask->stiffness(stiffness);
}

void HelpUpController::comTaskDamping(double damping, std::shared_ptr<mc_tasks::CoMTask> CoMTask)
{
  CoMTask->damping(damping);
}

// void HelpUpController::targetCoM(const Eigen::Vector3d & com, const Eigen::Vector3d & comp , const Eigen::Vector3d & compp)
// {
//   com_t_ = com;
//   comp_t_ = comp;
//   compp_t_ = compp;
// }

bool HelpUpController::isVertexInPlanes(Eigen::Vector3d Vertex, std::vector<Eigen::Vector4d> planes, double eps)
{
  bool isInside = true;
  Eigen::Vector3d normal;
  double offset;
  for (auto plane: planes)
    {
      normal << plane(0), plane(1), plane(2);
      offset = plane(3);
      if (normal.transpose()*Vertex > offset - eps)
	{
	  isInside = false;
	  break;
	}
    }
  return isInside;
}


std::map<std::string, double> HelpUpController::getConfigFMax() const
{
  auto maxForces = config_("surfacesMaxForces");
  std::map<std::string, double> configFMax;

  for (auto contact:contactSet_->get_contactNames())
    {
      // for each contact find the surface name (hopefully first part of the name
      auto underscore = contact.find("_");
      std::string surfaceName = contact.substr(0, underscore);

      double fmax = robot().mass() * 10;
      if (maxForces.has(surfaceName)) fmax = static_cast<double>(maxForces(surfaceName));
      else mc_rtc::log::warning("[HelpUpController] Surface {} is not in surfacesMaxForces", surfaceName);
      // then add the maxForce
      configFMax.emplace(contact, fmax);
    }

  return configFMax;
}

std::map<std::string, double> HelpUpController::getConfigFMin() const
{
  auto minForces = config_("surfacesMinForces");
  std::map<std::string, double> configFMin;

  for (auto contact:contactSet_->get_contactNames())
    {
      // for each contact find the surface name (hopefully first part of the name
      auto underscore = contact.find("_");
      std::string surfaceName = contact.substr(0, underscore);

      double fmin = robot().mass() * 10;
      if (minForces.has(surfaceName)) fmin = static_cast<double>(minForces(surfaceName));
      else mc_rtc::log::warning("[HelpUpController] Surface {} is not in surfacesMinForces", surfaceName);
      // then add the maxForce
      configFMin.emplace(contact, fmin);
    }

  return configFMin;
}

void HelpUpController::updateRealHumContacts()
{
  
  RFootGround->update(robot("human"), robot("ground"));
  LFootGround->update(robot("human"), robot("ground"));
  RCheekChair->update(robot("human"), robot("chair"));
  LCheekChair->update(robot("human"), robot("chair"));
  // RHandShoulder->update(robot(), robot("human"));
  // LHandBack->update(robot(), robot("human"));

  double distThreshold = 0.005;
  double speedThreshold = 1e-4;

  // for (auto contact:solver().contacts())
  // {
  //  std::cout<<contact.contactId(robots())<<std::endl;
  // }
  
  // std::cout<<"-------------------------------- human contacts"<<std::endl;

  // std::cout<<"top pose is"<<robot("chair").surfacePose("Top").translation()<<std::endl;
  // std::cout<<"rcheek pose is"<<robot("human").surfacePose("RCheek").translation()<<std::endl;
  // std::cout<<"rcheek error is"<<RCheekError.translation().norm()<<std::endl;

  contactSetHum_ = std::make_shared<ContactSet> (false);
  contactSetHum_->mass(humanMass_);
  contactSetHum_->setFrictionSides(6);

  // std::cout<<RFootGround->pair.getDistance()<<std::endl;
  // std::cout<<LFootGround->pair.getDistance()<<std::endl;
  // std::cout<<RCheekChair->pair.getDistance()<<std::endl;
  // std::cout<<LCheekChair->pair.getDistance()<<std::endl;
  // std::cout<<RHandShoulder->pair.getDistance()<<std::endl;
  // std::cout<<LHandBack->pair.getDistance()<<std::endl;
  


  // if (RFootGround->pair.getDistance()<=distThreshold)   // Distance is low enough to consider contact
  // {
  //   addRealHumContact("RightSole", 0, humanMass_*9.81, ContactType::support);
  //   // std::cout<<"adding right sole"<<std::endl;
  // }

  // if (LFootGround->pair.getDistance()<=distThreshold)
  // {
  //   addRealHumContact("LeftSole", 0, humanMass_*9.81, ContactType::support);
  //   // std::cout<<"adding left sole"<<std::endl;
  // }

  // We switch to a force shoes condition
  if (RFShoe_.force().z() + RBShoe_.force().z() >= 20.)   // Vertical force is high enough to consider contact
  {
    addRealHumContact("RightSole_ForceShoe", 0, humanMass_*9.81, ContactType::support);
    // std::cout<<"adding right sole"<<std::endl;
  }

  if (LFShoe_.force().z() + LBShoe_.force().z() >= 20.)
  {
    addRealHumContact("LeftSole_ForceShoe", 0, humanMass_*9.81, ContactType::support);
    // std::cout<<"adding left sole"<<std::endl;
  }

  if (RCheekChair->pair.getDistance()<=distThreshold)
  {
    addRealHumContact("RCheek", 0, humanMass_*9.81, ContactType::support);
    // std::cout<<"adding right cheek"<<std::endl;
  }

  if (LCheekChair->pair.getDistance()<=distThreshold)
  {
    addRealHumContact("LCheek", 0, humanMass_*9.81, ContactType::support);
    // std::cout<<"adding left cheek"<<std::endl;
  }

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

  acceleration << 0.5, 0, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << 0, 0.5, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << -0.5, 0, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  acceleration << 0, -0.5, -9.81;
  contactSetHum_->addCoMAcc(acceleration);

  computedHum_ = false;


}


void HelpUpController::addRealHumContact(std::string humanSurfName, double fmin, double fmax, ContactType type)
{
  const auto & surf = robot("human").surface(humanSurfName);
  auto points = surf.points();
    
  int ptCpt = 0; // point counter
  std::string ptName;
  double mu = 0.7; // friction coef h

  auto surf_PT = surf.X_0_s(robot("human"));

  for (auto point:points)
  {
    auto pos = surf_PT.rotation().transpose()*point.translation() + surf_PT.translation();

    Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
    homTrans.block<3,3>(0,0) = surf_PT.rotation().transpose()*point.rotation().transpose();
    homTrans.block<3,1>(0,3) = pos;

    ptName = humanSurfName + "_" + std::to_string(ptCpt);
    contactSetHum_->addContact(ptName, homTrans, mu, fmax, fmin, type);

    ptCpt++;
  }
}

sva::ForceVecd HelpUpController::getCurrentForceVec(std::vector<sva::ForceVecd> log , double startOffset, double freq)
{
  double acqTimeStep = 1/freq;
  if (t_ < startOffset)
  {
    return log[0];
  }
  else if (t_ >= startOffset && int(t_/acqTimeStep) - int(startOffset/acqTimeStep) < log.size())
  {
    // mc_rtc::log::info("acq timestep is {}, so t_/acq = {} and t/acq - start/step = {}", acqTimeStep, int(t_/acqTimeStep), int(t_/acqTimeStep) - int(startOffset/acqTimeStep));
    return log[int(t_/acqTimeStep) - int(startOffset/acqTimeStep)];
  }
  else
  {
    // mc_rtc::log::info(" else: acq timestep is {}, so t_/acq = {} and t/acq - start/step = {}", acqTimeStep, int(t_/acqTimeStep), int(t_/acqTimeStep) - int(startOffset/acqTimeStep));
    return log.back();
  }
  
}

void HelpUpController::distributeHandsWrench(const sva::ForceVecd & desiredWrench)
  {
    // Variables
    // ---------
    // x = [w_lh_lhc w_rh_rhc] where
    // w_lh_lhc: spatial force vector of robot left hand contact in left hand contact frame: world frame and centroidal frame share the same orientation (should it be inertial frame, ie at contact instead of world?)
    // w_rh_rhc: spatial force vector of robot right hand contact in right hand contact frame

    // Objective
    // ---------
    // Weighted minimization of the following tasks:
    // w_lh_C + w_rh_C == desiredWrench  -- realize desired wrench at human CoM
    // (X_0_lhc* w_lh_0).z() == 0 -- minimize left hand force applied on human
    // (X_0_rhc* w_rh_0).z() == 0 -- minimize right hand force applied on human
    
    // maybe add minimization of arm torques?


    // w_l_lankle == 0 -- minimize left foot ankle torque (anisotropic weight) (=smallest value possible?)
    // w_r_rankle == 0 -- minimize right foot ankle torque (anisotropic weight)
    // Constraints
    // -----------
    // CWC X_0_lhc* w_lh_0 <= 0  -- left hand wrench within contact wrench cone
    // CWC X_0_rhc* w_rh_0 <= 0  -- right hand wrench within contact wrench cone
    // (X_0_lhc* w_lh_0).z() > minPressure  -- minimum left hand contact pressure to maintain contact
    // (X_0_rhc* w_rh_0).z() > minPressure  -- minimum right hand contact pressure
    // (X_0_lhc* w_lh_0).z() + (X_0_rhc* w_rh_0).z() < maxPressure -- maximum pressure on the human
   

    // const auto & leftHandContact = contacts_.at(ContactState::Left);
    // Create hand contacts 
    // leftHandContact_ = std::make_shared<mc_tasks::lipm_stabilizer::internal::Contact>(robot(), "LeftHand", 0.7);
    // rightHandContact_ = std::make_shared<mc_tasks::lipm_stabilizer::internal::Contact>(robot(), "RightHand", 0.7);

    leftHandContact_ = std::make_shared<mc_tasks::lipm_stabilizer::internal::Contact>(robot("human"), "Back", 0.7);
    rightHandContact_ = std::make_shared<mc_tasks::lipm_stabilizer::internal::Contact>(robot("human"), "RightShoulder", 0.7);

    
    const auto & leftHandContact = *leftHandContact_;
    const auto & rightHandContact = *rightHandContact_;
    const sva::PTransformd & X_0_lhc = leftHandContact.surfacePose();
    const sva::PTransformd & X_0_rhc = rightHandContact.surfacePose();
    // sva::PTransformd X_0_vrp(desiredVRP()); // getting vrp transform
    sva::PTransformd X_0_C(xsensCoMpos_); // getting CoM transform
    sva::PTransformd X_lhc_c = X_0_lhc.inv() * X_0_C;
    sva::PTransformd X_rhc_c = X_0_rhc.inv() * X_0_C;
  

    constexpr unsigned NB_VAR = 6 + 6; // 6d wrench * 2 contacts
    constexpr unsigned COST_DIM = 6 + NB_VAR; // 6 for desired wrench, nb_var to minimize our variables individually
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    A.setZero(COST_DIM, NB_VAR);
    b.setZero(COST_DIM);


    // |w_lh_c + w_rh_c - desiredWrench|^2
    // We handle moments around the ZMP instead of the world origin to avoid numerical errors due to large moment values.
    // https://github.com/jrl-umi3218/mc_rtc/pull/285
    // Why zmp and not CoM ? In our case CoM seems more appropriate: desired wrench is the result of the various contacts on the CoM
    // Thus minimize difference between result of hand contact wrenches on the CoM and missing forces to apply (desired wrench)
    
    auto A_net = A.block<6, 12>(0, 0); 
    auto b_net = b.segment<6>(0);
    A_net.block<6, 6>(0, 0) = X_lhc_c.dualMatrix(); // matrix transform for wrench from hand contact to com (since variable is hand contact wrench)
    A_net.block<6, 6>(0, 6) = X_rhc_c.dualMatrix();
    // b_net = X_0_vrp.dualMul(desiredWrench).vector(); ? since desired wrench is on the com maybe just this?
    b_net = desiredWrench.vector();

    // |wrench_lh|^2 
    auto A_lhwrench = A.block<6, 6>(6, 0);
    // |wrench_rh|^2 
    auto A_rhwrench = A.block<6, 6>(12, 6);
    // anisotropic weights:  taux, tauy, tauz,   fx,   fy,   fz;
    A_lhwrench.diagonal() <<     1.,   1.,   1.,   1.,   1.,   1.; // 1.,   1., 1e-4, 1e-3, 1e-3, 1e-4;
    A_rhwrench.diagonal() <<     1.,   1.,   1.,   1.,   1.,   1.;
    // A_lhwrench *= X_0_lhc.dualMatrix(); I don't think transforms are needed since variables are in contact frame and those are the ones we want to minimize
    // A_rhwrench *= X_0_rhc.dualMatrix();
    // corresponding b vector is zero since we want to minimize (closest to zero)


    // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
    // this is to be sure to have an equal repartition

    // double lfr = leftFootRatio_;
    // auto A_pressure = A.block<1, 12>(18, 0);
    // A_pressure.block<1, 6>(0, 0) = (1 - lfr) * X_0_lc.dualMatrix().bottomRows<1>();
    // A_pressure.block<1, 6>(0, 6) = -lfr * X_0_rc.dualMatrix().bottomRows<1>();


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
    // Two contacts with cwc constraints + two min pressure constraints + one max pressure constraint
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

    // w_l_lc.force().z() >= min_pressure
    double minHandsPressure = 10.; // minimal contact force in Newtons
    A_ineq.block(nb_const - 3, 0, 1, 6) = -Eigen::Matrix6d::Identity().bottomRows<1>();  // selecting force only, identity bc va(riable is already in contact frame //-X_0_lhc.dualMatrix().bottomRows<1>();
    b_ineq(nb_const - 3) = -minHandsPressure;
    // w_r_rc.force().z() >= min_pressure
    A_ineq.block(nb_const - 2, 6, 1, 6) = -Eigen::Matrix6d::Identity().bottomRows<1>(); //-X_0_rhc.dualMatrix().bottomRows<1>();
    b_ineq(nb_const - 2) = -minHandsPressure;
    // (X_0_lhc* w_lh_0).z() + (X_0_rhc* w_rh_0).z() < maxPressure
    double maxCompression = 200.; // max pressure applied total on torso
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
