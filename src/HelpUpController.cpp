#include "HelpUpController.h"

HelpUpController::HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), polytopeIndex_(0), polytopeHumIndex_(0), computed_(false), computedHum_(false), computing_(false), 
computingHum_(false), transitionning_(false), transitionningHum_(false), readyForComp_(false), readyForCompHum_(false), 
humanSolver_(std::make_shared<mc_solver::QPSolver>(dt))
{
  // Load entire controller configuration file
  config_.load(config);

  // humanSolver_->gui(gui_);
  
  /* Observers
  */
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) { // robot() is the main robot (hrp4)
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
  });
  
  // datastore().make_call("KinematicAnchorFrame::human" , [this](const mc_rbdyn::Robot & robot) { // robot 3 (out of 4) is the human robots().robots()[3].name()
  //   return sva::interpolate(robot.surfacePose("LeftSole"), robot.surfacePose("RightSole"), 0.5);
  // });
  
  // HumanSolver_ = mc_solver::QPSolver();

  comIncPlaneConstraintPtr_ = std::make_shared<mc_solver::CoMIncPlaneConstr> (robots(), robots().robotIndex(), dt);
  comIncPlaneConstraintHumPtr_ = std::make_shared<mc_solver::CoMIncPlaneConstr> (robots(), robots().robotIndex("human"), dt);

  // comAccConstr_ = std::make_shared<BoundCoMAcceleration> (robots(), robots().robotIndex(), dt);

  // comVelConstr_ = std::make_shared<BoundCoMVelocity>(robots(), robots().robotIndex(), dt);

  // initialize the current computation point:
  currentCompPoint_ = std::make_shared<ComputationPoint>  (-1, std::make_shared<ContactSet>(false));
  currentHumCompPoint_ = std::make_shared<ComputationPoint>  (-1, std::make_shared<ContactSet>(false));
  
  comTask_ = std::make_shared<mc_tasks::CoMTask> (robots(), robots().robotIndex(), 5.0, 2e3); // Stiffness 5, weight 1000
  comTask_->damping(10.0); 
  // comTask_->com(realRobot().com());
  comDesired_ = robot().com();
  comTask_->com(robot().com()); 

  comTaskHum_ = std::make_shared<mc_tasks::CoMTask> (robots(), robots().robotIndex("human"), 5.0, 2e3); // Stiffness 5, weight 1000
  comTaskHum_->damping(10.0); 
  comDesiredHum_ = robot("human").com();
  comTaskHum_->com(robot("human").com());
  
  addLogEntries();
  addGuiElements();
  addTasksToSolver();
  

  mc_rtc::log::success("HelpUpController init done ");
}


bool HelpUpController::run()
{
  updateCombinedCoM();
  if (!computing_)
  {
    // update the contact set model 
    if (!readyForComp_)
    {
      computed_ = false;
      updateContactSet(robots().robotIndex("hrp4"), hrp4);
      // Instead of keeping the max forces as given in the config we update them as the current applied force on the contact ?
      // Not a good idea, only limiting. It is better to change the max force as what we allow the robot to exert (to stay safe for the human),
      // and the min force (necessary to help the human stand?)
      // See change in updateContactForces()
      updateContactForces();
      readyForComp_ = true;
    }

    // start the computation
    if (!computed_ and readyForComp_)
    {
      polytopeIndex_++;
      polytopeReady_ = false;

      stabThread_ = std::thread([this](int polIndex){
        this->computeStabilityRegion(contactSet_, hrp4, false, polIndex);
        polytopeReady_ = true;
      }, polytopeIndex_);
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
      // setFutureToNext(futureCompPoint_, nextCompPoint_);
      transitionning_ = true;
    }
  }

  if (!computingHum_)
  {
    // update the contact set model 
    if (!readyForCompHum_)
    {
      computedHum_ = false;
      updateContactSet(robots().robotIndex("human"), human);

      // Instead of keeping the max forces as given in the config we update them as the current applied force on the contact ?
      // Not a good idea, only limiting. It is better to change the max force as what we allow the robot to exert (to stay safe for the human),
      // and the min force (necessary to help the human stand?)
      // See change in updateContactForces()
      updateContactForces();
      readyForCompHum_ = true;
    }

    // start the computation
    if (!computedHum_ and readyForCompHum_)
    {
      polytopeHumIndex_++;
      polytopeHumReady_ = false;

      stabThreadHum_ = std::thread([this](int polIndex){
        this->computeStabilityRegion(contactSetHum_, human, true, polIndex);
        polytopeHumReady_ = true;
      }, polytopeHumIndex_);
      computingHum_ = true;
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
      // setFutureToNext(futureHumCompPoint_, nextHumCompPoint_);
      transitionningHum_ = true;
    }
  }
  // // update the desired CoM Acceleration:
  // Eigen::Vector3d comPos = robot("hrp4").com();
  // Eigen::Vector3d comVel = realRobot("hrp4").comVelocity();
  // Eigen::Vector3d comAcc = realRobot("hrp4").comAcceleration();
  
  // Eigen::Vector3d gradientPotential = Eigen::Vector3d::Zero();
  // gradientPotential = currentCompPoint_->computeGradient(comPos);


  double dt = timeStep;
  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, 9.81;
  double mass = robot().mass();

  Eigen::Vector3d newVel = Eigen::Vector3d::Zero();

  // admittance tasks?
  

  // transition
  if (transitionning_)
    {
      // mc_rtc::log::info("HelpUpController transitionning ");
      // if we are not in next
      Eigen::Vector3d currentCoM = realRobot("hrp4").com();
      nextCompPoint_->constraintPlanes();  
      if (isVertexInPlanes(currentCoM, nextCompPoint_->constraintPlanes(), 0.03))
      {
        setNextToCurrent(hrp4);
        transitionning_ = false;
      }
    }

  // transition
  if (transitionningHum_)
    {
      // mc_rtc::log::info("HelpUpController transitionning ");
      // if we are not in next
      Eigen::Vector3d currentCoM = realRobot("human").com();    
      if (isVertexInPlanes(currentCoM, nextHumCompPoint_->constraintPlanes(), 0.03))
      {
        setNextToCurrent(human);
        transitionningHum_ = false;
      }
    }

  bool ok = mc_control::fsm::Controller::run();

  // bool ok2;
  // if (!humanSolver_->run())
  // {
  //   mc_rtc::log::error("QP human failed to run()");
  //   ok2=false;
  // }
  // else ok2=true;
   
  // mc_rtc::log::success("HelpUpController has ran iteration ok ");

  return ok;
}

void HelpUpController::reset(const mc_control::ControllerResetData & reset_data)
{
  // Update dynamics constraints
  // dynamicsConstraint = mc_solver::DynamicsConstraint(robots(),
  //                                                    robot().robotIndex(),
  //                                                    solver().dt(), {0.1, 0.01, 0.5});
  // // Must be added to the solver before controller reset
  // solver().addConstraintSet(dynamicsConstraint);

  // Add human dynamics constraints
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
  //   // Adjust chair position relative to human model
  //   // if(robots().hasRobot("chair")){
  //   //   robots().robot("chair").posW(robots().robot("human").posW() * sva::PTransformd(Eigen::Vector3d(0.05, 0.0, -0.65)));
  //   // }

  // }
  // Update constraints and resets posture tasks
  // solver().updateConstrSize();
  mc_control::fsm::Controller::reset(reset_data);
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
  solver().addTask(comTask_);
  // desiredCoM(robot().com(), hrp4); // Not required : init com objective is set in controller init
  solver().addConstraintSet(*comIncPlaneConstraintPtr_);
  planes_ = {};
  planes(planes_, hrp4); //todo update planes

  // solver().addTask(comTaskHum_); // todo different solver? qp human
  // desiredCoM(robot("human").com(), human);
  solver().addConstraintSet(*comIncPlaneConstraintHumPtr_);
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
  double total_mass = realRobot("hrp4").mass()+realRobot("human").mass();
  combinedCoM_ = ((realRobot("hrp4").com()*realRobot("hrp4").mass())+(realRobot("human").com()*realRobot("human").mass()))/total_mass;
  // (maybe better with com.cpp method from rbdyn)
}

void HelpUpController::addLogEntries()
{
  logger().addLogEntry("polytopeIndex", [this] () -> const int { return polytopeIndex_; });
  logger().addLogEntry("polytopeIndex", [this] () -> const int { return polytopeHumIndex_; });

  logger().addLogEntry("polytope_computationTime", [this]() -> const int { return currentCompPoint_->computationTime();});
  logger().addLogEntry("polytope_computationTime", [this]() -> const int { return currentHumCompPoint_->computationTime();});

  // Logging the desired CoM computed
  auto desiredCoM = [this](){
    return comDesired_;
  };
  logger().addLogEntry("com_desired", desiredCoM);

  auto desiredHumCoM = [this](){
    return comDesiredHum_;
  };
  logger().addLogEntry("com_desired", desiredHumCoM);

  // Logging the control CoM position computed by mc_rtc
  auto controlCoM = [this](){
    return this->robot().com();
  };
  logger().addLogEntry("com_control", controlCoM);

  // Logging the estimated CoM position of the real robot estimated by the observers
  auto realCoM = [this](){
    return this->realRobot().com();
  };
  logger().addLogEntry("com_real", realCoM);

  // Logging the CoM velocity
  auto controlCoMVel = [this](){
    return this->robot().comVelocity();
  };
  logger().addLogEntry("comVel_control", controlCoMVel);

  auto realCoMVel = [this](){
    return this->realRobot().comVelocity();
  };
  logger().addLogEntry("comVel_real", realCoMVel);

  // auto desiredCoMVel = [this](){
  //   return this->comp_c_;
  // };
  // logger().addLogEntry("comVel_desired", desiredCoMVel);


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
    return this->realRobot("hrp4").jointTorques();
  };
  logger().addLogEntry("Measured torques hrp4", measuredTorquesHRP4);

  // auto controlTorquesHRP4 = [this](){
  //   return this->robot("hrp4").jointTorque().at(robot("hrp4").jointIndexByName("R_KNEE_P"));
  // };
  // logger().addLogEntry("Control torques hrp4", controlTorquesHRP4);

  
  // logging CoM Task weight
  // auto comTaskWeight = [this](){
  //   return this->comTask_->weight();
  // };
  // logger().addLogEntry("com_taskWeight", comTaskWeight);

  // logging the potential and the gradient of the potential:
  // auto potential = [this](){
  //   return this->currentCompPoint_->computePotential(this->robot("hrp4").com());
  // };
  // logger().addLogEntry("potential_potential", potential);
  
  // auto gradPotential = [this](){
  //   return this->currentCompPoint_->computeGradient(this->robot("hrp4").com());
  // };
  // logger().addLogEntry("potential_gradient", gradPotential);
  
  //add the comAccelerationTask in the logger
  // comAccelerationTaskKinematicPtr_->addToLogger(logger());
  // comAccelerationTaskDynamicPtr_->addToLogger(logger());
  // also log the target com acc in another place:
//   auto targetCoMAcc = [this](){
//     return targetCoMAcceleration_;
//   };
//   logger().addLogEntry("comAcc_target", targetCoMAcc);

}

void HelpUpController::addGuiElements()
{
  mc_rtc::gui::PointConfig CoMconfig1(mc_rtc::gui::Color{1., 0.5, 0.}, 0.05);
  mc_rtc::gui::PointConfig CoMconfig2(mc_rtc::gui::Color{0., 1., 1.}, 0.07);

  gui()->addElement({"CoM"},
      mc_rtc::gui::Point3D("CoMhrp4", CoMconfig1, [this]() { return realRobot("hrp4").com(); }), // Note that this is the control robot com and not the real robot com 
      mc_rtc::gui::Point3D("CoMhuman", CoMconfig1, [this]() { return realRobot("human").com(); }),
      mc_rtc::gui::Point3D("CoMcombined", CoMconfig2, [this]() { return combinedCoM_; })
  );

  gui()->addElement({"Polytopes"}, 
      mc_rtc::gui::Polygon("HRP4BalanceRegion", mc_rtc::gui::Color{1., 0., 0.}, [this]() { return currentCompPoint_->getTriangles(); }),
      mc_rtc::gui::Polygon("HumanBalanceRegion", mc_rtc::gui::Color{0., 1., 0.}, [this]() { return currentHumCompPoint_->getTriangles(); }) 
  );

  // todo: implement mc_rtc::gui::Polytope
  // gui()->addElement({"Polytopes"},
  //     mc_rtc::gui::Polytope("HRP4BalanceRegion", [this]() { return currentCompPoint_->getTriangles(); })
  // );
  


}

void HelpUpController::planes(std::vector<mc_rbdyn::Plane> constrPlanes, whatRobot rob)
{
  switch(rob)
  {
    case hrp4 : 
      planes_.clear();
      planes_ = constrPlanes;
      comIncPlaneConstraintPtr_->set_planes(solver(), constrPlanes, {}, {}, 0.1, 0.03, 0.6, 0.0);
      break;
    case human :
      planesHum_.clear();
      planesHum_ = constrPlanes;
      comIncPlaneConstraintHumPtr_->set_planes(solver(), constrPlanes, {}, {}, 0.1, 0.03, 0.6, 0.0);
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

void HelpUpController::updateContactSet(unsigned int robotIndex, whatRobot rob)
{
  auto contacts = solver().contacts();
  updateContactSet(contacts, robotIndex, rob);
}

void HelpUpController::updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex, whatRobot rob)
{
  auto maxForces = config_("surfacesMaxForces");
  Eigen::Vector3d acceleration;
  const auto & robot = robots().robot(robotIndex);
  switch (rob)
  {
  case hrp4:
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

    // Adding the accelerations
    

    acceleration << 0.0, 0.0, -9.81;
    contactSet_->addCoMAcc(acceleration);

    acceleration << 0.8, 0, -9.81;
    contactSet_->addCoMAcc(acceleration);

    acceleration << 0, 0.6, -9.81;
    contactSet_->addCoMAcc(acceleration);

    acceleration << -0.8, 0, -9.81;
    contactSet_->addCoMAcc(acceleration);

    acceleration << 0, -0.6, -9.81;
    contactSet_->addCoMAcc(acceleration);

    // std::cout << "#-----------------------------------------" << std::endl;
    // std::cout << "Displaying current contact Set" << std::endl;
    // contactSet_->showContactSet();

    readyForComp_ = false;
    computed_ = false;
    break;
  
  case human:
    contactSetHum_ = std::make_shared<ContactSet> (false);
    contactSetHum_->mass(robots().robot(robotIndex).mass());
    contactSetHum_->setFrictionSides(6);
    

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
              contactSetHum_->addContact(ptName, homTrans, mu, fmax, fmin, type);

              ptCpt++;
            }
        }
      }

    // Adding the accelerations
    

    acceleration << 0.0, 0.0, -9.81;
    contactSetHum_->addCoMAcc(acceleration);

    acceleration << 0.8, 0, -9.81;
    contactSetHum_->addCoMAcc(acceleration);

    acceleration << 0, 0.6, -9.81;
    contactSetHum_->addCoMAcc(acceleration);

    acceleration << -0.8, 0, -9.81;
    contactSetHum_->addCoMAcc(acceleration);

    acceleration << 0, -0.6, -9.81;
    contactSetHum_->addCoMAcc(acceleration);

    // std::cout << "#-----------------------------------------" << std::endl;
    // std::cout << "Displaying current contact Set" << std::endl;
    // contactSet_->showContactSet();

    readyForCompHum_ = false;
    computedHum_ = false;
    break;
  
  case combined:
    break;
  }
  
  
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
      newCoM = currentCompPoint_->objectiveCoM(1, realRobot().com()); // Here is set to mode 2 --> optimal com (qp) Chebychev qp is better: mode 1
      // if (override_CoMz) // true if optional is set, false if "empty" (set in custom state if needed)
      // {
      //   newCoM[2] = *override_CoMz;
      // }
      // newCoM[2] = 0.75; // Overwriting of z axis, which set to current z otherwise (instead of z of cheb center)
      desiredCoM(newCoM, rob); 
      break;
    case human :
      currentHumCompPoint_ = nextHumCompPoint_;
      planes(currentHumCompPoint_->constraintPlanes(), rob);
      newCoM = currentHumCompPoint_->objectiveCoM(1, realRobot("human").com());
      if (override_CoMz) // true if optional is set, false if "empty" (set in custom state if needed)
      {
        newCoM[2] = *override_CoMz;
      }
      // newCoM[2] = 0.75;
      desiredCoM(newCoM, rob);
      break;
    case combined :

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
  double coef = 0.1;
  Eigen::Vector3d prevCoM;
  switch(rob)
  {
    case hrp4 : 
      prevCoM = comDesired_;
      comDesired_ = (1-coef)*prevCoM + coef * desiredCoM;
      comTask_->com(comDesired_);
      break;
    case human :
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
  return realRobot(robotName).com();
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
