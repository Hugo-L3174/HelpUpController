#include "HelpUpController.h"

HelpUpController::HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), polytopeIndex_(0), computed_(false), computing_(false), transitionning_(false),
readyForComp_(false)
{
  // Load entire controller configuration file
  config_.load(config);

  /* Observers
  */
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) { // robot() is the main robot (hrp4)
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
  });
  
  datastore().make_call("KinematicAnchorFrame::human" , [this](const mc_rbdyn::Robot & robot) { // robot 3 (out of 4) is the human robots().robots()[3].name()
    return sva::interpolate(robot.surfacePose("LeftSole"), robot.surfacePose("RightSole"), 0.5);
  });


  comIncPlaneConstraintPtr_ = std::make_shared<mc_solver::CoMIncPlaneConstr> (robots(), robots().robotIndex(), dt); // todo: verify if needed for human


  // comAccConstr_ = std::make_shared<BoundCoMAcceleration> (robots(), robots().robotIndex(), dt);

  // comVelConstr_ = std::make_shared<BoundCoMVelocity>(robots(), robots().robotIndex(), dt);

  // initialize the current computation point:
  currentCompPoint_ = std::make_shared<ComputationPoint>  (-1, std::make_shared<ContactSet>(false));
  
  
  addLogEntries();
  // addGuiElements();


  mc_rtc::log::success("HelpUpController init done ");
}

//ok debug

bool HelpUpController::run()
{
  // mc_rtc::log::info("HelpUpController run iteration ");
  if (!computing_)
  {
    // mc_rtc::log::info("HelpUpController computing ");
    // update the contact set model 
    if (!readyForComp_)
    {
      computed_ = false;
      updateContactSet(robots().robotIndex("hrp4")); // todo, do the same for the human
      updateContactForces();
      readyForComp_ = true;
      // mc_rtc::log::info("HelpUpController ready for comp ");
    }

    // start the computation
    if (!computed_ and readyForComp_)
    {
      // mc_rtc::log::info("HelpUpController polytoping ");
      increasePolytopeIndex();
      polytopeReady_ = false;

      stabThread_ = std::thread([this](int polIndex){
        this->computeStabilityRegion(true, polIndex);
        polytopeReady_ = true;
      }, polytopeIndex_);
      computing_ = true;
      readyForComp_ = false;
    }
  }
  else // computing_ == true => currently computing
  {
    // check if the computation is finished
    // mc_rtc::log::info("HelpUpController waiting polytope ");
    if (polytopeReady_ and stabThread_.joinable())
    {
      // mc_rtc::log::info("HelpUpController updating poly ");
      stabThread_.join();
      computing_ = false;
      computed_ = true;
      // add the stuff to update the polytope
      setFutureToNext();
      startTransition();
      // mc_rtc::log::info("HelpUpController new poly set ");
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
           
      if (isVertexInPlanes(currentCoM, nextCompPoint_->constraintPlanes(), 0.03))
      {
        setNextToCurrent();
        transitionning_ = false;
      }
    }

  bool ok = mc_control::fsm::Controller::run();

  // mc_rtc::log::success("HelpUpController has ran iteration ok ");

  return ok;
}

void HelpUpController::reset(const mc_control::ControllerResetData & reset_data)
{
  // Update dynamics constraints
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(),
                                                     robot().robotIndex(),
                                                     solver().dt(), {0.1, 0.01, 0.5});
  // Must be added to the solver before controller reset
  solver().addConstraintSet(dynamicsConstraint);

  // Add human dynamics constraints
  if(robots().hasRobot("human")){
    humanDynamicsConstraint_ = mc_solver::DynamicsConstraint(robots(),
                                                       robots().robot("human").robotIndex(),
                                                       solver().dt(), {0.1, 0.01, 0.5});
    solver().addConstraintSet(humanDynamicsConstraint_);

    // Human model start posture
    if(config_("human").has("posture")){
      if(config_("human")("posture").has("target")){
        std::map<std::string, std::vector<double>> humanPostureTarget = config_("human")("posture")("target");
        // Set human mbc equal to the posture target
        for(auto const & t : humanPostureTarget){
          robots().robot("human").mbc().q[robots().robot("human").jointIndexByName(t.first)] = t.second;
        }
      }
    }
    // Adjust chair position relative to human model
    // if(robots().hasRobot("chair")){
    //   robots().robot("chair").posW(robots().robot("human").posW() * sva::PTransformd(Eigen::Vector3d(0.05, 0.0, -0.65)));
    // }

  }
  // Update constraints and resets posture tasks
  mc_control::fsm::Controller::reset(reset_data);
}

void HelpUpController::computeStabilityRegion(bool save, int polIndex, std::string suffix)
{
  // std::cout << "I arrived in the Compute Stability region function" << std::endl;
  // contactSet_->showContactSet();
  // std::cout << "Started computation " << polIndex << std::endl;
  futureCompPoint_ = std::make_shared<ComputationPoint> (polIndex, contactSet_);
  futureCompPoint_->computeEquilibriumRegion();
  // futureCompPoint_->chebichevCenter(); // compute and store the chebichev center to avoid having glpk being called twice at the same time
  if (save)
    {
      futureCompPoint_->save("");
    }

  // std::cout << "Ended computation " << polIndex << std::endl;
}


// bool HelpUpController::addTasksToSolver()
// {
//   solver().addTask(comTask_);
//   desiredCoM(robot().com());
//   solver().addConstraintSet(*comIncPlaneConstraintPtr_);
//   planes_ = {};
//   planes(planes_);

//   postureTask->stiffness(0.1);
  
//   return true;
// }

// bool HelpUpController::removeTasksFromSolver()
// {
//   solver().removeTask(comTask_);
//   return true;
// }


void HelpUpController::addLogEntries()
{
  logger().addLogEntry("polytopeIndex", [this] () -> const int { return polytopeIndex_; });
  logger().addLogEntry("polytope_computationTime", [this]() -> const int { return currentCompPoint_->computationTime();});

  // // Logging the desired CoM computed
  // auto desiredCoM = [this](){
  //   return comDesired_;
  // };
  // logger().addLogEntry("com_desired", desiredCoM);

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

void HelpUpController::planes(std::vector<mc_rbdyn::Plane> constrPlanes)
{
  planes_.clear();
  planes_ = constrPlanes;
  // mc_rtc::log::success("[BoxPusher] The set planes function has been called!");
  comIncPlaneConstraintPtr_->set_planes(solver(), planes_, {}, {}, 0.1, 0.03, 0.6, 0.0);
}

void HelpUpController::planes(std::vector<Eigen::Vector4d> constrPlanes)
{
  //LOG_INFO("Setting the comIncPlaneConstraint planes"
  std::vector<mc_rbdyn::Plane> p;

  for (auto plane: constrPlanes)
    {
      p.push_back({{-plane(0), -plane(1), -plane(2)}, plane(3)});
    }
  planes(p);
  // planesUpdated_ = true;
}

const std::vector<mc_rbdyn::Plane> HelpUpController::planes() const
{
  return planes_;
}

// void HelpUpController::desiredCoM(Eigen::Vector3d desiredCoM)
// {
//   auto prevCoM = comDesired_;
//   double coef = 0.1;
//   comDesired_ = (1-coef)*prevCoM + coef * desiredCoM;
//   comTask_->com(comDesired_);
// }

Eigen::Vector3d HelpUpController::currentCoM() const
{
  return realRobot().com();
}

// void HelpUpController::targetCoM(const Eigen::Vector3d & com, const Eigen::Vector3d & comp , const Eigen::Vector3d & compp)
// {
//   com_t_ = com;
//   comp_t_ = comp;
//   compp_t_ = compp;
// }

void HelpUpController::increasePolytopeIndex()
{
  polytopeIndex_++;
}

int HelpUpController::getPolytopeIndex()
{
  return polytopeIndex_;
}

void HelpUpController::updateContactSet(unsigned int robotIndex)
{
  auto contacts = solver().contacts();
  updateContactSet(contacts, robotIndex);
}

void HelpUpController::updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex)
{
  
  auto maxForces = config_("surfacesMaxForces");

  contactSet_ = std::make_shared<ContactSet> (false);
  contactSet_->mass(robot().mass());
  contactSet_->setFrictionSides(6);
  const auto & robot = robots().robot(robotIndex);

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
            // mc_rtc::log::warning( "Surface " + surface_name + " NOT found in the config for max force");
            fmax = robot.mass()*10;
          }

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
  Eigen::Vector3d acceleration;

  acceleration << 0.0, 0.0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0.4, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, 0.3, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << -0.4, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, -0.3, -9.81;
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

void HelpUpController::setFutureToCurrent()
{
  currentCompPoint_ = futureCompPoint_;
  planes(currentCompPoint_->constraintPlanes());
  // desiredCoM(currentCompPoint_->objectiveCoM(2, robot().com()));
}

void HelpUpController::setFutureToNext()
{
  nextCompPoint_ = futureCompPoint_;
}

void HelpUpController::setNextToCurrent()
{
  currentCompPoint_ = nextCompPoint_;
  planes(currentCompPoint_->constraintPlanes());
  // desiredCoM(currentCompPoint_->objectiveCoM(2, robot().com()));
}

 // auto currentCoM = robot().com();
 //  double currentZ = currentCoM(2);
 //  desiredCoM_(2)=currentZ;

 //  // set the planes and the desired CoM in the controler
 //  planes(desiredPlanes_);
 //  desiredCoM(desiredCoM_);

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