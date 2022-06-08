#include "HelpUpController.h"

HelpUpController::HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // Load entire controller configuration file
  config_.load(config);

  // datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
  // return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
  // });
  // /* Observers
  //  */
  double leftFootRatio = 0.5;
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
			[this, &leftFootRatio]( const mc_rbdyn::Robot & robot)
			{
			  // return sva::interpolate(robot.surfacePose("LeftFoot"),
			  // 			  robot.surfacePose("RightFoot"),
			  // 			  leftFootRatio);
			  return robot.surfacePose("RightFoot");
			});


  mc_rtc::log::success("HelpUpController init done ");
}

bool HelpUpController::run()
{
  return mc_control::fsm::Controller::run();
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


