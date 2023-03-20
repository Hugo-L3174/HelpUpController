#include "XsensHuman.h"

#include "../HelpUpController.h"

void XsensHuman::configure(const mc_rtc::Configuration & config)
{
  config("stiffness", stiffness_);
  config("robot", robot_);
  config("offset", offset_);
}

void XsensHuman::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  if(robot_.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] \"robot\" parameter required");
  }
  else if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named \"{}\"");
  }
  auto & robot = ctl.robot(robot_);
  if(!ctl.config()("Xsens").has(robot.name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot {} not supported (missing Xsens->{} configuration)", robot.name(), name(), robot.name());
  }
  if(!ctl.datastore().has("XsensPlugin"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] This state requires the XsensPlugin", name());
  }
  
  auto & grounding_offset = ctl.datastore().make<sva::PTransformd>("XsensHuman::GetGroundOffset");

  auto robotConfig = static_cast<std::map<std::string, mc_rtc::Configuration>>(ctl.config()("Xsens")(robot.name()));
  for(const auto & bodyConfig : robotConfig)
  {
    const auto & bodyName = bodyConfig.first;
    const auto & bodyConf = bodyConfig.second;
    bodyConfigurations_[bodyName] = XsensBodyConfiguration{};
    auto & bodyC = bodyConfigurations_[bodyName];
    bodyC.bodyName = bodyName;
    bodyC.segmentName = static_cast<std::string>(bodyConf("segment"));
    bodyConf("offset", bodyC.offset);
    ctl.gui()->addElement({"Xsens", robot_, "Bodies", bodyName},
      mc_rtc::gui::ArrayInput("Offset translation [m]",
      [&bodyC]()
      {
        return bodyC.offset.translation();
      },
      [&bodyC](const Eigen::Vector3d & offset)
      {
        bodyC.offset.translation() = offset;
      }),
      mc_rtc::gui::ArrayInput("Offset rotation [rad]",
      [&bodyC]()
      {
        return mc_rbdyn::rpyFromMat(bodyC.offset.rotation());
      },
      [&bodyC](const Eigen::Vector3d & offset)
      {
        bodyC.offset.rotation() = mc_rbdyn::rpyToMat(offset);
      })
    );
  }

  // init filters
  gram_sg::SavitzkyGolayFilterConfig velfilterconf_(10,10,2,0,ctl.timeStep);
  velocityFilter_ = std::make_shared<gram_sg::MotionVecdFilter>(velfilterconf_);
 

  // gram_sg::SavitzkyGolayFilterConfig accfilterconf_(50,50,2,1,ctl.timeStep);
  // accFilter_ = std::make_shared<gram_sg::SavitzkyGolayFilter>(accfilterconf_);

  // Initialize tasks
  for(auto & body: bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second.bodyName;
    if(robot.hasBody(bodyName))
    {
      auto task = std::unique_ptr<mc_tasks::TransformTask>(new mc_tasks::TransformTask(bodyName, ctl.robots(), robot.robotIndex(), stiffness_, weight_));
      task->reset();
      ctl.solver().addTask(task.get());
      tasks_[bodyName] = std::move(task); 
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", bodyName);
    }
  }
  run(ctl);
}

bool XsensHuman::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  auto & robot = ctl.robot(robot_);

  auto & realRobot = ctl.realRobot(robot_);
  auto & grounding_offset = ctl.datastore().get<sva::PTransformd>("XsensHuman::GetGroundOffset");

  // getting transform from foot to ground (to apply everywhere)
  auto X_ground_foot = realRobot.surfacePose("RightSole").inv();
  X_ground_foot.translation().x() = 0;
  X_ground_foot.translation().y() = 0;
  X_ground_foot.translation().z() *= 3; // *3 otherwise is too small
  // writing offset in datastore, to be applied by XsensPlugin (applying on positions read by mujoco and others)
  grounding_offset = X_ground_foot;
  

  for(const auto & body: bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second.segmentName;
    if(robot.hasBody(bodyName))
    {
      try
      {
        const auto segmentPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName); 
        auto poseTarget = body.second.offset * segmentPose * offset_;
        tasks_[bodyName]->target(poseTarget);
  
        // // get velocity from current and previous poses
        // auto X_p1_p2 = poseTarget * body.second.prevBodyPose_.inv();
        // auto vel = 1/ctl.timeStep * sva::transformVelocity(X_p1_p2);

        // // Add velocity to the filter
        // velocityFilter_->add(vel);
        // sva::MotionVecd filteredVel = velocityFilter_->filter();
        // setting reference velocity
        // tasks_[bodyName]->refVelB(filteredVel);

        // getting ref acceleration from velocity
        // accFilter_.add();
        // tasks_[bodyName]->refAccel();
        
        bodyConfigurations_.at(bodyName).prevBodyPose_ = poseTarget;

        const auto CoMpos = ctl.datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMpos"); 
        const auto CoMvel = ctl.datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMvel"); 
        const auto CoMacc = ctl.datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMacc"); 
        ctl.setxsensCoM(CoMpos, CoMvel, CoMacc);
      }
      catch(...)
      {
        mc_rtc::log::error("[{}] No pose for segment {}", name(), segmentName);
      }
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", name(), bodyName);
    }
  }
  output("OK");
  return true;
}

void XsensHuman::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HelpUpController &>(ctl_);
  for(const auto & task : tasks_)
  {
    ctl.solver().removeTask(task.second.get());
  }
}

EXPORT_SINGLE_STATE("XsensHuman", XsensHuman)
