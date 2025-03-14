#include "XsensHuman.h"
#include <mc_tasks/TransformTask.h>

#include "../HelpUpController.h"
#include "utils/make_task.h"

void XsensHuman::configure(const mc_rtc::Configuration & config)
{
  config("stiffness", stiffness_);
  config("weight", weight_);
  config("robot", robot_);
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
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot {} not supported (missing Xsens->{} configuration)",
                                                     robot.name(), name(), robot.name());
  }
  if(!ctl.datastore().has("XsensPlugin"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] This state requires the XsensPlugin", name());
  }

  if(ctl.datastore().has("XsensMode"))
  {
    liveMode_ = ctl.datastore().get<bool>("XsensMode");
    if(!liveMode_)
    {
      auto plugins = ctl.config()("Plugins").operator std::vector<std::string>();
      if(std::find(plugins.begin(), plugins.end(), "Replay") != plugins.end())
      {
        if(!ctl.config().has("Replay"))
        {
          mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Missing Replay configuration", name());
        }
        mc_rtc::log::info("Running human state with logged Xsens mode.");
      }
      else
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[{}] Running the logged Xsens mode requires the Replay plugin", name());
      }
    }
  }
  else
  {
    mc_rtc::log::info("Running human state with live Xsens mode.");
  }

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
    ctl.gui()->addElement(
        {"Xsens", robot_, "Bodies", bodyName},
        mc_rtc::gui::ArrayInput(
            "Offset translation [m]", [&bodyC]() { return bodyC.offset.translation(); },
            [&bodyC](const Eigen::Vector3d & offset) { bodyC.offset.translation() = offset; }),
        mc_rtc::gui::ArrayInput(
            "Offset rotation [rad]", [&bodyC]() { return mc_rbdyn::rpyFromMat(bodyC.offset.rotation()); },
            [&bodyC](const Eigen::Vector3d & offset) { bodyC.offset.rotation() = mc_rbdyn::rpyToMat(offset); }));
  }

  // init filters
  // gram_sg::SavitzkyGolayFilterConfig velfilterconf_(10,10,2,0,ctl.timeStep);
  // velocityFilter_ = std::make_shared<gram_sg::MotionVecdFilter>(velfilterconf_);

  // gram_sg::SavitzkyGolayFilterConfig accfilterconf_(50,50,2,1,ctl.timeStep);
  // accFilter_ = std::make_shared<gram_sg::SavitzkyGolayFilter>(accfilterconf_);

  // Initialize tasks
  for(auto & body : bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    if(robot.hasBody(bodyName))
    {
      auto task = make_task_optional_gui<mc_tasks::TransformTask, false, false>(ctl.robot().frame(bodyName), stiffness_,
                                                                                weight_);
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
  // Should this be commented?
  auto & grounding_offset = ctl.datastore().get<sva::PTransformd>("XsensHuman::GroundOffset");

  // // getting transform from foot to ground (to apply everywhere)
  auto X_foot_0 =
      sva::interpolate(robot.surfacePose("LeftSole_ForceShoe"), robot.surfacePose("RightSole_ForceShoe"), 0.5).inv();
  X_foot_0.translation().x() = 0; // (we only want the vertical offset)
  X_foot_0.translation().y() = 0;
  // writing offset in datastore, to be applied by XsensPlugin (applying on positions read by mujoco and others)
  grounding_offset = X_foot_0;

  // // Debug:
  // mc_rtc::log::info("pose is {}", X_foot_0.inv().translation().z());
  // mc_rtc::log::info("offset to apply is {}", X_foot_0.translation().z());
  // mc_rtc::log::info("entries are: ");
  // for (const auto & entry:ctl.datastore().keys())
  // {
  //   mc_rtc::log::info("{}", entry);
  // }

  // Checking that xsens plugin is advertising data
  if(ctl.datastore().has("XsensPlugin"))
  {
    try
    {
      const auto CoMpos = ctl.datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMpos");
      const auto CoMvel = ctl.datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMvel");
      const auto CoMacc = ctl.datastore().call<Eigen::Vector3d>("XsensPlugin::GetCoMacc");
      ctl.setxsensCoM(CoMpos, CoMvel, CoMacc);
    }
    catch(...)
    {
      mc_rtc::log::error("No datastore value for human CoM");
    }

    for(const auto & body : bodyConfigurations_)
    {
      const auto & bodyName = body.first;
      const auto & segmentName = body.second.segmentName;
      if(robot.hasBody(bodyName))
      {
        try
        {
          const auto segmentPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName);
          // Offset computations here if processing of log (livemode_ = false), otherwise done in xsensplugin
          auto targetPose = sva::PTransformd::Identity();
          targetPose = segmentPose;
          if(!liveMode_)
          {
            targetPose = segmentPose * grounding_offset;
            if(bodyName.compare("RAnkleLink") == 0 || bodyName.compare("LAnkleLink") == 0)
            {
              auto footTarget = targetPose;
              auto zRot = footTarget.rotation().bottomRightCorner<1, 1>();
              targetPose.rotation() = sva::PTransformd::Identity().rotation();
              targetPose.rotation().diagonal() << 1., 1., zRot;
            }
          }

          tasks_[bodyName]->target(targetPose);
        }
        catch(...)
        {
          mc_rtc::log::error("[{}] No pose for segment {}", name(), segmentName);
        }

        try
        {
          const auto segmentVel = ctl.datastore().call<sva::MotionVecd>("XsensPlugin::GetSegmentVel", segmentName);
          tasks_[bodyName]->refVelB(segmentVel);
        }
        catch(...)
        {
          mc_rtc::log::error("[{}] No velocity for segment {}", name(), segmentName);
        }

        try
        {
          const auto segmentAcc = ctl.datastore().call<sva::MotionVecd>("XsensPlugin::GetSegmentAcc", segmentName);
          tasks_[bodyName]->refAccel(segmentAcc);
        }
        catch(...)
        {
          mc_rtc::log::error("[{}] No acceleration for segment {}", name(), segmentName);
        }
      }
      else
      {
        mc_rtc::log::error("[{}] No body named {}", name(), bodyName);
      }
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
