/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FirstOrderAdmittance.h"

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_filter/utils/clamp.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

namespace force
{

using mc_filter::utils::clampInPlaceAndWarn;

FirstOrderAdmittance::FirstOrderAdmittance(const std::string & surfaceName,
                                           const mc_rbdyn::Robots & robots,
                                           unsigned int robotIndex,
                                           double stiffness,
                                           double weight)
: FirstOrderAdmittance(robots.robot(robotIndex).frame(surfaceName), stiffness, weight)
{
}

FirstOrderAdmittance::FirstOrderAdmittance(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: AdmittanceTask(frame, stiffness, weight)
{
  name_ = "FirstOrderAdmi_" + frame_->robot().name() + "_" + frame_->name();
  reset();
}

void FirstOrderAdmittance::update(mc_solver::QPSolver &)
{
  /* Update function of original admittance:
  // Compute wrench error
  wrenchError_ = measuredWrench() - targetWrench_;

  // Compute linear and angular velocity based on wrench error and admittance
  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());

  // Clamp both values in order to have a 'security'
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");

  // Filter
  refVelB_ = velFilterGain_ * refVelB_ + (1 - velFilterGain_) * sva::MotionVecd(angularVel, linearVel);

  // Compute position and rotation delta
  sva::PTransformd delta(mc_rbdyn::rpyToMat(timestep_ * refVelB_.angular()), timestep_ * refVelB_.linear());

  // Acceleration
  TransformTask::refAccel((refVelB_ + feedforwardVelB_ - TransformTask::refVelB()) / timestep_);

  // Velocity
  TransformTask::refVelB(refVelB_ + feedforwardVelB_);

  // Position
  target(delta * target());
  */

  /* update function of damping:
  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");
  refVelB_ = feedforwardVelB_ + sva::MotionVecd{angularVel, linearVel};

  // SC: we could do add an anti-windup strategy here, e.g. back-calculation.
  // Yet, keep in mind that our velocity bounds are artificial. Whenever
  // possible, the best is to set to gains so that they are not saturated.

  TransformTask::refVelB(refVelB_);
  */

  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");
  // feedforwardVelB_ is set by refVelB() of admittance task
  refVelB_ = feedforwardVelB_ + sva::MotionVecd{angularVel, linearVel};

  // Compute position and rotation delta
  sva::PTransformd delta(mc_rbdyn::rpyToMat(timestep_ * refVelB_.angular()), timestep_ * refVelB_.linear());

  // This is ref vel of the underlying transform, not of the admittance: this refVelB_ has both velocities
  TransformTask::refVelB(refVelB_);
  // Position
  target(delta * target());
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "FirstOrderAdmittance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto frame = [&]() -> std::string
      {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("FirstOrderAdmittanceLoader", "surface", "frame");
          return config("surface");
        }
        return config("frame");
      }();
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "FirstOrderAdmittance");
      auto t = std::make_shared<mc_tasks::force::FirstOrderAdmittance>(solver.robots().robot(rIndex).frame(frame));
      t->reset();
      t->load(solver, config);
      return t;
    });
}
