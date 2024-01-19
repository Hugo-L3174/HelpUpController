// This is the ForceConstrainedTransformTask as written in https://github.com/isri-aist/mc-fc-transform-task/
#include "ForceConstrainedTransformTask.h"

#include <mc_solver/TasksQPSolver.h>

namespace mc_tasks
{

ForceConstrainedTransformTask::ForceConstrainedTransformTask(const mc_rbdyn::RobotFrame & frame,
                                                             double stiffness,
                                                             double weight)
: TransformTask(frame, stiffness, weight)
{
  if(mc_solver::QPSolver::context_backend() != mc_solver::QPSolver::Backend::Tasks)
  {
    mc_rtc::log::error_and_throw("[ForceConstrainedTransformTask] Only implemented for the Tasks backend");
  }
}

void ForceConstrainedTransformTask::clearConstraints() noexcept
{
  constraints_.clear();
  Q_ = Eigen::MatrixXd::Identity(6, 6);
  C_ = Eigen::VectorXd::Zero(6);
  Aineq_ = Eigen::MatrixXd::Zero(0, 0);
  Bineq_ = Eigen::VectorXd::Zero(0);
}

Eigen::MatrixXd ForceConstrainedTransformTask::dofToSelector(const Eigen::Vector6d & dof)
{
  Eigen::Matrix6d selector_6d = Eigen::Matrix6d::Identity();
  Eigen::DenseIndex active_line = 0;
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    if(dof(i) != 0)
    {
      selector_6d.row(active_line) = dof(i) * selector_6d.row(i);
      active_line += 1;
    }
  }
  if(active_line < 1)
  {
    mc_rtc::log::error_and_throw("[ForceConstrainedTransformTask] The dof selector must enable at least one dimension");
  }
  return selector_6d.topRows(active_line);
}

size_t ForceConstrainedTransformTask::addFrameConstraint(mc_rbdyn::ConstFramePtr frame,
                                                         const Eigen::MatrixXd & selector,
                                                         const Eigen::VectorXd & wrench,
                                                         const Eigen::VectorXd & margin,
                                                         const Eigen::VectorXd & safety_damping,
                                                         const Eigen::VectorXd & admittance_gain)
{
  // Sanity check
  assert(selector.rows() == wrench.rows());
  assert(wrench.rows() == margin.rows());
  assert(margin.rows() == safety_damping.rows());
  assert(safety_damping.rows() == admittance_gain.rows());
  // Add the constraint

  constraints_.push_back({next_id, frame, selector, Eigen::VectorXd::Zero(wrench.rows()), selector, selector, wrench,
                          margin, safety_damping, admittance_gain});
  // Resize the problem matrixes to their maximum possible size
  Q_ = Eigen::MatrixXd::Identity(Q_.rows() + selector.rows(), Q_.cols() + selector.rows());
  C_ = Eigen::VectorXd::Zero(Q_.rows());
  Aineq_ = Eigen::MatrixXd::Zero(Aineq_.rows() + selector.rows(), Q_.rows());
  Bineq_ = Eigen::VectorXd::Zero(Aineq_.rows());
  mc_rtc::log::info("[{}] adding constraint fo wrench\n{}\nwith admittance gain of\n{}\nwith selector\n{}", name(),
                    constraints_.back().wrench, constraints_.back().admittance, constraints_.back().selector);
  return next_id++;
}

void ForceConstrainedTransformTask::update(mc_solver::QPSolver & solver)
{
  // FIXME We want to override the constructor to abort early if the frame does not have a sensor attached
  sva::PTransformd X_frame_0 = frame().position().inv();

  // Get the measured wrench in control frame
  auto measured_wrench = frame().wrench().vector();

  // measured_wrench.force().x() *= -1;
  // auto measured_wrench_vec = measured_wrench.vector();
  auto errorT = static_cast<tasks::qp::SurfaceTransformTask *>(this->errorT.get());
  // FIXME This does not follow open/closed loop behavior
  errorT->update(solver.robots().mbs(), solver.robots().mbcs(), tasks_solver(solver).data());

  // Commented because this is unused and throws an std::out_of_range
  // frame_accB_ = (frame().X_b_f() * solver.robot().bodyAccB(frame().body())).vector();

  // We now check which constraints should be activated
  n_cstr_ = 0;
  for(auto & cstr : constraints_)
  {
    if(cstr.frame)
    {
      // H and H_dual represents the transformation from the control frame to the constained frame
      cstr.H = cstr.selector * (cstr.frame->position() * X_frame_0).matrix();
      cstr.H_dual = cstr.selector * (cstr.frame->position() * X_frame_0).dualMatrix();
    }
    Eigen::VectorXd cstr_wrench = cstr.H_dual * measured_wrench;
    // TODO: investigate values here
    for(Eigen::DenseIndex i = 0; i < cstr_wrench.rows(); ++i)
    {

      if(cstr_wrench(i) > (cstr.wrench + cstr.margin)(i))
      {
        if(cstr.active(i) == 0)
        {
          cstr.active(i) = 1;
          mc_rtc::log::info("[{}] Broken cstr over margin {} at axis {}", name(), cstr.id, i);
          mc_rtc::log::info("[{}] {} >= {}", name(), cstr_wrench(i), (cstr.wrench + cstr.margin)(i));
        }
        // Case (2): the wrench is above the margin
        // - H * \ddot(x) + v1 <=  D_s * H * \dot(x) - A * (H_dual * w_m - b_c) + \ddot(x_r)
        Aineq_.row(n_cstr_).head(6) = -cstr.H.row(i);
        Aineq_(n_cstr_, 6 + n_cstr_) = 1.0;
        Bineq_(n_cstr_) =
            cstr.damping(i) * (cstr.H * errorT->speed())(i)-cstr.admittance(i) * (cstr_wrench - cstr.wrench)(i);
        n_cstr_ += 1;
      }
      else if(cstr_wrench(i) >= cstr.wrench(i))
      {
        // Case (1): the wrench is above the wrench but below the margin
        // - H * \ddot(x) + v1 <=  D_s * H * \dot(x) + \ddot(x_r)
        Aineq_.row(n_cstr_).head(6) = -cstr.H.row(i);
        Aineq_(n_cstr_, 6 + n_cstr_) = 1.0;
        Bineq_(n_cstr_) = cstr.damping(i) * (cstr.H * errorT->speed())(i);
        n_cstr_ += 1;
      }
      else if(cstr.active(i) == 1)
      {
        cstr.active(i) = 0;
        mc_rtc::log::info("[{}] Released broken cstr {} at axis {}", name(), cstr.id, i);
      }
    }
  }

  // Note: in Tasks convention eval() is x_ref - x
  original_ref_accel_ =
      (dimStiffness().cwiseProduct(errorT->eval()) - dimDamping().cwiseProduct(errorT->speed() - refVel()))
      + user_ref_accel_;

  if(n_cstr_ == 0)
  {
    // No active constraint, just act like a regular TransformTask
    optimized_accel_.setZero();
    TransformTask::refAccel(user_ref_accel_);
    // FIXME We want to do this but update is private in TransformTask, practically it is not a problem because the
    // update does nothing but it's ugly :s
    // TransformTask::update(solver);
    return;
  }

  int nrvar = 6 + n_cstr_;
  qp_.problem(nrvar, 0, n_cstr_);
  // FIXME Allow setting slack_weight?
  double slack_weight = 1e6;
  Q_.block(6, 6, n_cstr_, n_cstr_).diagonal().setConstant(slack_weight);
  // Prepare C_
  C_.head(6) = -original_ref_accel_;
  bool qp_ok = qp_.solve(Q_.topLeftCorner(nrvar, nrvar), C_.head(nrvar), Aeq_, Beq_, Aineq_.topRows(n_cstr_),
                         Bineq_.head(n_cstr_));
  if(!qp_ok)
  {
    mc_rtc::log::error("[ForceConstrainedTransformTask] Internal QP failed");
    optimized_accel_.setZero();
    TransformTask::refAccel(user_ref_accel_);
    // FIXME See above
    // TransformTask::update(solver);
    return;
  }

  optimized_accel_ = qp_.result().head(6);
  // We compensate the computations done by Tasks such that the final desired acceleration is the one computed by our
  // optimization problem
  TransformTask::refAccel(sva::MotionVecd(optimized_accel_ - original_ref_accel_ + user_ref_accel_));
}

void ForceConstrainedTransformTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);
  logger.addLogEntries(
      this, name() + "_optimized_acceleration_frame",
      [this]() -> Eigen::Vector6d
      { return sva::PTransformd(frame().position().rotation()).matrix() * optimized_accel_; },
      name() + "_control_robot_acceleration_frame", [this]() -> Eigen::Vector6d { return frame_accB_; },
      name() + "_TransformTask_RefAccel_frame",
      [this]() -> Eigen::Vector6d
      {
        return sva::PTransformd(frame().position().rotation()).matrix()
               * (optimized_accel_ - original_ref_accel_ + user_ref_accel_);
      },
      name() + "_reference_acceleration_frame",
      [this]() -> Eigen::Vector6d
      { return sva::PTransformd(frame().position().rotation()).matrix() * original_ref_accel_; },
      name() + "_nr_constraints", [this]() { return n_cstr_; });
}

void ForceConstrainedTransformTask::removeConstraint(size_t id)
{
  auto it = getConstraint(id);
  if(it != constraints_.end())
  {
    constraints_.erase(it);
  }
}

auto ForceConstrainedTransformTask::getConstraint(size_t id) -> std::vector<ConstraintDescription>::iterator
{
  auto it = std::find_if(constraints_.begin(), constraints_.end(),
                         [&id](const ConstraintDescription & cs) { return cs.id == id; });
  if(it == constraints_.end())
  {
    mc_rtc::log::error_and_throw("[{}] No constraint with the given id: {}", name(), id);
  }
  return it;
}

auto ForceConstrainedTransformTask::getConstraint(size_t id) const -> std::vector<ConstraintDescription>::const_iterator
{
  auto it = std::find_if(constraints_.begin(), constraints_.end(),
                         [&id](const ConstraintDescription & cs) { return cs.id == id; });
  if(it == constraints_.end())
  {
    mc_rtc::log::error_and_throw("[{}] No constraint with the given id: {}", name(), id);
  }
  return it;
}

const Eigen::VectorXd & ForceConstrainedTransformTask::constraintWrench(size_t id) const
{
  auto it = getConstraint(id);
  return it->wrench;
}
const Eigen::VectorXd & ForceConstrainedTransformTask::constraintMargin(size_t id) const
{
  auto it = getConstraint(id);
  return it->margin;
}
const Eigen::VectorXd & ForceConstrainedTransformTask::constraintDamping(size_t id) const
{
  auto it = getConstraint(id);
  return it->damping;
}
const Eigen::VectorXd & ForceConstrainedTransformTask::constraintAdmittance(size_t id) const
{
  auto it = getConstraint(id);
  return it->admittance;
}

void ForceConstrainedTransformTask::constraintWrench(size_t id, const Eigen::VectorXd & value)
{
  auto it = getConstraint(id);
  assert(value.size() == it->wrench.size());
  it->wrench = value;
}
void ForceConstrainedTransformTask::constraintMargin(size_t id, const Eigen::VectorXd & value)
{
  auto it = getConstraint(id);
  assert(value.size() == it->margin.size());
  it->margin = value;
}
void ForceConstrainedTransformTask::constraintDamping(size_t id, const Eigen::VectorXd & value)
{
  auto it = getConstraint(id);
  assert(value.size() == it->damping.size());
  it->damping = value;
}
void ForceConstrainedTransformTask::constraintAdmittance(size_t id, const Eigen::VectorXd & value)
{
  auto it = getConstraint(id);
  assert(value.size() == it->admittance.size());
  it->admittance = value;
}

} // namespace mc_tasks
