// This is the ForceConstrainedTransformTask as written in https://github.com/isri-aist/mc-fc-transform-task/

#pragma once

#include <mc_tasks/TransformTask.h>

#include <eigen-quadprog/QuadProg.h>

#if defined _WIN32 || defined __CYGWIN__
#  define ForceConstrainedTransformTask_DLLIMPORT __declspec(dllimport)
#  define ForceConstrainedTransformTask_DLLEXPORT __declspec(dllexport)
#  define ForceConstrainedTransformTask_DLLLOCAL
#else
#  if __GNUC__ >= 4
#    define ForceConstrainedTransformTask_DLLIMPORT __attribute__((visibility("default")))
#    define ForceConstrainedTransformTask_DLLEXPORT __attribute__((visibility("default")))
#    define ForceConstrainedTransformTask_DLLLOCAL __attribute__((visibility("hidden")))
#  else
#    define ForceConstrainedTransformTask_DLLIMPORT
#    define ForceConstrainedTransformTask_DLLEXPORT
#    define ForceConstrainedTransformTask_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef ForceConstrainedTransformTask_STATIC
#  define ForceConstrainedTransformTask_DLLAPI
#  define ForceConstrainedTransformTask_LOCAL
#else
#  ifdef ForceConstrainedTransformTask_EXPORTS
#    define ForceConstrainedTransformTask_DLLAPI ForceConstrainedTransformTask_DLLEXPORT
#  else
#    define ForceConstrainedTransformTask_DLLAPI ForceConstrainedTransformTask_DLLIMPORT
#  endif // ForceConstrainedTransformTask_EXPORTS
#  define ForceConstrainedTransformTask_LOCAL ForceConstrainedTransformTask_DLLLOCAL
#endif // ForceConstrainedTransformTask_STATIC

namespace mc_tasks
{

struct ForceConstrainedTransformTask_DLLAPI ForceConstrainedTransformTask : public TransformTask
{
  /*! \brief Constructor
   *
   * \param frame Frame controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  ForceConstrainedTransformTask(const mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500.0);

  /** Add a constraint expressed in the world frame using the default damping and admittance */
  inline size_t addWorldConstraint(const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin)
  {
    return addWorldConstraint(dof, wrench, margin, default_safety_damping_, default_admittance_gain_);
  }
  /** Add a constraint expressed in the world frame using the default damping and admittance */
  inline size_t addWorldConstraint(const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin)
  {
    return addWorldConstraint(dof, wrench, margin, default_safety_damping_, default_admittance_gain_);
  }
  /** Add a constraint expressed in the world frame using scalar damping and admittance */
  inline size_t addWorldConstraint(const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin,
                                   double damping,
                                   double admittance)
  {
    return addWorldConstraint(dof, wrench, margin, Eigen::VectorXd::Constant(wrench.rows(), damping),
                              Eigen::VectorXd::Constant(wrench.rows(), admittance));
  }
  /** Add a constraint expressed in the world frame using scalar damping and admittance */
  inline size_t addWorldConstraint(const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin,
                                   double damping,
                                   double admittance)
  {
    return addWorldConstraint(dof, wrench, margin, sva::AdmittanceVecd(Eigen::Vector6d::Constant(damping)),
                              sva::AdmittanceVecd(Eigen::Vector6d::Constant(admittance)));
  }
  /** Add a constraint expressed in the world frame using scalar gains */
  inline size_t addWorldConstraint(const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin,
                                   const Eigen::VectorXd & damping,
                                   const Eigen::VectorXd & admittance)
  {
    return addFrameConstraint(nullptr, dofToSelector(dof), wrench, margin, damping, admittance);
  }
  /** Add a constraint expressed in the world frame using scalar gains */
  inline size_t addWorldConstraint(const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin,
                                   const sva::AdmittanceVecd & damping,
                                   const sva::AdmittanceVecd & admittance)
  {
    auto selector = dofToSelector(dof);
    return addFrameConstraint(nullptr, selector, wrench.vector(), margin.vector(), damping.vector(),
                              admittance.vector());
  }

  /** Add a constraint expressed in the control frame using the default damping and admittance */
  inline size_t addLocalConstraint(const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin)
  {
    return addLocalConstraint(dof, wrench, margin, default_safety_damping_, default_admittance_gain_);
  }
  /** Add a constraint expressed in the control frame using the default damping and admittance */
  inline size_t addLocalConstraint(const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin)
  {
    return addLocalConstraint(dof, wrench, margin, default_safety_damping_, default_admittance_gain_);
  }
  /** Add a constraint expressed in the control frame using scalar damping and admittance */
  inline size_t addLocalConstraint(const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin,
                                   double damping,
                                   double admittance)
  {
    return addLocalConstraint(dof, wrench, margin, Eigen::VectorXd::Constant(wrench.rows(), damping),
                              Eigen::VectorXd::Constant(wrench.rows(), admittance));
  }
  /** Add a constraint expressed in the control frame using scalar damping and admittance */
  inline size_t addLocalConstraint(const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin,
                                   double damping,
                                   double admittance)
  {
    return addLocalConstraint(dof, wrench, margin, sva::AdmittanceVecd(Eigen::Vector6d::Constant(damping)),
                              sva::AdmittanceVecd(Eigen::Vector6d::Constant(admittance)));
  }
  /** Add a constraint expressed in the control frame using scalar gains */
  inline size_t addLocalConstraint(const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin,
                                   const Eigen::VectorXd & damping,
                                   const Eigen::VectorXd & admittance)
  {
    return addFrameConstraint(frame(), dof, wrench, margin, damping, admittance);
  }
  /** Add a constraint expressed in the control frame using scalar gains */
  inline size_t addLocalConstraint(const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin,
                                   const sva::AdmittanceVecd & damping,
                                   const sva::AdmittanceVecd & admittance)
  {
    return addFrameConstraint(frame(), dof, wrench, margin, damping, admittance);
  }

  /** Add a constraint expressed in an arbitrary frame using the default damping and admittance */
  inline size_t addFrameConstraint(const mc_rbdyn::Frame & frame,
                                   const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin)
  {
    return addFrameConstraint(frame, dof, wrench, margin, default_safety_damping_, default_admittance_gain_);
  }
  /** Add a constraint expressed in an arbitrary frame using the default damping and admittance */
  inline size_t addFrameConstraint(const mc_rbdyn::Frame & frame,
                                   const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin)
  {
    return addFrameConstraint(frame, dof, wrench, margin, default_safety_damping_, default_admittance_gain_);
  }
  /** Add a constraint expressed in an arbitrary frame using scalar damping and admittance */
  inline size_t addFrameConstraint(const mc_rbdyn::Frame & frame,
                                   const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin,
                                   double damping,
                                   double admittance)
  {
    return addFrameConstraint(frame, dof, wrench, margin, Eigen::VectorXd::Constant(wrench.rows(), damping),
                              Eigen::VectorXd::Constant(wrench.rows(), admittance));
  }
  /** Add a constraint expressed in an arbitrary frame using scalar damping and admittance */
  inline size_t addFrameConstraint(const mc_rbdyn::Frame & frame,
                                   const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin,
                                   double damping,
                                   double admittance)
  {
    return addFrameConstraint(frame, dof, wrench, margin, sva::AdmittanceVecd(Eigen::Vector6d::Constant(damping)),
                              sva::AdmittanceVecd(Eigen::Vector6d::Constant(admittance)));
  }
  /** Add a constraint expressed in an arbitrary frame using scalar gains */
  inline size_t addFrameConstraint(const mc_rbdyn::Frame & frame,
                                   const Eigen::Vector6d & dof,
                                   const Eigen::VectorXd & wrench,
                                   const Eigen::VectorXd & margin,
                                   const Eigen::VectorXd & damping,
                                   const Eigen::VectorXd & admittance)
  {
    return addFrameConstraint(frame.shared_from_this(), dofToSelector(dof), wrench, margin, damping, admittance);
  }
  /** Add a constraint expressed in an arbitrary frame using scalar gains */
  inline size_t addFrameConstraint(const mc_rbdyn::Frame & frame,
                                   const Eigen::Vector6d & dof,
                                   const sva::ForceVecd & wrench,
                                   const sva::ForceVecd & margin,
                                   const sva::AdmittanceVecd & damping,
                                   const sva::AdmittanceVecd & admittance)
  {
    auto selector = dofToSelector(dof);
    return addFrameConstraint(frame.shared_from_this(), selector, wrench.vector(), margin.vector(), damping.vector(),
                              admittance.vector());
  }

  void clearConstraints() noexcept;

  /** Remove a constraint given its id */
  void removeConstraint(size_t id);

  /** Returns the wrench limit for a given constraint */
  const Eigen::VectorXd & constraintWrench(size_t id) const;
  /** Returns the margin for a given constraint */
  const Eigen::VectorXd & constraintMargin(size_t id) const;
  /** Returns the damping gains for a given constraint */
  const Eigen::VectorXd & constraintDamping(size_t id) const;
  /** Returns the admittance gains for a given constraint */
  const Eigen::VectorXd & constraintAdmittance(size_t id) const;

  /** Set the wrench limit for a given constraint */
  void constraintWrench(size_t id, const Eigen::VectorXd & value);
  /** Set the margin for a given constraint */
  void constraintMargin(size_t id, const Eigen::VectorXd & value);
  /** Set the damping gains for a given constraint */
  void constraintDamping(size_t id, const Eigen::VectorXd & value);
  /** Set the admittance gains for a given constraint */
  void constraintAdmittance(size_t id, const Eigen::VectorXd & value);

  /** Default safety damping used when \ref addConstraint is called without specific values */
  inline double defaultSafetyDamping() const noexcept
  {
    return default_safety_damping_;
  }

  /** Set the default safety damping
   *
   * It will only affect constraints that are added after this is changed
   */
  inline void defaultSafetyDamping(double sd) noexcept
  {
    default_safety_damping_ = sd;
  }

  /** Default admittance gain used when \ref addConstraint is called without specific values */
  inline double defaultAdmittanceGain() const noexcept
  {
    return default_admittance_gain_;
  }

  /* Set the default admittance gain
   *
   * It will only affect constraints that are added after this is changed
   */
  inline void defaultAdmittanceGain(double ag) noexcept
  {
    default_admittance_gain_ = ag;
  }

  inline const Eigen::Vector6d & refAccel() const noexcept
  {
    return user_ref_accel_;
  }

  inline void refAccel(const Eigen::Vector6d & refAccel) noexcept
  {
    user_ref_accel_ = refAccel;
  }

  void update(mc_solver::QPSolver & solver) override;

  void addToLogger(mc_rtc::Logger & logger) override;

protected:
  // User parameters
  double default_safety_damping_ = 0;
  double default_admittance_gain_ = 0;

  struct ConstraintDescription
  {
  public:
    size_t id;
    mc_rbdyn::ConstFramePtr frame;
    Eigen::MatrixXd selector;
    Eigen::VectorXd active;
    Eigen::MatrixXd H;
    Eigen::MatrixXd H_dual;
    Eigen::VectorXd wrench;
    Eigen::VectorXd margin;
    Eigen::VectorXd damping;
    Eigen::VectorXd admittance;
  };

  // Next constraint id
  size_t next_id = 0;

  // Constraints
  std::vector<ConstraintDescription> constraints_;

  // Ref accel provided by the user
  Eigen::Vector6d user_ref_accel_ = Eigen::Vector6d::Zero();

  // Robot Frame Accel in Frame Coordinate
  Eigen::Vector6d frame_accB_ = Eigen::Vector6d::Zero();

  // Intermediary reference acceleration computed as-if we had no bounds
  Eigen::Vector6d original_ref_accel_;

  // Optimized acceleration
  Eigen::Vector6d optimized_accel_;

  // Number of active constraints this run
  size_t n_cstr_ = 0;

  // Optimization problem
  Eigen::QuadProgDense qp_;
  // Cost matrix (identity)
  Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(6, 6);
  // Cost vector
  Eigen::VectorXd C_ = Eigen::VectorXd::Zero(6);
  // Equality matrix and vector are always zero-sized
  Eigen::MatrixXd Aeq_ = Eigen::MatrixXd::Zero(0, 0);
  Eigen::VectorXd Beq_ = Eigen::VectorXd::Zero(0);
  // Inequality matrix
  Eigen::MatrixXd Aineq_ = Eigen::MatrixXd::Zero(0, 0);
  // Inequality vector
  Eigen::VectorXd Bineq_ = Eigen::VectorXd::Zero(0);

  // For internal usage
  static Eigen::MatrixXd dofToSelector(const Eigen::Vector6d & dof);

  // Internal usage
  size_t addFrameConstraint(mc_rbdyn::ConstFramePtr frame,
                            const Eigen::MatrixXd & selector,
                            const Eigen::VectorXd & wrench,
                            const Eigen::VectorXd & margin,
                            const Eigen::VectorXd & damping,
                            const Eigen::VectorXd & admittance);

  std::vector<ConstraintDescription>::iterator getConstraint(size_t id);

  std::vector<ConstraintDescription>::const_iterator getConstraint(size_t id) const;
};

} // namespace mc_tasks
