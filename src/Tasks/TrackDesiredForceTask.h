#include <mc_solver/utils/ContactWrenchMatrixToLambdaMatrix.h>
#include <mc_solver/utils/UpdateNrVars.h>
#include <Tasks/QPTasks.h>

/** Implementation of the tracking task for the Tasks library
 *
 * - tasks::qp::Task is the interface in Tasks
 * - mc_solver::utils::UpdateNrVarsLambda provides us the implementation for updateNrVars which tracks where in the
 *   global cost matrix this should be inserted
 */
struct TrackDesiredForceTask : public tasks::qp::Task, mc_solver::utils::UpdateNrVarsLambda
{
  inline TrackDesiredForceTask(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid, double weight = 1)
  : tasks::qp::Task(weight), mc_solver::utils::UpdateNrVarsLambda(cid), wrenchToLambda_(solver, cid)
  {
    Q_ = wrenchToLambda_.transform().transpose() * wrenchToLambda_.transform();
  }

  /** Returns the position in the global cost matrix
   *
   * ABegin_ is updated in the updateNrVars call
   */
  inline std::pair<int, int> begin() const override
  {
    return {ABegin_, ABegin_};
  }

  /** Called by Tasks when the problem structure changes */
  inline void updateNrVars(const std::vector<rbd::MultiBody> & mbs, const tasks::qp::SolverData & data) override
  {
    mc_solver::utils::UpdateNrVarsLambda::updateNrVarsImpl(mbs, data);
  }

  inline void update(const std::vector<rbd::MultiBody> & mbs,
                     const std::vector<rbd::MultiBodyConfig> & mbcs,
                     const tasks::qp::SolverData & data) override
  {
  }

  /** Returns the cost matrix */
  inline const Eigen::MatrixXd & Q() const override
  {
    return Q_;
  }

  /** Returns the cost vector */
  inline const Eigen::VectorXd & C() const override
  {
    return C_;
  }

  inline void setTargetWrench(const sva::ForceVecd & w)
  {
    C_ = -wrenchToLambda_.transform().transpose() * w.vector();
  }

private:
  mc_solver::utils::ContactWrenchMatrixToLambdaMatrix wrenchToLambda_;
  Eigen::MatrixXd Q_;
  Eigen::VectorXd C_ = Eigen::Vector6d::Zero();
};
