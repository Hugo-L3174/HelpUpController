#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_filter/LeakyIntegrator.h>
#include <mc_filter/LowPass.h>
#include "MCStabilityPolytope.h"
#include "api.h"
#include "problemDescriptor/contactPoints.h" // for ContactType enum
#include "problemDescriptor/contactSet.h"
#include <eigen-quadprog/QuadProg.h>
#include <gram_savitzky_golay/gram_savitzky_golay.h>

#include "utils/DCM_VRPtracker.h"

struct MCStabilityPolytope;
namespace mc_tasks
{
struct TransformTask;
namespace lipm_stabilizer
{
struct StabilizerTask;
namespace internal
{
struct Contact;
}
} // namespace lipm_stabilizer
} // namespace mc_tasks

namespace mc_solver
{
struct CoMIncPlaneConstr;
}

namespace mc_control
{
struct SimulationContactPair;
}

enum whatRobot
{
  mainRob,
  human,
  combined
};

enum HelpUpControllerContactType
{
  RH, // right hand
  LH, // left hand
  RF, // right foot
  LF // left foot
};

struct HelpUpController_DLLAPI HelpUpController : public mc_control::fsm::Controller
{
  HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  /*! Use to delay the start of the equilibrium criterion
   */
  bool addTasksToSolver();

  /*! \brief function called in the constructor to set up the log entries
   */
  void addLogEntries();

  /*! \brief function called in the constructor to set up the GUI elements
   */
  void addGuiElements();

  /*! \brief Seta the planes for the CoMIncPlaneConstr
   */
  void planes(const std::vector<mc_rbdyn::Plane> & constrPlanes, whatRobot rob);

  /*! \brief Set the planes for the CoMIncPlaneConstr
   */
  void planes(const std::vector<Eigen::Vector4d> & constrPlanes, whatRobot rob);

  // /*! \brief Gives the planes currently used bu CoMIncPlaneConstr
  //  */
  // const std::vector<mc_rbdyn::Plane> planes() const;

  /*! \brief Set the CoM target for CoM task
   */
  void desiredCoM(Eigen::Vector3d desiredCoM, whatRobot rob);

  /*! \brief Update the contactSet with the current contacts of the rbdyn robot object
   */
  void updateContactSet(unsigned int robotIndex);

  /*! \brief Update the contactSet with the given contacts
   */
  // void updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex, whatRobot rob);
  void updateContactSet(const std::vector<mc_rbdyn::Contact> & contacts, unsigned int robotIndex);

  /*! \brief Update the normal contact force upper and lower bound in the contactSetObject
   * \param contactFMax map of the names of the contacts to update with the corresponding upper bounds
   * \param contactFMin map of the names of the contacts to update with the corresponding lower bounds
   */
  void contactForces(const std::map<std::string, double> & contactFMax,
                     const std::map<std::string, double> & contactFMin);

  void updateContactForces();

  void updateCombinedCoM();

  /*! \brief Utility function to check if a vertex is inside a set of planes
   * \param Vertex vertex to test
   * \param planes planes to test with
   * \param eps minimum distance from a plane for the vertex to be considered inside
   */
  bool isVertexInPlanes(const Eigen::Vector3d & Vertex, const std::vector<Eigen::Vector4d> & planes, double eps = 0.01);

  std::map<std::string, double> getConfigFMax() const;
  std::map<std::string, double> getConfigFMin() const;

  std::optional<double> override_CoMz;

  // Checking if distance between real human feet/ground - butt/chair - (robot hands/back and shoulder? maybe robotside)
  // is low enough to considenr a contact
  void updateRealHumContacts();

  void addRealHumContact(std::string humanSurfName, double fmin, double fmax, ContactType type);

  void setxsensCoM(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc)
  {
    xsensCoMpos_ = pos;
    xsensCoMvel_ = vel;
    xsensCoMacc_ = acc;
  };

  // TODO: maybe find a cleaner way than hard writing which surface is right and left
  /*! \brief Distribute missing computed wrench on the desired surfaces
   * \param desiredWrench missing 6d wrench at CoM
   * \param targetRobot "robot" to assist (can be human model)
   * \param leftSurface surface on which force can be applied by robot left hand
   * \param rightSurface surface on which force can be applied by robot right hand
   */
  void distributeHandsWrench(const sva::ForceVecd & desiredWrench,
                             const mc_rbdyn::Robot & targetRobot,
                             const std::string & leftSurface,
                             const std::string & rightSurface);

  // Parametrized start offset of the log to sychronize. Default: start offset at 0, acquisition frequency of force
  // shoes at 100Hz
  sva::ForceVecd getCurrentForceVec(const std::vector<sva::ForceVecd> & log, double startOffset = 0, double freq = 100);

  sva::ForceVecd getLHWrenchComputed()
  {
    return LHwrench_;
  }

  sva::ForceVecd getRHWrenchComputed()
  {
    return RHwrench_;
  }

  // This should: compute the polytope with the current contact set, update the DCM objective and give it to the
  // stabilizer task associated to the robot
  void computePolytope(const Eigen::Vector3d & currentPos, bool & firstPolyOK_, whatRobot rob);

  // Todo: add options in both functions to choose if we check CoM or DCM, wether to update the objective or the robot
  // stabilizer via datastore, and which update contacts function to use (human is real contacts + hard written surfaces)

  // Update the objective: if the currentPos is in the balance polytope, then objective stays current CoM/DCM, otherwise
  // closest polytope point
  void updateObjective(MCStabilityPolytope & polytope,
                       Eigen::Vector3d currentPos,
                       Eigen::Vector3d & objective,
                       whatRobot rob);

private:
  mc_rtc::Configuration config_;

  double t_ = 0.0;

  Eigen::Vector3d comDesired_;
  Eigen::Vector3d comDesiredHum_;

  bool handContactsForBalance_ = false;

  // Hacky check to add panda task when positions have been reset
  bool pandaTaskAdded_ = false;
  std::shared_ptr<mc_tasks::TransformTask> pandaTransform_;

  // model mode to choose to compute VRP using CoM acceleration (true) or contact forces (false)
  bool modelMode_ = true;

  bool OmegaZAcc_ = true;
  bool FilteredDerivation_ = true;

  Eigen::Vector3d DCMobjective_; // formerly constant xsensFinalpos_
  Eigen::Vector3d robDCMobjective_;

  Eigen::Vector3d robMeasuredDCM_;

  // Missing forces to apply at CoM human to achieve dynamic balance
  // Eigen::Vector3d missingForces_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d xsensCoMpos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d xsensCoMvel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d xsensCoMacc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rawxsensCoMacc_ = Eigen::Vector3d::Zero();

  double cutoffPeriod_ = 0.05;
  mc_filter::LowPass<Eigen::Vector3d> accLowPass_;

  double DCMpropgain_ = 3.0;
  double DCMinteggain_ = 0;

  std::shared_ptr<DCM_VRPtracker> humanDCMTracker_;
  std::shared_ptr<DCM_VRPtracker> robotDCMTracker_;

  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabTask_;
  std::shared_ptr<mc_tasks::lipm_stabilizer::internal::Contact> leftHandContact_;
  std::shared_ptr<mc_tasks::lipm_stabilizer::internal::Contact> rightHandContact_;

  Eigen::QuadProgDense vrpSolver_;

  std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintPtr_;
  std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintHumPtr_;

  std::vector<mc_rbdyn::Plane> planes_;
  std::vector<mc_rbdyn::Plane> planesHum_;

  std::shared_ptr<mc_solver::QPSolver> humanSolver_;

  std::vector<Eigen::Vector3d> traj_;

  double humanMass_ = 50; // 52 + 2*1.1 + 10 + 2*2.3; // Celia is 52, each force shoe 1.1kg, body weight 10 kg, legs
                          // weights 2.3kg, wrists weights 1.5kg
  bool withSuit_ = false;
  bool withLegs_ = false;
  bool withWrists_ = false;
  bool withShoes_ = true;

  const double shoesMass_ = 1.1;
  const double wristsMass_ = 1.5;
  const double legsMass_ = 2.3;
  const double vestMass_ = 10;

  /* Non normalized vector representing the plane (todo: normalize or implement a gui func to represent the polytopes)
   */
  // std::vector<Eigen::Vector3d> edgesPoly_;

  int polytopeIndex_ = 0;
  int polytopeHumIndex_ = 0;

  /*! Bound the acceleration of the CoM
   * Home made constraint (Thanks Mohamed!)
   */
  // std::shared_ptr<BoundCoMAcceleration> comAccConstr_;

  /*! Bound the  velocity of the CoM
   * Home made constraint (Thanks Mohamed!)
   */
  // std::shared_ptr<BoundCoMVelocity> comVelConstr_;

  /*! Contact set futurePolytope_ will be computed from
   */
  std::shared_ptr<ContactSet> contactSet_ = std::make_shared<ContactSet>(true); // false is to use the robust case

  /*! Contact set futurePolytope_ will be computed from
   */
  std::shared_ptr<ContactSet> contactSetHum_ =
      std::make_shared<ContactSet>(true); // probably not necessary? contactSet updated everytime

  /** @name Threaded polytope computation
   *  @{
   */
  MCStabilityPolytope robotPolytope_;
  MCStabilityPolytope humanPolytope_;
  bool firstPolyRobOK_ = false;
  bool firstPolyHumOK_ = false;
  /** @} */

  // /*! vector of tolerated accelerations for human polytope
  // */
  // std::vector<Eigen::Vector3d> humaccelerations_;

  Eigen::Vector3d combinedCoM_;

  double chebichevCoef_ = 0.9;

  double cutoffPeriodForceShoes_ = 0.05;
  mc_filter::LowPass<sva::ForceVecd> lowPassLF_, lowPassRF_, lowPassLB_, lowPassRB_;
  std::vector<sva::ForceVecd> LFShoeVec_, RFShoeVec_, LBShoeVec_, RBShoeVec_;
  sva::ForceVecd LFShoe_, RFShoe_, LBShoe_, RBShoe_;
  sva::ForceVecd LCheekForce_, RCheekForce_ = sva::ForceVecd::Zero();
  sva::ForceVecd ButtForce_ = sva::ForceVecd::Zero();
  sva::ForceVecd LHwrench_, RHwrench_;
  sva::ForceVecd RedistribWrench_;

  // storing desired max and min forces if they differ from the value given in the config file
  std::map<std::string, double> contactFMax_;
  std::map<std::string, double> contactFMin_;

  // Eigen::Vector3d com_c_, comp_c_, compp_c_; // targets for the comTask

  // Hands contact task
  std::shared_ptr<mc_tasks::EndEffectorTask> rightHandTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> leftHandTask_;

  // DynamicsConstraint for human model
  // mc_solver::DynamicsConstraint humanDynamicsConstraint_;

  // bool hasRightHandAdmittanceTask_, hasLeftHandAdmittanceTask_;
  // Eigen::Vector3d LHForceAdmittanceCoef_, LHWrenchAdmittanceCoef_;
  // Eigen::Vector3d RHForceAdmittanceCoef_, RHWrenchAdmittanceCoef_;

  // Surfaces to check collisions
  std::shared_ptr<mc_rbdyn::Surface> BackSurf, RightShoulderSurf, RCheekSurf, LCheekSurf, RFootSurf, LFootSurf, TopSurf,
      RHandSurf, LHandSurf, GroundSurf;
  std::shared_ptr<mc_control::SimulationContactPair> RCheekChair, LCheekChair, RFootGround, LFootGround, RHandShoulder,
      LHandBack;

public:
  using mc_control::fsm::Controller::updateContacts;
};
