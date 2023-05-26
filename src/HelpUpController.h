#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_solver/CoMIncPlaneConstr.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <mc_tasks/CoMTask.h>

#include "Tasks/BoundCoMAcceleration.h"
#include "Tasks/CoMAccelerationTask.h"
#include "Tasks/BoundCoMVelocity.h"

#include "utils/ComputationPoint.h"
#include "utils/TrajectoryModel.h"
#include <mc_rtc/gui/plot.h>

#include <mc_control/SimulationContactPair.h>

#include <thread>
#include <atomic>

#include "api.h"

enum whatRobot
{
  hrp4,
  human,
  combined
};

enum HelpUpControllerContactType {
  RH, // right hand
  LH, // left hand
  RF, // right foot
  LF  // left foot
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

    /*! \brief Set the planes for the CoMIncPlaneConstr
     */
    void planes(std::vector<mc_rbdyn::Plane> constrPlanes, whatRobot rob);
    
    /*! \brief Set the planes for the CoMIncPlaneConstr
     */
    void planes(std::vector<Eigen::Vector4d> constrPlanes, whatRobot rob);

    // /*! \brief Gives the planes currently used bu CoMIncPlaneConstr
    //  */
    // const std::vector<mc_rbdyn::Plane> planes() const;

    /*! \brief Set the CoM target for CoM task
     */
    void desiredCoM(Eigen::Vector3d desiredCoM, whatRobot rob);

    /*! \brief Gives acces to target CoM of the CoM task
     */
    Eigen::Vector3d currentCoM(std::string robotName) const;

    inline double comTaskWeight(std::shared_ptr<mc_tasks::CoMTask> CoMTask) const
    {
      return CoMTask->weight();
    }

    void comTaskWeight(double weight, std::shared_ptr<mc_tasks::CoMTask> CoMTask);

    inline double comTaskStiffness(std::shared_ptr<mc_tasks::CoMTask> CoMTask) const
    {
      return CoMTask->stiffness();
    }

    void comTaskStiffness(double stiffness, std::shared_ptr<mc_tasks::CoMTask> CoMTask);

    inline double comTaskDamping(std::shared_ptr<mc_tasks::CoMTask> CoMTask) const
    {
      return CoMTask->damping();
    }

    void comTaskDamping(double damping, std::shared_ptr<mc_tasks::CoMTask> CoMTask);

    void targetCoM(const Eigen::Vector3d & com, const Eigen::Vector3d & comp = Eigen::Vector3d::Zero(), const Eigen::Vector3d & compp = Eigen::Vector3d::Zero());

    void increasePolytopeIndex(int polyIndex);
    int getPolytopeIndex(int polyIndex);

    /*! \brief Update the contactSet with the current contacts of the rbdyn robot object
     */
    void updateContactSet(unsigned int robotIndex, whatRobot rob);

    /*! \brief Update the contactSet with the given contacts
     */
    void updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex, whatRobot rob);

    /*! \brief Update the normal contact force upper and lower bound in the contactSetObject
     * \param contactFMax map of the names of the contacts to update with the corresponding upper bounds
     * \param contactFMin map of the names of the contacts to update with the corresponding lower bounds
     */
    void contactForces(std::map<std::string, double> contactFMax, std::map<std::string, double> contactFMin);

    void updateContactForces();

    void updateCombinedCoM();


    /*! \brief Compute the equilibrium region stored in futurePolytope using contactSet as input
     */ 
    void computeStabilityRegion(std::shared_ptr<ContactSet> contactset, whatRobot rob, bool save=false, int polIndex=0, std::string suffix="");

    // /*! \brief Dangerous function that move future to current without asking questions
    //  */
    // void setFutureToCurrent(std::shared_ptr<ComputationPoint> current, std::shared_ptr<ComputationPoint> future);

    // /*! \brief Set the Next Polytope as a the future one 
    //  * Prepare the objective for the transition
    //  */
    // void setFutureToNext(std::shared_ptr<ComputationPoint> future, std::shared_ptr<ComputationPoint> next);

    /*! \brief Set the current polytope as the next one
     * This method should be called once the transition to next is finished
     */
    void setNextToCurrent(whatRobot rob);

    /*! \brief Utility function to check if a vertex is inside a set of planes
     * \param Vertex vertex to test
     * \param planes planes to test with
     * \param eps minimum distance from a plane for the vertex to be considered inside
     */
    bool isVertexInPlanes(Eigen::Vector3d Vertex, std::vector<Eigen::Vector4d> planes, double eps=0.01);

    std::map<std::string, double> getConfigFMax() const;
    std::map<std::string, double> getConfigFMin() const;

    inline bool transitionFinished()
    {
      return !transitionning_;
    }

    void addRightHandAdmittanceTask();
    void addLeftHandAdmittanceTask();

    // security issue
    std::shared_ptr<mc_tasks::CoMTask> getComTaskHum()
    {
      return comTaskHum_;
    }
  
    std::optional<double> override_CoMz;

    // Checking if distance between real human feet/ground - butt/chair - (robot hands/back and shoulder? maybe robotside) is low enough to consider a contact
    void updateRealHumContacts();

    void addRealHumContact(std::string humanSurfName, double fmin, double fmax, ContactType type);

    void setxsensCoM(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc)
    {
      xsensCoMpos_ = pos;
      xsensCoMvel_ = vel;
      xsensCoMacc_ = acc;
    };

    double humanOmega()
    {
      return std::sqrt(9.81/xsensCoMpos_.z());
    };

    double humanb()
    {
      return 1/humanOmega();
    };

    double dotHumanOmega()
    {
      return 0;
    };

    double mainOmega()
    {
      return std::sqrt(9.81/robot().com().z());
    };

    double mainRealOmega()
    {
      return std::sqrt(9.81/realRobot().com().z());
    };

    Eigen::Vector3d humanXsensDCM()
    {
      return xsensCoMpos_ + xsensCoMvel_ / humanOmega();
    };

    Eigen::Vector3d dotHumanXsensDCM()
    {
      return xsensCoMvel_ + xsensCoMacc_ / humanOmega();
    };

    Eigen::Vector3d humanXsensVRP()
    {
      // return xsensCoMpos_ + xsensCoMacc_ / (humanOmega()*humanOmega() - dotHumanOmega());
      return humanXsensDCM() - humanb() * dotHumanXsensDCM();
    };

    Eigen::Vector3d mainCtlDCM()
    {
      return robot().com() + robot().comVelocity() / mainOmega();
    };

    Eigen::Vector3d mainRealDCM()
    {
      return realRobot().com() + realRobot().comVelocity() / mainRealOmega();
    };


private:
    mc_rtc::Configuration config_;

    double t_ = 0.0;

    std::shared_ptr<mc_tasks::CoMTask> comTask_;
    std::shared_ptr<mc_tasks::CoMTask> comTaskHum_;
    Eigen::Vector3d comDesired_;
    Eigen::Vector3d comDesiredHum_;

    Eigen::Vector3d xsensCoMpos_;
    Eigen::Vector3d xsensCoMvel_;
    Eigen::Vector3d xsensCoMacc_;

    std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabTask_;

    std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintPtr_;
    std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintHumPtr_;

    std::vector<mc_rbdyn::Plane> planes_;
    std::vector<mc_rbdyn::Plane> planesHum_;

    std::shared_ptr<mc_solver::QPSolver> humanSolver_; 

    std::shared_ptr<TrajectoryModel> trajectories_; 
    std::vector<Eigen::Vector3d> traj_;

    double humanMass_ = 65;


    /* Non normalized vector representing the plane (todo: normalize or implement a gui func to represent the polytopes)
    */
    // std::vector<Eigen::Vector3d> edgesPoly_; 

    int polytopeIndex_;

    int polytopeHumIndex_;
    
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
    std::shared_ptr<ContactSet> contactSet_ = std::make_shared<ContactSet> (true); // false is to use the robust case

    /*! Contact set futurePolytope_ will be computed from
     */
    std::shared_ptr<ContactSet> contactSetHum_ = std::make_shared<ContactSet> (true); // probably not necessary? contactSet updated everytime

    /*! computed currently used planes
     */
    std::shared_ptr<ComputationPoint> currentCompPoint_;
    /*! computed planes that will be used next
     */ 
    std::shared_ptr<ComputationPoint> nextCompPoint_;
    // std::shared_ptr<StabilityPolytope> nextPolytope_;
    /*! planes to be computed or currently computed
     */
    std::shared_ptr<ComputationPoint> futureCompPoint_;
    /*! computed polytope for display (no constraint on the CoM)
     */
    std::shared_ptr<ComputationPoint> balanceCompPoint_;

    // /*! vector of tolerated accelerations for hrp4 polytope
    // */
    // std::vector<Eigen::Vector3d> accelerations_;


    /*! computed currently used planes
     */
    std::shared_ptr<ComputationPoint> currentHumCompPoint_;
    /*! computed planes that will be used next
     */ 
    std::shared_ptr<ComputationPoint> nextHumCompPoint_;
    // std::shared_ptr<StabilityPolytope> nextPolytope_;
    /*! planes to be computed or currently computed
     */
    std::shared_ptr<ComputationPoint> futureHumCompPoint_;
    /*! computed polytope for display (no constraint on the CoM)
     */
    std::shared_ptr<ComputationPoint> balanceHumCompPoint_;

    // /*! vector of tolerated accelerations for human polytope
    // */
    // std::vector<Eigen::Vector3d> humaccelerations_;

    Eigen::Vector3d combinedCoM_;

    /*! Thread stuff
     */
    std::thread stabThread_;
    std::thread stabThreadHum_;

    std::atomic<bool> polytopeReady_;

    std::atomic<bool> polytopeHumReady_;


    bool readyForComp_;
    bool computing_;
    bool computed_;

    bool transitionning_;

    bool readyForCompHum_;
    bool computingHum_;
    bool computedHum_;

    bool transitionningHum_;

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

    // std::shared_ptr<mc_rbdyn::Frame> Back, RightShoulder, RCheek, LCheek, RFoot, LFoot;

    // Surfaces to check collisions
    std::shared_ptr<mc_rbdyn::Surface> BackSurf, RightShoulderSurf, RCheekSurf, LCheekSurf, RFootSurf, LFootSurf, TopSurf, RHandSurf, LHandSurf, GroundSurf;
    std::shared_ptr<mc_control::SimulationContactPair> RCheekChair, LCheekChair, RFootGround, LFootGround, RHandShoulder, LHandBack;

public:
    using mc_control::fsm::Controller::updateContacts;

};