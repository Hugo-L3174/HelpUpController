#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_solver/CoMIncPlaneConstr.h>

#include <mc_tasks/CoMTask.h>

#include "Tasks/BoundCoMAcceleration.h"
#include "Tasks/CoMAccelerationTask.h"
#include "Tasks/BoundCoMVelocity.h"

#include "utils/ComputationPoint.h"

#include <thread>
#include <atomic>

#include "api.h"

struct HelpUpController_DLLAPI HelpUpController : public mc_control::fsm::Controller
{
    HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    /*! \brief function called in the constructor to set up the log entries
     */ 
    void addLogEntries();

    /*! \brief function called in the constructor to set up the GUI elements
     */
    void addGuiElements();

    /*! \brief Set the planes for the CoMIncPlaneConstr
     */
    void planes(std::vector<mc_rbdyn::Plane> constrPlanes);
    
    /*! \brief Set the planes for the CoMIncPlaneConstr
     */
    void planes(std::vector<Eigen::Vector4d> constrPlanes);

    /*! \brief Gives the planes currently used bu CoMIncPlaneConstr
     */
    const std::vector<mc_rbdyn::Plane> planes() const;

    /*! \brief Set the CoM target for CoM task
     */
    // void desiredCoM(Eigen::Vector3d desiredCoM);

    /*! \brief Gives acces to target CoM of the CoM task
     */
    Eigen::Vector3d currentCoM() const;

    // inline double comTaskWeight() const
    // {
    //   return comTask_->weight();
    // }

    // void comTaskWeight(double weight);

    // inline double comTaskStiffness() const
    // {
    //   return comTask_->stiffness();
    // }

    // void comTaskStiffness(double stiffness);

    // inline double comTaskDamping() const
    // {
    //   return comTask_->damping();
    // }

    // void comTaskDamping(double damping);

    // void targetCoM(const Eigen::Vector3d & com, const Eigen::Vector3d & comp = Eigen::Vector3d::Zero(), const Eigen::Vector3d & compp = Eigen::Vector3d::Zero());

    void increasePolytopeIndex();
    int getPolytopeIndex();

    /*! \brief Update the contactSet with the current contacts of the rbdyn robot object
     */
    void updateContactSet(unsigned int robotIndex);

    /*! \brief Update the contactSet with the given contacts
     */
    void updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex);

    /*! \brief Update the normal contact force upper and lower bound in the contactSetObject
     * \param contactFMax map of the names of the contacts to update with the corresponding upper bounds
     * \param contactFMin map of the names of the contacts to update with the corresponding lower bounds
     */
    void contactForces(std::map<std::string, double> contactFMax, std::map<std::string, double> contactFMin);

    void updateContactForces();


    /*! \brief Compute the equilibrium region stored in futurePolytope using contactSet as input
     */ 
    void computeStabilityRegion(bool save=false, int polIndex=0, std::string suffix="");

    /*! \brief Dangerous function that move future to current without asking questions
     */
    void setFutureToCurrent();

    /*! \brief Set the Next Polytope as a the future one 
     * Prepare the objective for the transition
     */
    void setFutureToNext();

    /*! \brief Set the current polytope as the next one
     * This method should be called once the transition to next is finished
     */
    void setNextToCurrent();

    /*! \brief Start the transition from current to next
     */
    inline void startTransition()
    {
      transitionning_ = true;
    }

    /*! \brief Used to check if the transition from current to next has finished
     */
    inline bool transitionFinished()
    {
      return !transitionning_;
    }
    
    inline void setReady()
    {
      readyForComp_ = true;
    }
    
    inline bool computationFinished() const
    {
      return (!computing_ and computed_);
    }

    /*! \brief Utility function to check if a vertex is inside a set of planes
     * \param Vertex vertex to test
     * \param planes planes to test with
     * \param eps minimum distance from a plane for the vertex to be considered inside
     */
    bool isVertexInPlanes(Eigen::Vector3d Vertex, std::vector<Eigen::Vector4d> planes, double eps=0.01);


private:
    mc_rtc::Configuration config_;

    // std::shared_ptr<mc_tasks::CoMTask> comTask_;
    // Eigen::Vector3d comDesired_;

    std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintPtr_;
    std::vector<mc_rbdyn::Plane> planes_;

    int polytopeIndex_;
    
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

    /*! Thread stuff
     */
    std::thread stabThread_;
    std::atomic<bool> polytopeReady_;


    bool readyForComp_;
    bool computing_;
    bool computed_;

    bool transitionning_;

    // storing desired max and min forces if they differ from the value given in the config file
    std::map<std::string, double> contactFMax_;
    std::map<std::string, double> contactFMin_;

    // Eigen::Vector3d com_c_, comp_c_, compp_c_; // targets for the comTask

    // Hands contact task
    std::shared_ptr<mc_tasks::EndEffectorTask> rightHandTask_;
    std::shared_ptr<mc_tasks::EndEffectorTask> leftHandTask_;

    // DynamicsConstraint for human model
    mc_solver::DynamicsConstraint humanDynamicsConstraint_;

};