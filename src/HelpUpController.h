#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_solver/CoMIncPlaneConstr.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <mc_tasks/CoMTask.h>

// #include "Tasks/BoundCoMAcceleration.h"
// #include "Tasks/CoMAccelerationTask.h"
// #include "Tasks/BoundCoMVelocity.h"

#include "utils/ComputationPoint.h"
#include <mc_rtc/gui/plot.h>

#include <mc_control/SimulationContactPair.h>

#include <mc_tasks/lipm_stabilizer/Contact.h>
#include <thread>
#include <atomic>

#include <gram_savitzky_golay/gram_savitzky_golay.h>
#include <boost/circular_buffer.hpp>
#include <mc_filter/LeakyIntegrator.h>

#include "api.h"

enum whatRobot
{
  mainRob,
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
    void updateContactSet(unsigned int robotIndex);

    /*! \brief Update the contactSet with the given contacts
     */
    // void updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex, whatRobot rob);
    void updateContactSet(std::vector<mc_rbdyn::Contact> contacts, unsigned int robotIndex);

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

    // inline bool transitionFinished()
    // {
    //   return !transitionning_;
    // }

  
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

    // CAREFUL is this the right way or should it be -9.81?
    Eigen::Vector3d gravityVec()
    {
      return Eigen::Vector3d(0.0, 0.0, 9.81);
    }

    void computeHumanOmega()
    {
      if (OmegaZAcc_)
      {
        humanOmega_ = std::sqrt((9.81 + xsensCoMacc_.z())/xsensCoMpos_.z());
      }
      else
      {
        humanOmega_ = std::sqrt(9.81/xsensCoMpos_.z());
      }
          
    };

    void computeDotHumanOmega()
    {
      if (FilteredDerivation_)
      {
        gram_sg::SavitzkyGolayFilter filter(9, 9, 4, 1, timeStep);
        dotHumanOmega_ = filter.filter(humanOmegaBuffer_);
      }
      else
      {
        dotHumanOmega_ = ((humanOmega_ - prevOmega_)/timeStep);
      }
      // return ((humanOmega() - prevOmega_)/timeStep);

      // config: m = 9 (Window size is 2*m+1), t = m = 9 (evaluate polynomial at first point in the [-m;m] window) , n = 4 (Polynomial Order), s = 1 (first order derivative), dt = timestep 
      
    };

    double mainOmega()
    {
      return std::sqrt(9.81/robot().com().z());
    };


    Eigen::Vector3d humanXsensDCM()
    {
      return xsensCoMpos_ + xsensCoMvel_ / humanOmega_;
    };

    // This is the previous assumption that omega is a constant
    // Eigen::Vector3d dotHumanXsensDCM()
    // {
    //   return xsensCoMvel_ + xsensCoMacc_ / humanOmega();
    // };

    // Eigen::Vector3d humanXsensVRP()
    // {
    //   // return xsensCoMpos_ + xsensCoMacc_ / (humanOmega()*humanOmega() - dotHumanOmega());
    //   return humanXsensDCM() - dotHumanXsensDCM() / humanOmega();
    // };

    Eigen::Vector3d humanVRPmodel()
    {
      return xsensCoMpos_ - (xsensCoMacc_/(humanOmega_*humanOmega_-dotHumanOmega_));
    }

    // This is equivalent to the model except we replace the acceleration with the sum of forces applied to the CoM, divided by the human mass (Newton: SumF = m*acc)
    Eigen::Vector3d humanVRPmeasured()
    {
      return xsensCoMpos_ - (humanVRPforces().force() - humanMass_*gravityVec() )/(humanMass_ * (humanOmega_*humanOmega_-dotHumanOmega_));
    }


    sva::ForceVecd humanVRPforces()
    {
      auto X_0_C = sva::PTransformd(xsensCoMpos_);
      auto X_LF_0 = robot("human").surfacePose("LFsensor").inv();
      auto X_LB_0 = robot("human").surfacePose("LBsensor").inv();
      auto X_RF_0 = robot("human").surfacePose("RFsensor").inv();
      auto X_RB_0 = robot("human").surfacePose("RBsensor").inv();


      auto w_LF_0 = X_LF_0.dualMul(LFShoe_);
      auto w_LB_0 = X_LB_0.dualMul(LBShoe_);
      auto w_RF_0 = X_RF_0.dualMul(RFShoe_);
      auto w_RB_0 = X_RB_0.dualMul(RBShoe_);


      auto Fc = X_0_C.dualMul(w_LF_0) + X_0_C.dualMul(w_LB_0) + X_0_C.dualMul(w_RF_0) + X_0_C.dualMul(w_RB_0);
      
      // Do this with transforms directly to the com instead of world origin: high moments can lead to numerical errors
      return Fc;
    }

    // sva::ForceVecd estimatedChairForce()
    // {
    //   auto X_0_C = sva::PTransformd(xsensCoMpos_);
      
    // }

    // Required missing forces on the hands to achieve desired VRP command
    // Change this to missing forces to achieve *missing* VRP command, not total
    void computeMissingForces()
    {
      if (modelMode_)
      {
        missingForces_ = humanMass_ * (humanOmega_*humanOmega_-dotHumanOmega_) * (xsensCoMpos_ - commandVRP_) - humanMass_ * xsensCoMacc_;  
      }
      else
      {
        missingForces_ = humanMass_ * (humanOmega_*humanOmega_-dotHumanOmega_) * (xsensCoMpos_ - commandVRP_) + humanMass_ * gravityVec() - humanVRPforces().force() /*Missing butt contact force !*/; 
      }      
    }

    void computeDCMerror()
    {
      DCMerror_ = DCMobjective_ - humanXsensDCM(); // DCMobjective formerly constant xsensFinalpos_
    }

    void computeVRPerror()
    {
      if (modelMode_)
      {
        VRPerror_ = commandVRP_ - humanVRPmodel();
      }
      else
      {
        VRPerror_ = commandVRP_ - humanVRPmeasured();
      } 
      
    }

    void computeDCMobjectiveVel()
    {
      // config: m = 9 (Window size is 2*m+1), t = m = 9 (evaluate polynomial at first point in the [-m;m] window) , n = 4 (Polynomial Order), s = 1 (first order derivative), dt = timestep 
      if(FilteredDerivation_)
      {
        gram_sg::SavitzkyGolayFilter filter(9, 9, 4, 1, timeStep);
        DCMobjectiveVel_ = filter.filter(DCMobjectiveBuffer_);
      } 
      else
      {
        DCMobjectiveVel_ = ((DCMobjective_ - prevDCMobjective_)/timeStep);
      }
    }

    void computeIntegDCMerror()
    {
      DCMintegrator_.add(DCMerror_, timeStep);
      DCMaverageError_ = DCMintegrator_.eval();
    }


    void computeCommandVRP()
    {
      // TODO fix feedforward term
      commandVRP_ = humanXsensDCM() - (1/(humanOmega_-(dotHumanOmega_/humanOmega_)))*(/*DCMobjectiveVel_*/ + DCMpropgain_*(DCMerror_) + DCMinteggain_ * DCMaverageError_);
    }


    void distributeHandsWrench(const sva::ForceVecd & desiredWrench);


    Eigen::Vector3d mainCtlDCM()
    {
      return robot().com() + robot().comVelocity() / mainOmega();
    }

    // Eigen::Vector3d mainRealDCM()
    // {
    //   return realRobot().com() + realRobot().comVelocity() / mainRealOmega();
    // };

    // Parametrized start offset of the log to sychronize. Default: start offset at 0, acquisition frequency of force shoes at 100Hz
    sva::ForceVecd getCurrentForceVec(std::vector<sva::ForceVecd> log, double startOffset = 0, double freq = 100);

    sva::ForceVecd getLHWrenchComputed()
    {
      return LHwrench_;
    }

    sva::ForceVecd getRHWrenchComputed()
    {
      return RHwrench_;
    }

    // This should: compute the polytope with the current contact set, update the DCM objective and give it to the stabilizer task associated to the robot
    void computePolytope(bool & computing, bool & readyToComp, bool & computed, bool & transitionning, bool & polyReady, 
                        std::thread & thread, 
                        std::shared_ptr<ContactSet> contactSet, 
                        std::shared_ptr<ComputationPoint> & futureCompPoint, 
                        std::shared_ptr<ComputationPoint> & balanceCompPoint, 
                        whatRobot rob);

    // Todo: add options in both functions to choose if we check CoM or DCM, wether to update the objective or the robot stabilizer via datastore,
    // and which update contacts function to use (human is real contacts + hard written surfaces)

    // Update the objective: if the currentPos is in the balance polytope, then objective stays current CoM/DCM, otherwise closest polytope point
    void updateObjective(std::shared_ptr<ComputationPoint> & balanceCompPoint,
                        Eigen::Vector3d currentPos, 
                        Eigen::Vector3d & objective, 
                        whatRobot rob);

private:
    mc_rtc::Configuration config_;

    double t_ = 0.0;

    Eigen::Vector3d comDesired_;
    Eigen::Vector3d comDesiredHum_;

    bool handContactsForBalance_ = false;

    // model mode to choose to compute VRP using CoM acceleration (true) or contact forces (false)
    bool modelMode_ = false;

    bool OmegaZAcc_ = true;
    bool FilteredDerivation_ = true;

    Eigen::Vector3d DCMobjective_; // formerly constant xsensFinalpos_
    Eigen::Vector3d robDCMobjective_;

    // Missing forces to apply at CoM human to achieve dynamic balance
    Eigen::Vector3d missingForces_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d xsensCoMpos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d xsensCoMvel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d xsensCoMacc_ = Eigen::Vector3d::Zero();

    double humanOmega_ = std::sqrt(9.81);
    double prevOmega_ = std::sqrt(9.81);
    double dotHumanOmega_ = 0.0;

    Eigen::Vector3d DCMerror_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prevDCMobjective_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d DCMobjectiveVel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d DCMaverageError_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d VRPerror_ = Eigen::Vector3d::Zero();

    // filter buffers for omega and dcm derivatives
    boost::circular_buffer<double> humanOmegaBuffer_;
    boost::circular_buffer<Eigen::Vector3d> DCMobjectiveBuffer_;
    // filter for dcm integrator
    mc_filter::LeakyIntegrator<Eigen::Vector3d> DCMintegrator_;

    Eigen::Vector3d commandVRP_ = Eigen::Vector3d::Zero();
    double DCMpropgain_ = 3.0;
    double DCMdgain_ = 0.3;
    double DCMinteggain_ = 0.3;

    std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabTask_;
    std::shared_ptr<mc_tasks::lipm_stabilizer::internal::Contact> leftHandContact_;
    std::shared_ptr<mc_tasks::lipm_stabilizer::internal::Contact>  rightHandContact_;

    Eigen::QuadProgDense vrpSolver_;

    std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintPtr_;
    std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintHumPtr_;

    std::vector<mc_rbdyn::Plane> planes_;
    std::vector<mc_rbdyn::Plane> planesHum_;

    std::shared_ptr<mc_solver::QPSolver> humanSolver_; 

    std::vector<Eigen::Vector3d> traj_;

    double humanMass_ = 50; //52 + 2*1.1 + 10 + 2*2.3; // Celia is 52, each force shoe 1.1kg, body weight 10 kg, legs weights 2.3kg, wrists weights 1.5kg
    bool withSuit_ = false;
    bool withLegs_ = false;
    bool withWrists_ = false;
    bool withShoes_ = true;

    const double shoesMass_ = 1.1;
    const double wristsMass_ = 1.5;
    const double legsMass_ = 2.3;
    const double vestMass_ =10;


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

    bool polytopeReady_;

    bool polytopeHumReady_;


    // Boolean flags
    bool readyForComp_;
    bool computing_;
    bool computed_;

    bool firstPolyRobOK_;

    bool readyForCompHum_;
    bool computingHum_;
    bool computedHum_;

    bool firstPolyHumOK_;

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

    // std::shared_ptr<mc_rbdyn::Frame> Back, RightShoulder, RCheek, LCheek, RFoot, LFoot;

    // Surfaces to check collisions
    std::shared_ptr<mc_rbdyn::Surface> BackSurf, RightShoulderSurf, RCheekSurf, LCheekSurf, RFootSurf, LFootSurf, TopSurf, RHandSurf, LHandSurf, GroundSurf;
    std::shared_ptr<mc_control::SimulationContactPair> RCheekChair, LCheekChair, RFootGround, LFootGround, RHandShoulder, LHandBack;

public:
    using mc_control::fsm::Controller::updateContacts;

};