#pragma once
#include <mc_control/fsm/Controller.h>

#include <mc_filter/LeakyIntegrator.h>
#include <mc_filter/LowPass.h>
#include <mc_filter/LowPassCompose.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <boost/circular_buffer.hpp>
#include <gram_savitzky_golay/gram_savitzky_golay.h>

/*
This class tracks the DCM dynamics of the robot or human using the time-varying natural frequency omega,
and computes the command VRP necessary to achieve a desired DCM.
The external forces to be applied on the tracked robot or human can be computed in order to be given as tasks objectives
to another robot
*/

class DCM_VRPtracker
{
public:
  DCM_VRPtracker(double timeStep, double cutoffPeriod, double mass, double propGain, double integGain);

  /*! \brief Updates and computes the dynamic points tracked.
     CAREFUL this should be called only once per controller iteration because of the internal filters
     *
     */
  void updateTrackedValues();
  void updateObjectiveValues(const Eigen::Vector3d & DCMobjective);

  void setCoMDyn(const Eigen::Vector3d & posCoM, const Eigen::Vector3d & velCoM, const Eigen::Vector3d & accCoM);
  void setAppliedForces(const std::vector<std::pair<sva::PTransformd, sva::ForceVecd>> & forceContacts);

  void setModelMode(bool model)
  {
    modelMode_ = model;
  };

  void resetTracker(Eigen::Vector3d posCoM, Eigen::Vector3d velCoM, Eigen::Vector3d accCoM);

  /*! \brief function called in the constructor to set up the GUI elements
   */
  void addGuiElements(mc_rtc::gui::StateBuilder & gui);

  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /*! \brief function called in the constructor to set up the log entries
   */
  void addLogEntries(std::string robotName, mc_rtc::Logger & logger);

  void removeLogEntries(mc_rtc::Logger & logger);

  // CAREFUL is this the right way or should it be -9.81?
  Eigen::Vector3d gravityVec()
  {
    return Eigen::Vector3d(0.0, 0.0, 9.81);
  }

  /********************Getters*********************/

  const Eigen::Vector3d & getDCM()
  {
    return DCM_;
  }

  const double & getOmega()
  {
    return omega_;
  }

  const sva::ForceVecd & getAppliedForcesSum()
  {
    return appliedForces_;
  }

  const Eigen::Vector3d & getModelVRP()
  {
    return modelVRP_;
  }

  const Eigen::Vector3d & getForcesVRP()
  {
    return measuredForcesVRP_;
  }

  const sva::ForceVecd & getMissingForces()
  {
    return missingForces_;
  }

  const Eigen::Vector3d & getDCMerror()
  {
    return DCMerror_;
  }

  const Eigen::Vector3d & getVRPerror()
  {
    return VRPerror_;
  }

  const Eigen::Vector3d & getCommandVRP()
  {
    return commandVRP_;
  }

protected:
  /* \brief Computes the time-varying natural frequency of the inverted pendulum model
   *
   */
  void computeOmega();

  /* \brief Computes the variation of time-varying natural frequency of the inverted pendulum model
   *
   */
  void computeDotOmega();

  /* \brief Computes the Divergent Component of Motion with a time-varying omega
   *
   */
  void computeDCM();

  /* \brief Computes the current measured DCM error
   *
   */
  void computeDCMerror();

  /* \brief Computes the moving average error of the DCM using a leaky integrator (integral term of VRP command)
   *
   */
  void computeIntegDCMerror()
  {
    DCMintegrator_.add(DCMerror_, timeStep_);
    DCMavgerror_ = DCMintegrator_.eval();
  }

  /* \brief Compute the VRP value from the current CoM dynamics
   *
   */
  void computeModelVRP();

  /* \brief Compute the VRP value from the external forces applied to the CoM
   *
   */
  void computeForcesVRP();

  void computeCombinedVRP();

  /* \brief Compute the VRP command necessary to achieve the desired DCM
   *
   */
  void computeCommandVRP(double P, double I);

  /* \brief Computes the error between the current VRP and the computed desired VRP (command VRP)
   *
   */
  void computeVRPerror();

  /* \brief Required missing forces at the CoM to achieve desired VRP command
   *
   */
  void computeMissingForces();

private:
  /* controller timestep (for filters)
   */
  const double timeStep_;

  /* mass of the tracked humanoid
   */
  const double mass_;

  /* model mode to choose to compute VRP using CoM acceleration (true) or contact forces (false)
   */
  bool modelMode_ = true;

  /*Computing omega using vertical CoM acceleration (true) or not (false)
   */
  bool omegaZAcc_ = true;

  /* Computing dot values using gram savitzky golay filtering (true) or simple 1-iteration variation (false)
   */
  bool filteredDerivation_ = true;

  /* filter for dcm integrator
   */
  mc_filter::LeakyIntegrator<Eigen::Vector3d> DCMintegrator_;

  /* composing filter for VRP measured through forces and VRP measured through accelerations
   */
  mc_filter::LowPassCompose<Eigen::Vector3d> VRPestimator_;

  /* lowPass for dot omega filtering
   */
  mc_filter::LowPass<Eigen::Vector3d> omegaLowPass_;

  /* filter buffer for omega derivative
   */
  boost::circular_buffer<double> omegaBuffer_;

  Eigen::Vector3d posCoM_, velCoM_, accCoM_ = Eigen::Vector3d::Zero();
  double omega_, dotOmega_, prevOmega_ = 0.0;
  Eigen::Vector3d DCM_, DCMerror_, DCMavgerror_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d DCMobjective_;

  /* measured VRP using the CoM acceleration model (modelMode true)
   */
  Eigen::Vector3d modelVRP_ = Eigen::Vector3d::Zero();

  /* measured VRP using the measured multi-contact forces applied to the CoM (modelMode false)
     careful: this assumes all forces acting on the CoM are accounted for !
  */
  Eigen::Vector3d measuredForcesVRP_ = Eigen::Vector3d::Zero();

  /* combined estimation of VRP using high pass on model VRP and low pass on forces VRP
   */
  Eigen::Vector3d combinedVRP_ = Eigen::Vector3d::Zero();

  sva::ForceVecd appliedForces_ = sva::ForceVecd::Zero();

  double propGain_, integGain_;
  /* computed VRP to apply to achieve desired DCM
   */
  Eigen::Vector3d commandVRP_ = Eigen::Vector3d::Zero();

  /* Error between current VRP and command VRP
   */
  Eigen::Vector3d VRPerror_ = Eigen::Vector3d::Zero();

  /* Missing forces to apply at the CoM to achieve desired VRP
   */
  sva::ForceVecd missingForces_ = sva::ForceVecd::Zero();
};
