#include "DCM_VRPtracker.h"

DCM_VRPtracker::DCM_VRPtracker(double timeStep, double cutoffPeriod, double mass, double propGain, double integGain)
: timeStep_(timeStep), mass_(mass), propGain_(propGain), integGain_(integGain), omegaBuffer_(19),
  omegaLowPass_(timeStep, cutoffPeriod), VRPestimator_(timeStep, 0.05)
{
  for(size_t i = 0; i < omegaBuffer_.capacity(); i++)
  {
    omegaBuffer_.push_back(std::sqrt(9.81));
  }
}

void DCM_VRPtracker::resetTracker(Eigen::Vector3d posCoM, Eigen::Vector3d velCoM, Eigen::Vector3d accCoM)
{
  // TODO reset errors and targets to avoid high error in the beginning
}

void DCM_VRPtracker::setCoMDyn(const Eigen::Vector3d & posCoM,
                               const Eigen::Vector3d & velCoM,
                               const Eigen::Vector3d & accCoM)
{
  posCoM_ = posCoM;
  velCoM_ = velCoM;
  accCoM_ = accCoM;
}

void DCM_VRPtracker::setAppliedForces(const std::vector<std::pair<sva::PTransformd, sva::ForceVecd>> & forceContacts)
{
  appliedForces_ = sva::ForceVecd::Zero();
  auto X_0_C = sva::PTransformd(posCoM_);
  for(auto forceContact : forceContacts)
  {
    // getting inv transform of contact pose in world frame
    auto X_r_0 = forceContact.first.inv();
    // computing wrench of contact in world frame
    auto w_r_0 = X_r_0.dualMul(forceContact.second);
    // computing wrench back into CoM
    appliedForces_ += X_0_C.dualMul(w_r_0);
  }
  // TODO Do this with transforms directly to the com instead of world origin: high moments can lead to numerical errors
}

void DCM_VRPtracker::updateTrackedValues()
{
  computeOmega();
  computeDotOmega();
  // update of no filtering mode for dot omega
  prevOmega_ = omega_;
  computeDCM();
  computeModelVRP();
  computeForcesVRP(); // if model mode false?
  computeCombinedVRP();
}

void DCM_VRPtracker::updateObjectiveValues(const Eigen::Vector3d & DCMobjective)
{
  DCMobjective_ = DCMobjective;
  computeDCMerror();
  computeIntegDCMerror();
  computeCommandVRP(propGain_, integGain_);
  computeVRPerror(); // optional, for logging only
  computeMissingForces();
}

void DCM_VRPtracker::computeOmega()
{
  omega_ = std::sqrt((9.81 + accCoM_.z()) / posCoM_.z());
}

void DCM_VRPtracker::computeDotOmega()
{
  if(filteredDerivation_)
  {
    // config: m = 9 (Window size is 2*m+1), t = m = 9 (evaluate polynomial at first point in the [-m;m] window) , n =
    // 4 (Polynomial Order), s = 1 (first order derivative), dt = timestep
    omegaBuffer_.push_back(omega_);
    gram_sg::SavitzkyGolayFilter filter(9, 9, 4, 1, timeStep_);
    dotOmega_ = filter.filter(omegaBuffer_);
    // re filter with low pass
    omegaLowPass_.update(Eigen::Vector3d(0, 0, dotOmega_));
    dotOmega_ = omegaLowPass_.eval().z();
  }
  else
  {
    dotOmega_ = ((omega_ - prevOmega_) / timeStep_);
  }
}

void DCM_VRPtracker::computeDCM()
{
  DCM_ = posCoM_ + velCoM_ / omega_;
}

void DCM_VRPtracker::computeDCMerror()
{
  DCMerror_ = DCMobjective_ - DCM_;
}

void DCM_VRPtracker::computeModelVRP()
{
  modelVRP_ = posCoM_ - (accCoM_ / (omega_ * omega_ - dotOmega_));
}

// FIXME this assumes all forces go through the CoM, moments about the CoM are not taken into account
void DCM_VRPtracker::computeForcesVRP()
{
  // should this be a vector3d or a forcevecd?
  if(appliedForces_.force().array().isNaN().any())
  {
    mc_rtc::log::error("[DCM_VRPtracker] Applied forces are NaN ({}), using [0, 0, 0] instead",
                       appliedForces_.force().transpose());
    appliedForces_.force() = Eigen::Vector3d::Zero();
  }
  measuredForcesVRP_ =
      posCoM_ - (appliedForces_.force() - mass_ * gravityVec()) / (mass_ * (omega_ * omega_ - dotOmega_));
}

void DCM_VRPtracker::computeCombinedVRP()
{
  if(!modelMode_)
  {
    VRPestimator_.update(modelVRP_, measuredForcesVRP_);
    // VRPestimator_.update(measuredForcesVRP_, modelVRP_);
    combinedVRP_ = VRPestimator_.eval();
  }
  else
  {
    combinedVRP_ = modelVRP_;
  }
}

void DCM_VRPtracker::computeCommandVRP(double P, double I)
{
  // TODO fix feedforward term: is it coherent to give the current DCMvel as target while targeting a different DCM pos?
  // In practice DCMvel should be current if we are in, and zero if out: we are targeting a fixed pos.
  // if moving target (moving projection on polytope?), must prove convergence (reverse time in state of the art)
  // commandVRP_ = humanXsensDCM() - (1/(humanOmega_-(dotHumanOmega_/humanOmega_)))*(/*DCMobjectiveVel_*/ +
  // DCMpropgain_*(DCMerror_) + DCMinteggain_ * DCMaverageError_);
  Eigen::Vector3d DCMvel = Eigen::Vector3d::Zero();
  if(modelMode_)
  {
    DCMvel = (omega_ - (dotOmega_ / omega_)) * (DCM_ - modelVRP_);
  }
  else
  {
    DCMvel = (omega_ - (dotOmega_ / omega_)) * (DCM_ - measuredForcesVRP_);
  }

  commandVRP_ = DCM_ - (1 / (omega_ - (dotOmega_ / omega_))) * (DCMvel + P * DCMerror_ + I * DCMavgerror_);
}

void DCM_VRPtracker::computeVRPerror()
{
  VRPerror_ = commandVRP_ - modelVRP_;
  // for now only using acc model
  if(modelMode_)
  {
    VRPerror_ = commandVRP_ - modelVRP_;
  }
  else
  {
    VRPerror_ = commandVRP_ - measuredForcesVRP_;
  }
}

void DCM_VRPtracker::computeMissingForces()
{
  // missingForces_.force() = mass_ * (omega_ * omega_ - dotOmega_) * (posCoM_ - commandVRP_) - mass_ * accCoM_;
  if(modelMode_)
  {
    missingForces_.force() = mass_ * (omega_ * omega_ - dotOmega_) * (posCoM_ - commandVRP_) - mass_ * accCoM_;
  }
  else
  {
    missingForces_.force() = mass_ * (omega_ * omega_ - dotOmega_) * (posCoM_ - commandVRP_) + mass_ * gravityVec()
                             - appliedForces_.force(); /*Missing butt contact force !*/
  }
}

void DCM_VRPtracker::addGuiElements(mc_rtc::gui::StateBuilder & gui)
{

  const std::map<char, mc_rtc::gui::Color> COLORS = {
      {'r', mc_rtc::gui::Color{1.0, 0.0, 0.0}}, {'g', mc_rtc::gui::Color{0.0, 1.0, 0.0}},
      {'b', mc_rtc::gui::Color{0.0, 0.0, 1.0}}, {'y', mc_rtc::gui::Color{1.0, 0.5, 0.0}},
      {'c', mc_rtc::gui::Color{0.0, 0.5, 1.0}}, {'m', mc_rtc::gui::Color{1.0, 0.0, 0.5}},
      {'o', mc_rtc::gui::Color{1.0, 0.7, 0.0}}};

  mc_rtc::gui::ArrowConfig forceArrowConfig;
  forceArrowConfig.shaft_diam = 0.01;
  forceArrowConfig.head_diam = 0.015;
  forceArrowConfig.head_len = 0.01;
  forceArrowConfig.scale = 1.;
  forceArrowConfig.start_point_scale = 0.;
  forceArrowConfig.end_point_scale = 0.;

  mc_rtc::gui::ArrowConfig VRPforceArrowConfig = forceArrowConfig;
  VRPforceArrowConfig.color = COLORS.at('o');

  mc_rtc::gui::ArrowConfig DCMforceArrowConfig = forceArrowConfig;
  DCMforceArrowConfig.color = COLORS.at('c');

  mc_rtc::gui::ArrowConfig ShoesforceArrowConfig = forceArrowConfig;
  ShoesforceArrowConfig.color = COLORS.at('y');

  mc_rtc::gui::ArrowConfig MissingforceArrowConfig = forceArrowConfig;
  MissingforceArrowConfig.color = COLORS.at('g');

  gui.addElement(
      {"Points", "DCM dynamics"},
      mc_rtc::gui::Point3D("CoM", mc_rtc::gui::PointConfig(COLORS.at('b'), 0.03),
                           [this]() -> const Eigen::Vector3d & { return posCoM_; }),
      mc_rtc::gui::Point3D("DCM", mc_rtc::gui::PointConfig(COLORS.at('r'), 0.03),
                           [this]() -> const Eigen::Vector3d & { return DCM_; }),
      mc_rtc::gui::Point3D("VRP", mc_rtc::gui::PointConfig(COLORS.at('g'), 0.03),
                           [this]() -> const Eigen::Vector3d & { return combinedVRP_; }),
      // mc_rtc::gui::Point3D("VRP acceleration model", mc_rtc::gui::PointConfig(COLORS.at('g'), 0.015),
      //                      [this]() -> const Eigen::Vector3d & { return modelVRP_; }),
      // mc_rtc::gui::Point3D("VRP forces model", mc_rtc::gui::PointConfig(COLORS.at('c'), 0.015),
      //                      [this]() -> const Eigen::Vector3d & { return measuredForcesVRP_; }),
      // this is the computed vrp to achieve the desired xsensFinalpos_ (not sure)
      // mc_rtc::gui::Arrow("missingForces", MissingforceArrowConfig, [this]() -> Eigen::Vector3d { return desiredVRP();
      // }, [this]() -> Eigen::Vector3d { return xsensCoMpos_; }),
      mc_rtc::gui::Arrow(
          "DCM-VRP", VRPforceArrowConfig, [this]() -> const Eigen::Vector3d & { return combinedVRP_; },
          [this]() -> const Eigen::Vector3d & { return DCM_; })

  );
}

void DCM_VRPtracker::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeElements(this);
}

void DCM_VRPtracker::addLogEntries(std::string robotName, mc_rtc::Logger & logger)
{
  logger.addLogEntry(fmt::format("DCMtracker_{}_DCM", robotName), [this]() -> const Eigen::Vector3d & { return DCM_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_DCM objective", robotName),
                     [this]() -> const Eigen::Vector3d & { return DCMobjective_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_DCM error", robotName),
                     [this]() -> const Eigen::Vector3d & { return DCMerror_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_VRP error", robotName),
                     [this]() -> const Eigen::Vector3d & { return VRPerror_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_VRP acceleration model", robotName),
                     [this]() -> const Eigen::Vector3d & { return modelVRP_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_VRP forces model", robotName),
                     [this]() -> const Eigen::Vector3d & { return measuredForcesVRP_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_VRP combined", robotName),
                     [this]() -> const Eigen::Vector3d & { return combinedVRP_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_omega", robotName), [this]() -> const double & { return omega_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_desired VRP command", robotName),
                     [this]() -> const Eigen::Vector3d & { return commandVRP_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_applied forces sum at CoM", robotName),
                     [this]() -> const sva::ForceVecd & { return appliedForces_; });
  logger.addLogEntry(fmt::format("DCMtracker_{}_Missing forces", robotName),
                     [this]() -> const sva::ForceVecd & { return missingForces_; });
}

void DCM_VRPtracker::removeLogEntries(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}
