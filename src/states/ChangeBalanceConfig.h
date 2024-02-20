#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>
using ContactState = mc_tasks::lipm_stabilizer::ContactState;

struct ChangeBalanceConfig : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  /*! For each of the contact points that are added, update fmax according to the current iteration
   * Only update the upper bound!
   * \param  multiplier number of step ahead the contactSet must be computed for.
   */
  // void updateContactForce(HelpUpController & ctl, int multiplier=2);
private:
  mc_rtc::Configuration config_;
  mc_rtc::Configuration StabilizerConfig_;

  bool hasCompletion_ = false; /**< If the latest definition of the state configuration has an empty "completion"
                                 element, no completion rule will be used */

  Eigen::Vector3d dcmThreshold_ = Eigen::Vector3d{0.01, 0.01, 0.05}; /**< Completion criteria threshold */

  std::vector<ContactState> contactState_ = {ContactState::Left, ContactState::Right};

  std::map<std::string, double> newFMax_;
  std::map<std::string, double> newFMin_;
  double Ftarget_ = 0.0;
  double Finit_ = 0.0;

  /*! \brief When ratio reaches 1.0 the state should finnish.
   */
  double ratio_;

  // time attributes, working in ms
  std::chrono::milliseconds desiredDuration_;
  std::chrono::milliseconds intervals_;
  std::chrono::milliseconds currentDuration_;
  std::chrono::milliseconds previousStep_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};
