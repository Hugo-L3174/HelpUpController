#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

#include "api.h"

struct HelpUpController_DLLAPI HelpUpController : public mc_control::fsm::Controller
{
    HelpUpController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
    
    // Hands contact task
    std::shared_ptr<mc_tasks::EndEffectorTask> rightHandTask_;
    std::shared_ptr<mc_tasks::EndEffectorTask> leftHandTask_;

    // DynamicsConstraint for human model
    mc_solver::DynamicsConstraint humanDynamicsConstraint_;

};