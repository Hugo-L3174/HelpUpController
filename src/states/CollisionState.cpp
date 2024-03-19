#include "CollisionState.h"

#include <mc_control/fsm/Controller.h>

void CollisionState::addRemoveCollisions(mc_control::fsm::Controller & ctl, bool add = true, bool before = true)
{
  const auto & stateConfig = config_;
  const auto & ctlConfig = ctl.config();
  std::string name = fmt::format("{}CollisionSets{}", add ? "Add" : "Remove", before ? "Before" : "After");
  if(auto addCollisionSets = stateConfig.find(name))
  {
    if(addCollisionSets->size())
    {
      // value_or combined with the find chains the calls to find nested objects in the config
      if(auto robotCollisionSets =
             ctlConfig.find("collision_sets").value_or(mc_rtc::Configuration{}).find(ctl.robot().name()))
      {
        for(const auto & setName : *addCollisionSets)
        {
          if(auto collisionSet = robotCollisionSets->find(setName))
          {
            auto colSet = *collisionSet;
            auto r1 = colSet("r1", ctl.robot().name());
            auto r2 = colSet("r2", ctl.robot().name());
            auto collisions = colSet("collisions", std::vector<mc_rbdyn::Collision>{});
            if(add)
            {
              mc_rtc::log::info("Adding collision set {} for robot {}", setName, ctl.robot().name());
              ctl.addCollisions(r1, r2, collisions);
            }
            else
            {
              mc_rtc::log::info("Removing collision set {} for robot {}", setName, ctl.robot().name());
              ctl.removeCollisions(r1, r2, collisions);
            }
          }
        }
      }
    }
  }
}

void CollisionState::start(mc_control::fsm::Controller & ctl)
{

  addRemoveCollisions(ctl, true, true);
  addRemoveCollisions(ctl, false, true);
}

bool CollisionState::run(mc_control::fsm::Controller & ctl)
{
  output("OK");
  return true;
}

void CollisionState::teardown(mc_control::fsm::Controller & ctl)
{
  addRemoveCollisions(ctl, true, false);
  addRemoveCollisions(ctl, false, false);
}

EXPORT_SINGLE_STATE("CollisionState", CollisionState)
