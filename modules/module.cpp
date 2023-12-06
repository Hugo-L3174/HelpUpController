#include "module.h"

#include "config.h"

#include <RBDyn/parsers/urdf.h>
#include <sch/S_Object/S_Box.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

PandaHelpUpRobotModule::PandaHelpUpRobotModule() : mc_robots::PandaRobotModule(false, false, false)
{
  // XXX dummy box data
  auto data = std::string(R"(
---
# DUMMY DATA, 50 grams solid cylinder, length 18 cm, radius 1cm
# Mass (in kilograms)
mass: 0.05
# Center of mass (in meters)
com: [0.0, 0.0, 0.09]
# Inertia (kg.meters²) [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
inertia: [0.00013625, 0.0, 0.0, 0.00013625, 0.0, 0.00000042]
  )");
  mc_rtc::Configuration inertiaC;
  inertiaC.loadYAMLData(data);
  mc_rtc::log::info("inertiaC: {}", inertiaC.dump(true, true));
  double mass = inertiaC("mass");
  Eigen::Vector3d com = inertiaC("com");
  Eigen::Vector6d inertiaV = inertiaC("inertia");
  Eigen::Matrix3d inertiaM;
  // clang-format off
  inertiaM << inertiaV(0), inertiaV(1), inertiaV(2),
                      0.0, inertiaV(3), inertiaV(4),
                      0.0,         0.0, inertiaV(5);
  // clang-format on
  inertiaM = inertiaM.selfadjointView<Eigen::Upper>();
  { // torso_box
    auto name = std::string{"torso_box"};
    rbd::parsers::Geometry::Box box;
    box.size = Eigen::Vector3d{0.137, 0.385, 0.42};
    rbd::parsers::Geometry geom;
    geom.data = box;
    geom.type = rbd::parsers::Geometry::BOX;
    _visual[name] = {{name, sva::PTransformd::Identity(), geom, {}}};
    _collisionObjects[name] = {name, std::make_shared<sch::S_Box>(box.size.x(), box.size.y(), box.size.z())};
    _collisionTransforms[name] = sva::PTransformd::Identity();

    mbg.addBody({mass, com, inertiaM, name});
    mbg.addJoint({rbd::Joint::Type::Fixed, true, "link8_to_box"});
    mbg.linkBodies("panda_link8", sva::PTransformd(Eigen::Vector3d{0, 0, box.size.z() / 2}), name,
                   sva::PTransformd::Identity(), "link8_to_box");
  }

  { // base box
    auto name = "base_box";
    rbd::parsers::Geometry::Box box;
    box.size = Eigen::Vector3d{0.75, 1.0, 0.75};
    rbd::parsers::Geometry geom;
    geom.data = box;
    geom.type = rbd::parsers::Geometry::BOX;
    _visual[name] = {{name, sva::PTransformd::Identity(), geom, {}}};
    _collisionObjects[name] = {name, std::make_shared<sch::S_Box>(box.size.x(), box.size.y(), box.size.z())};
    _collisionTransforms[name] = sva::PTransformd::Identity();

    mbg.addBody({mass, com, inertiaM, name});
    mbg.addJoint({rbd::Joint::Type::Fixed, true, "link0_to_base_box"});
    mbg.linkBodies("panda_link0", sva::PTransformd(Eigen::Vector3d{-(box.size.x() / 2 - 0.27), 0, -box.size.z() / 2}),
                   name, sva::PTransformd::Identity(), "link0_to_base_box");
    _default_attitude = {1, 0, 0, 0, 0, 0, box.size.z()};
  }

  double i = 0.02;
  double s = 0.01;
  double d = 0;
  _minimalSelfCollisions.emplace_back("convex_panda_link5", "torso_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link4", "torso_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link3", "torso_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link2", "torso_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link1", "torso_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link0", "torso_box", i, s, d);
  _minimalSelfCollisions.emplace_back("torso_box", "base_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link5", "base_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link6", "base_box", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link7", "base_box", i, s, d);

  mb = mbg.makeMultiBody(mb.body(0).name(), true);
  mbc = rbd::MultiBodyConfig(mb);
  mbc.zero(mb);

  auto urdf_path = bfs::temp_directory_path() / ("PandaHelpUp.urdf");
  {
    rbd::parsers::Limits limits;
    limits.lower = _bounds[0];
    limits.upper = _bounds[1];
    limits.velocity = _bounds[3];
    limits.torque = _bounds[5];
    std::ofstream ofs(urdf_path.string());
    ofs << rbd::parsers::to_urdf({mb, mbc, mbg, limits, _visual, _collision, "PandaHelpUp"});
  }
  this->urdf_path = urdf_path.string();
  this->calib_dir = panda_prosthesis::calib_DIR;
  this->name = "panda_default";
  mc_rtc::log::info("Wrote URDF to {}", urdf_path.string());
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"PandaHelpUp"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaHelpUp")
    if(n == "PandaHelpUp")
    {
      return new mc_robots::PandaHelpUpRobotModule();
    }
    else
    {
      mc_rtc::log::error("PandaHelpUp module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
