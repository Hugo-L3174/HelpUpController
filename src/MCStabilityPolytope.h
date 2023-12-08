#pragma once

#include "utils/PointProjector.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <polytope/robustStabilityPolytope.h>
#include <polytope/stabilityPolytope.h>
#include <problemDescriptor/contactSet.h>
#include <thread>

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui.h>

struct PolytopeResult
{
  std::vector<std::array<Eigen::Vector3d, 3>> triangles;
  std::vector<std::vector<Eigen::Vector3d>> edges;
  std::vector<Eigen::Vector4d> constraintPlanes;
  Eigen::Vector3d objective;
};

struct MCStabilityPolytope
{
  MCStabilityPolytope(const std::string & name);
  ~MCStabilityPolytope();

  inline void stop()
  {
    computing_ = false;
    thread_.join();
  }

  void load(const mc_rtc::Configuration & config);

  void update(const std::shared_ptr<ContactSet> & contactSet, const Eigen::Vector3d & currentPos);

  /** @brief Returns the result of the polytope computation
   *
   * If you only need a specific element, prefer using the individual accessors (triangles, edges, projector)
   */
  inline PolytopeResult polytopeResult()
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_;
  }

  inline auto triangles() const
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.triangles;
  }

  inline auto edges() const
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.edges;
  }

  inline Eigen::Vector3d objective() const
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.objective;
  }

  inline bool computed() const noexcept
  {
    return computedFirst_;
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {"Polytopes"});
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

protected:
  auto updateEdges() const
  {
    std::vector<std::vector<Eigen::Vector3d>> edges;
    edges.reserve(computationPolytope_->fullFaces().size());
    std::vector<Eigen::Vector3d> triangle;
    triangle.reserve(computationPolytope_->fullFaces().size() * 3);
    for(const auto & face : computationPolytope_->fullFaces())
    {
      triangle.clear();
      triangle.push_back(face->get_vertex1()->get_coordinates());
      triangle.push_back(face->get_vertex2()->get_coordinates());
      triangle.push_back(face->get_vertex3()->get_coordinates());
      edges.push_back(triangle);
    }
    return edges;
  }

  /** Thread for polytope computations */
  void compute();

  /** @brief Project the point onto the polytope if it is outside
   *
   * When currentPos is outside of the polyhedron this is a heavy computation
   *
   * @note There is room for optimization by avoiding converting to sch::S_Polyhedron in PointProjector
   */
  Eigen::Vector3d objectiveInPolytope(const Eigen::Vector3d & currentPos);

protected:
  std::string name_;

  //{ Thread internals
  std::thread thread_;
  std::condition_variable cv_;
  std::atomic<bool> computing_{false};
  std::atomic<bool> computedFirst_{false};
  std::shared_ptr<RobustStabilityPolytope> computationPolytope_ = nullptr;
  PointProjector projector_;
  //}

  //{ Thread inputs
  std::mutex contactMutex_;
  std::shared_ptr<ContactSet> contactSet_;
  Eigen::Vector3d currentPos_;
  //}

  //{ Thread outputs
  mutable std::mutex resultMutex_;
  //< Result of the polytope computation that may be of use to other modules
  // This should be returned by copy and protected by resultMutex_
  PolytopeResult polytopeResult_;

  //{ Configuration
  // TODO load from config
  double precision_ = 0.1;
  // Load config for GUI
  //}
};
