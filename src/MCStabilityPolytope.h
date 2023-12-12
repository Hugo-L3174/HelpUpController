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
#include <mc_rtc/clock.h>
#include <mc_rtc/gui.h>

struct PolytopeResult
{
  std::vector<std::array<Eigen::Vector3d, 3>> triangles;
  std::vector<std::vector<Eigen::Vector3d>> edges;
  std::vector<Eigen::Vector4d> constraintPlanes;
  std::shared_ptr<RobustStabilityPolytope> polytope;

  mc_rtc::duration_ms dt_loop_total;
  mc_rtc::duration_ms dt_build_polytope;
  mc_rtc::duration_ms dt_init_solver;
  mc_rtc::duration_ms dt_compute_stability_polyhedron;
  mc_rtc::duration_ms dt_end_solver;
  mc_rtc::duration_ms dt_swap_result;
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

  inline auto constraintPlanes() const
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.constraintPlanes;
  }

  inline bool computed() const noexcept
  {
    return computedFirst_;
  }

  inline mc_rtc::duration_ms dt_build_polytope() const noexcept
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.dt_build_polytope;
  }

  inline mc_rtc::duration_ms dt_init_solver() const noexcept
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.dt_init_solver;
  }

  inline mc_rtc::duration_ms dt_compute_stability_polyhedron() const noexcept
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.dt_compute_stability_polyhedron;
  }

  inline mc_rtc::duration_ms dt_end_solver() const noexcept
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.dt_end_solver;
  }

  inline mc_rtc::duration_ms dt_project_in_polytope() const noexcept
  {
    return dt_project_in_polytope_;
  }

  inline mc_rtc::duration_ms dt_swap_result() const noexcept
  {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return polytopeResult_.dt_swap_result;
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {"Polytopes"});
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix = "Polytopes");
  void removeFromLogger(mc_rtc::Logger & logger);

  /** @brief Project the point onto the polytope if it is outside
   *
   * When currentPos is outside of the polyhedron this is a heavy computation
   *
   * @note There is room for optimization by avoiding converting to sch::S_Polyhedron in PointProjector
   */
  Eigen::Vector3d objectiveInPolytope(const Eigen::Vector3d & currentPos);

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

protected:
  std::string name_;

  //{ Thread internals
  std::thread thread_;
  std::condition_variable cv_;
  std::atomic<bool> computing_{false};
  std::atomic<bool> computedFirst_{false};
  std::atomic<bool> computePolyhedronSuccess_{false};
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
  mc_rtc::gui::PolyhedronConfig guiConfig_;
  //}

  mc_rtc::duration_ms dt_project_in_polytope_;
  bool objectiveInPolytope_ = false;
};
