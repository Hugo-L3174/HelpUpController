#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <polytope/robustStabilityPolytope.h>
#include <polytope/stabilityPolytope.h>
#include <problemDescriptor/contactSet.h>
#include <thread>
#include <condition_variable>
#include <atomic>
#include "utils/PointProjector.h"

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
  MCStabilityPolytope(const std::string & name) : name_(name)
  { 
  thread_ = std::thread(&MCStabilityPolytope::compute, this);
  #ifndef WIN32
    // Lower thread priority so that it has a lesser priority than the real time
    // thread
    auto th_handle = thread_.native_handle();
    int policy = 0;
    sched_param param{};
    pthread_getschedparam(th_handle, &policy, &param);
    param.sched_priority = 10;
    if(pthread_setschedparam(th_handle, SCHED_RR, &param) != 0)
    {
      mc_rtc::log::warning("[{}] Failed to lower thread priority. If you are running on a real-time system, "
                           "this might cause latency to the real-time loop.",
                           name_);
    }
  #endif
  }

  ~MCStabilityPolytope()
  {
    stop();
  }

  void stop()
  {
    computing_ = false;
    thread_.join();
  }

  void load(const mc_rtc::Configuration & config)
  {
    config("precision", precision_);
  }

  void update(const std::shared_ptr<ContactSet> & contactSet, const Eigen::Vector3d & currentPos)
  {   
    std::lock_guard<std::mutex> lock(contactMutex_);
    contactSet_ = contactSet;
    currentPos_ = currentPos;
    // mc_rtc::log::warning("[{}] setContacts with size {}", name_, contactSet->numberOfContacts());
    if(!computing_)
    {
      computing_ = true;
      cv_.notify_one();
    }
  }

  /** @brief Returns the result of the polytope computation
   *
   * If you only need a specific element, prefer using the individual accessors (triangles, edges, projector)
   */
  PolytopeResult polytopeResult() 
  {
    /* std::lock_guard<std::mutex> lock(resultMutex_); */
    return polytopeResult_;
  }

  auto triangles() const
  {
    /* std::lock_guard<std::mutex> lock(resultMutex_); */
    return polytopeResult_.triangles;
  }

  auto edges() const
  {
    /* std::lock_guard<std::mutex> lock(resultMutex_); */
    return polytopeResult_.edges;
  }

  Eigen::Vector3d objective() const
  {
    /* std::lock_guard<std::mutex> lock(resultMutex_); */
    return polytopeResult_.objective;
  }

  bool computed() const noexcept
  {
    return computedFirst_; 
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {"Polytopes"})
  {
    mc_rtc::gui::PolyhedronConfig pconfig;
    pconfig.triangle_color = mc_rtc::gui::Color(0.2, 0.2, 0.2, 0.3);
    pconfig.use_triangle_color = true;
    pconfig.show_triangle = true;
    pconfig.show_vertices = false;
    pconfig.show_edges = false;
    pconfig.fixed_edge_color = true;
    pconfig.edge_config.color = mc_rtc::gui::Color::LightGray;
    pconfig.edge_config.width = 0.003;
    static bool publish_as_vertices_triangles = false;

    mc_rtc::gui::PolyhedronConfig pconfig_rob = pconfig;
    pconfig_rob.triangle_color = mc_rtc::gui::Color(1, 0, 0, 0.3);

    auto cat = category;
    category.push_back("Polyhedrons");
    gui.addElement(this,
        category,
        mc_rtc::gui::Polyhedron(fmt::format("{} balance region", name_), pconfig_rob,
                                [this]()
                                { 
                                  return triangles(); 
                                }));


    cat.push_back("Triangles");
    gui.addElement(this,
        cat,
        mc_rtc::gui::Polygon(fmt::format("{} balance region", name_), 
              mc_rtc::gui::Color::Red,
              [this]()
              { 
              return edges(); 
              }
        ));
  }

  void removeFromGUI(mc_rtc::gui::StateBuilder & gui)
  {
    gui.removeElements(this);
  }

protected:
  auto updateEdges() const
  {
    std::vector<std::vector<Eigen::Vector3d>> edges;
    std::vector<Eigen::Vector3d> triangle;
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

  void compute()
  {
    std::mutex mutex;
    std::unique_lock<std::mutex> lk(mutex);
    cv_.wait(lk, [this]() -> bool { return computing_; });

    Eigen::Vector3d currentPos;
    PolytopeResult result;
    while(computing_)
    {
      {
        std::lock_guard<std::mutex> lock(contactMutex_);
        computationPolytope_ = std::make_shared<RobustStabilityPolytope>(contactSet_, 20, precision_, ::GLPK);
        currentPos = currentPos_;
      }
      computationPolytope_->initSolver();
      bool ok = computationPolytope_->computeProjectionStabilityPolyhedron();
      computationPolytope_->endSolver();
      if(ok)
      {
        result.objective = objectiveInPolytope(currentPos_);
        result.constraintPlanes = computationPolytope_->constraintPlanes();
        result.triangles = computationPolytope_->triangles(); 
        result.edges = updateEdges();
        {
          std::lock_guard<std::mutex> lock(resultMutex_);
          polytopeResult_ = result;
        }
        computedFirst_ = true;
      }
      else
      {
        std::cout << "error: " << computationPolytope_->getError() << " num iter: " << computationPolytope_->getIteration() << std::endl;
      }
    }
  }

  /** @brief Project the point onto the polytope if it is outside
   *
   * When currentPos is outside of the polyhedron this is a heavy computation 
   *
  * @note There is room for optimization by avoiding converting to sch::S_Polyhedron in PointProjector
   */
  Eigen::Vector3d objectiveInPolytope(const Eigen::Vector3d & currentPos)
  {
    if(!isVertexInPlanes(currentPos, computationPolytope_->constraintPlanes(), 0.03))
    {
      // Heavy computation
      projector_.setPolytope(computationPolytope_);
      projector_.setPoint(currentPos);
      projector_.project();
      return projector_.projectedPoint();
    }
    else
    {
      return currentPos;
    }
  }

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
