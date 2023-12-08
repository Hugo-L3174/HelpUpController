#include "MCStabilityPolytope.h"


MCStabilityPolytope::MCStabilityPolytope(const std::string & name) : name_(name)
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

MCStabilityPolytope::~MCStabilityPolytope()
{
  stop();
}

void MCStabilityPolytope::load(const mc_rtc::Configuration & config)
{
  config("precision", precision_);
}

void MCStabilityPolytope::update(const std::shared_ptr<ContactSet> & contactSet, const Eigen::Vector3d & currentPos)
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

void MCStabilityPolytope::addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
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
    gui.addElement(this, category,
                   mc_rtc::gui::Polyhedron(fmt::format("{} balance region", name_), pconfig_rob,
                                           [this]() { return triangles(); }));

    cat.push_back("Triangles");
    gui.addElement(this, cat,
                   mc_rtc::gui::Polygon(fmt::format("{} balance region", name_), mc_rtc::gui::Color::Red,
                                        [this]() { return edges(); }));
}

  void MCStabilityPolytope::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
  {
    gui.removeElements(this);
  }

void MCStabilityPolytope::compute()
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
        mc_rtc::log::error("[{}] error: {} num iter: {}", name_, computationPolytope_->getError(),
                     computationPolytope_->getIteration());
      }
    }
}

  Eigen::Vector3d MCStabilityPolytope::objectiveInPolytope(const Eigen::Vector3d & currentPos)
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

