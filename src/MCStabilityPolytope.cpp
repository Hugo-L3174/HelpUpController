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
  if(auto gui = config.find("polyhedron"))
  {
    guiConfig_.fromConfig(*gui);
  }
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
  auto cat = category;
  category.push_back("Polyhedrons");
  gui.addElement(
      this, category,
      mc_rtc::gui::Polyhedron(fmt::format("{} balance region", name_), guiConfig_, [this]() { return triangles(); }));

  auto triangle_color = guiConfig_.triangle_color;
  triangle_color.a = 1;
  cat.push_back("Triangles");
  gui.addElement(
      this, cat,
      mc_rtc::gui::Polygon(fmt::format("{} balance region", name_), triangle_color, [this]() { return edges(); }));
}

void MCStabilityPolytope::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeElements(this);
}

void MCStabilityPolytope::addToLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  auto p = prefix + "_" + name_;
  logger.addLogEntry(p + "_dt_build_polytope", this, [this]() { return dt_build_polytope().count(); });
  logger.addLogEntry(p + "_dt_init_solver", this, [this]() { return dt_init_solver().count(); });
  logger.addLogEntry(p + "_dt_compute_stability_polyhedron", this,
                     [this]() { return dt_compute_stability_polyhedron().count(); });
  logger.addLogEntry(p + "_dt_end_solver", this, [this]() { return dt_end_solver().count(); });
  logger.addLogEntry(p + "_dt_project_in_polytope", this, [this]() { return dt_project_in_polytope().count(); });
  logger.addLogEntry(p + "_dt_swap_result", this, [this]() { return dt_swap_result().count(); });
  logger.addLogEntry(p + "_objectiveInPolytope", this, [this]() { return objectiveInPolytope_; });
  logger.addLogEntry(p + "_dt_stabiliplus_lp", this, [this]() { return dt_stabiliplus_lp().count(); });
  logger.addLogEntry(p + "_dt_stabiliplus_init", this, [this]() { return dt_stabiliplus_init().count(); });
  logger.addLogEntry(p + "_dt_stabiliplus_struct", this, [this]() { return dt_stabiliplus_struct().count(); });
}

void MCStabilityPolytope::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
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
    auto start_loop_time = mc_rtc::clock::now();
    auto start_build_polytope = mc_rtc::clock::now();
    {
      std::lock_guard<std::mutex> lock(contactMutex_);
      computationPolytope_ = std::make_shared<RobustStabilityPolytope>(contactSet_, 20, precision_, ::GLPK);
      currentPos = currentPos_;
    }
    result.dt_build_polytope = mc_rtc::clock::now() - start_build_polytope;
    auto start_init_solver = mc_rtc::clock::now();
    computationPolytope_->initSolver();
    result.dt_init_solver = mc_rtc::clock::now() - start_init_solver;
    auto start_compute_stability_polyhedron = mc_rtc::clock::now();
    computePolyhedronSuccess_ = computationPolytope_->computeProjectionStabilityPolyhedron();
    result.dt_compute_stability_polyhedron = mc_rtc::clock::now() - start_compute_stability_polyhedron;
    auto start_end_solver = mc_rtc::clock::now();
    computationPolytope_->endSolver();
    result.dt_end_solver = mc_rtc::clock::now() - start_end_solver;
    if(computePolyhedronSuccess_)
    {
      result.constraintPlanes = computationPolytope_->constraintPlanes();
      result.triangles = computationPolytope_->triangles();
      result.edges = updateEdges();
      result.polytope = computationPolytope_;
      result.chebichevCenter = computationPolytope_->chebichevCenter();
      result.baryCenter = computationPolytope_->baryPoint();
      result.dt_stabiliplus_lp = mc_rtc::duration_ms(computationPolytope_->LPTime() / 1000);
      result.dt_stabiliplus_init = mc_rtc::duration_ms(computationPolytope_->initTime() / 1000);
      result.dt_stabiliplus_struct = mc_rtc::duration_ms(computationPolytope_->structTime() / 1000);
      {
        auto start_swap_result = mc_rtc::clock::now();
        std::lock_guard<std::mutex> lock(resultMutex_);
        result.dt_swap_result = mc_rtc::clock::now() - start_swap_result;
        result.dt_loop_total = mc_rtc::clock::now() - start_loop_time;
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
  auto start_project_in_polytope = mc_rtc::clock::now();
  auto objective = Eigen::Vector3d::Zero().eval();
  if(!isVertexInPlanes(currentPos, constraintPlanes(), 0.03))
  {
    objectiveInPolytope_ = false;
    std::shared_ptr<RobustStabilityPolytope> polytope;
    {
      std::lock_guard<std::mutex> lock(resultMutex_);
      polytope = polytopeResult_.polytope;
    }
    projector_.setPolytope(polytope);
    projector_.setPoint(currentPos);
    projector_.project();
    objective = projector_.projectedPoint();
  }
  else
  {
    objectiveInPolytope_ = true;
    objective = currentPos;
  }
  dt_project_in_polytope_ = mc_rtc::clock::now() - start_project_in_polytope;
  return objective;
}
