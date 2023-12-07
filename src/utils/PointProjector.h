#pragma once

#include <memory>
#include <utility>
// #include <math.h>

// stabiliplus
#include <polytope/robustStabilityPolytope.h>

// sch
#include <sch-core/S_Point.h>
#include <sch-core/S_Polyhedron.h>
#include <sch/CD/CD_Pair.h>

inline bool isVertexInPlanes(const Eigen::Vector3d & Vertex, const std::vector<Eigen::Vector4d> & planes, double eps)
{
  bool isInside = true;
  Eigen::Vector3d normal;
  double offset;
  for(auto plane : planes)
  {
    normal << plane(0), plane(1), plane(2);
    offset = plane(3);
    if(normal.transpose() * Vertex > offset - eps)
    {
      isInside = false;
      break;
    }
  }
  return isInside;
}

class PointProjector
{
public:
  PointProjector();

  void setPolytope(std::shared_ptr<RobustStabilityPolytope> poly);
  void setPoint(Eigen::Vector3d point);
  void project();

  inline Eigen::Vector3d projectedPoint() const
  {
    return projectedPoint_;
  }

  inline double distance() const
  {
    return distance_;
  }

  inline bool isInside() const
  {
    return isInside_;
  }

  inline bool isSet() const
  {
    return isSet_;
  }

  void displaySqueleton();

private:
  std::shared_ptr<RobustStabilityPolytope> polytope_;
  sch::S_Polyhedron polyhedron_;
  sch::S_Point point_;

  double distance_;
  Eigen::Vector3d projectedPoint_;
  bool isInside_;
  bool isSet_;
};
