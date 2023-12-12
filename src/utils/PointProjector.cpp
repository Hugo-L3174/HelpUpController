#include "PointProjector.h"
#include <sch/S_Polyhedron/Polyhedron_algorithms.h>
#include <strings.h>

PointProjector::PointProjector() : isSet_(false), polyhedron_(new sch::S_Polyhedron()) {}

void PointProjector::setPolytope(const std::shared_ptr<RobustStabilityPolytope> & poly)
{
  auto & polytope = *poly;
  polyhedron_->clear();
  auto & polyAlgo = *polyhedron_->getPolyhedronAlgorithm();

  /*
    First add the vertices:
    - get them from the poly object
    - create a S_PolyhedronVertex Object
    - add it the S_Polyhedron object using the right method
  */

  const auto & fullVertices = polytope.fullVertices();
  sch::S_PolyhedronVertex * v; // removed when polyhedron_ is destroyed
  Eigen::Vector3d coord;
  std::map<int, int> indexToPos;

  for(size_t i = 0; i < fullVertices.size(); ++i)
  {
    auto & vertex = *fullVertices[i];
    coord = vertex.get_coordinates();

    v = new sch::S_PolyhedronVertex();
    v->setCoordinates(coord[0], coord[1], coord[2]);
    indexToPos[vertex.get_index()] = i; // polyAlgo.vertexes_.size();
    polyAlgo.vertexes_.push_back(v);
  }

  /*
    Second add the normal/faces
    - Get the faces from the polytope object
    - create the corresponding PolyhedronTriangle Object
    - normal
    - corresponding vertices
    - add it to the S_Polyhedron

    Third add the neighboring vertices
    - Go through the edges of the polytope object
    - Or add them when creating the triangles
  */

  int a, b, c;
  sch::S_PolyhedronVertex *va, *vb, *vc;

  const auto & fullFaces = polytope.fullFaces();
  for(size_t i = 0; i < fullFaces.size(); ++i)
  {
    auto & face = *fullFaces[i];
    coord = face.get_normal();

    sch::PolyhedronTriangle triangle;
    triangle.a = indexToPos[face.get_vertex1()->get_index()];
    triangle.b = indexToPos[face.get_vertex2()->get_index()];
    triangle.c = indexToPos[face.get_vertex3()->get_index()];
    triangle.normal = sch::Vector3{coord[0], coord[1], coord[2]}.normalized();
    polyAlgo.triangles_.push_back(triangle);
  }

  polyAlgo.updateVertexNeighbors();
  /*
    Forth apply the finishing method
    - updateFastArrays
    - deleteVertexesWithoutNeighbors
  */

  polyAlgo.deleteVertexesWithoutNeighbors(); // XXX try not to do this, looks heavy
  polyAlgo.updateFastArrays();

  // polyhedron_.updateVertexNeighbors();
  // polyhedron_.updateFastArrays();
  // std::cout << "The polyhedron has " << polyhedron_.getTrianglesNumber() << " triangles" << std::endl;

  isSet_ = true;
}

void PointProjector::setPoint(const Eigen::Vector3d & point)
{
  point_.setPosition(point[0], point[1], point[2]);
}

void PointProjector::project()
{
  sch::CD_Pair pair_(polyhedron_.get(), &point_);
  // pair_.setRelativePrecision(1e-5);

  sch::Point3 p1, p2;
  pair_.getClosestPoints(p1, p2);

  projectedPoint_ << p1.m_x, p1.m_y, p1.m_z;
  // std::cout << "The projected point on the polytope is: " << projectedPoint_.transpose() << std::endl;
  // std::cout << "The second point is " << p2.m_x << " " <<  p2.m_y << " " <<  p2.m_z << std::endl;
}

void PointProjector::displaySqueleton()
{
  auto const polyAlgo = polyhedron_->getPolyhedronAlgorithm();

  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d point;
  std::vector<std::pair<int, int>> edges;

  for(auto vertex : polyAlgo->vertexes_)
  {
    point << vertex->getCoordinates().m_x, vertex->getCoordinates().m_y, vertex->getCoordinates().m_z;
    points.push_back(point);
  }

  auto pairEq = [](std::pair<int, int> p1, std::pair<int, int> p2)
  { return (p1.first == p2.first and p1.second == p2.second) or (p1.first == p2.second and p1.second == p2.first); };

  std::pair<int, int> eds[3];

  for(auto tri : polyAlgo->triangles_)
  {
    eds[0] = std::make_pair(tri.a, tri.b);
    eds[1] = std::make_pair(tri.a, tri.c);
    eds[2] = std::make_pair(tri.b, tri.c);

    for(auto e : eds)
    {
      if(std::find_if(edges.begin(), edges.end(), [e, pairEq](std::pair<int, int> edge) { return pairEq(edge, e); })
         == edges.end())
      {
        edges.push_back(e);
      }
    }
  }

  int cpt(0);

  std::cout << "List of vertexes: " << std::endl;

  for(auto pt : points)
  {
    std::cout << cpt << ": " << pt.transpose() << std::endl;
    cpt++;
  }

  std::cout << "\n List of edges: " << std::endl;

  for(auto e : edges)
  {
    std::cout << "    " << e.first << " " << e.second << std::endl;
  }
}
