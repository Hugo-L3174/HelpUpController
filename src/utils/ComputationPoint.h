#pragma once

#include <problemDescriptor/contactSet.h>
#include <polytope/robustStabilityPolytope.h>
#include <polytope/staticStabilityPolytope.h>
#include <polytope/stabilityPolytope.h>
// #include <polytope/constrainedEquilibriumPolytope.h>

#include <memory>
#include <functional> // C++ 11

#include "CoMQP.h"
#include "PointProjector.h"

#include <tinyxml2.h>

class ComputationPoint
{
 public:
  ComputationPoint(int index, std::shared_ptr<ContactSet> contactSet);

  // compute the equilibrium region
  /* \brief project the equilibrium region using the stabiliplus library
   *
   */
  void computeEquilibriumRegion();

  /*
   */
  void computeOptimalCoMPosition(Eigen::Vector3d currentCoM);
  
  // compute the different possible CoM...
  /* \brief compute the points that should be saved using lambdas
   * This function is used for debug/logging/illustration in parallel with the python postprocess
   */
  void computeLambdaPoints();
  
  // save the computation point in the same way as for the experimenter in stabiliplus
  /* \brief save the computation point 
   *
   */
  void save(std::string fileName);

  /* \brief Gives the vector of the triangles. Triangles are as 3 Vector3d for the vertices and 1 Vector3d for the normal
   */
  void updateTriangles();

  // edges (temporarybefore triangles)
  // void updateEdges();

  // ----- Setters -----

  // conservative polytope or pointprojector with the conservative polytope already inside.

  // lambda for saving special points
  void addLambda(std::string name, std::function<Eigen::Vector3d(ComputationPoint*)> computer, std::string color="xkcd:red");
  
  // ----- Getters -----

  inline bool computed() const
  {
    return computed_;
  }

  /* \brief returns the triangles vector for gui polytope
  */
  std::vector<std::vector<Eigen::Vector3d>> getTriangles() const
  {
    return triangles_;
  }

  // std::vector<Eigen::Vector3d> getEdges() const
  // {
  //   return edgesPoly_;
  // }
  
  // constrainedPlanes
  std::vector<Eigen::Vector4d> constraintPlanes() const;
  // objectiveCoM
  Eigen::Vector3d objectiveCoM(int mode=0, Eigen::Vector3d currentCoM=Eigen::Vector3d::Zero());

  Eigen::Vector3d optimalCoM(Eigen::Vector3d com = Eigen::Vector3d::Zero());
  Eigen::Vector3d projectedOptimalCoM(Eigen::Vector3d com = Eigen::Vector3d::Zero());

  /* Get the chebichev center of the polytope
   */
  Eigen::Vector3d chebichevCenter();

  double computeDistance(Eigen::Vector4d plane, Eigen::Vector3d pt) const;
  /* Compute the value of the potential at the given CoM position
   */
  double computePotential(Eigen::Vector3d CoM, double thres=1e-5) const;

  /* Compute the gradient of the potential at the given CoM position
   */
  Eigen::Vector3d computeGradient(Eigen::Vector3d CoM, double thres=1e-5) const;
  
  inline int index() const {return index_;}
  inline int computationTime() const { return computationTime_; }
  
 private:
  /* index of the computation point.
   * It should be unique but for now it is managed externally...
   */ 
  int index_;

  /* Contact Set corresponding to the computation point
   */ 
  std::shared_ptr<ContactSet> contactSet_;

  /* Polytope computed corresponding to the contact set
   *std::shared_ptr<StabilityPolytope> polytope_; switching to robust only
   */
  std::shared_ptr<RobustStabilityPolytope> polytope_;

  /* Triangles vector
   */
  std::vector<std::vector<Eigen::Vector3d>> triangles_;

  //   /* Edges vector
  //  */
  // std::vector<Eigen::Vector3d> edgesPoly_;

  /* CoMQP object used to compute the optimal position of the CoM 
   */
  std::shared_ptr<CoMQP> comQP_;
  
  /* PointProjector that can be used to project a point in the conservative area
   */
  std::shared_ptr<PointProjector> projector_;

  /* true if the stabilityPolytope has been computed, false otherwise
   */
  bool computed_;
  
  /* Computation time in µs
   */ 
  int computationTime_;

  /* \brief List of lambda function that compute points that will be saved
     each lambda function should take as argument a pointer to a computation point (I guess...)
   */
  std::map<std::string, std::function<Eigen::Vector3d(ComputationPoint*)>> computerPoints_; 
  std::map<std::string, Eigen::Vector3d> computedPoints_;
  std::map<std::string, std::string> computedPointsColor_;

  /* Store the chebichev center of the polytope
   */
  Eigen::Vector3d chebichevCenter_;
  /* True when the Chebichev center has been computed
   */
  bool chebichevComputed_;
};
