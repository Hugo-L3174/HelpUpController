#include "ComputationPoint.h"

ComputationPoint::ComputationPoint(int index, std::shared_ptr<ContactSet> contactSet): index_(index), contactSet_(contactSet),
										       computed_(false), computationTime_(0),
										       chebichevComputed_(false)
{
  projector_ = std::make_shared<PointProjector> ();
  comQP_ = std::make_shared<CoMQP> (contactSet_);
}

void ComputationPoint::computeEquilibriumRegion()
{
  auto start = std::chrono::high_resolution_clock::now();

  double precision = 0.1;
  try
  {  
    // polytope_ = std::make_shared<StaticStabilityPolytope> (contactSet_, 20, 0.01, GLPK);
    polytope_ = std::make_shared<RobustStabilityPolytope> (contactSet_, 20, precision, GLPK);
    polytope_->initSolver();
    polytope_->projectionStabilityPolyhedron();
    polytope_->endSolver();
    updateTriangles();
  }
  catch (std::runtime_error e)
  {
    std::cout << "error: " << polytope_->getError() << " num iter: " << polytope_->getIteration()<< std::endl;
    // contactSet_->showContactSet();
    contactSet_->saveContactSet("/tmp/stabiliplus_fail_contactSet.xml");
    std::cout << "Trying again!" << std::endl;

    // polytope_ = std::make_shared<StaticStabilityPolytope> (contactSet_, 20, 0.01, GLPK);
    polytope_ = std::make_shared<RobustStabilityPolytope> (contactSet_, 20, precision, GLPK);
    polytope_->initSolver();
    polytope_->projectionStabilityPolyhedron();
    polytope_->endSolver();
    updateTriangles();
    // updateEdges();
    // throw e;
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds> (end-start);

  computed_ = true;
  computationTime_ = duration.count();

}

void ComputationPoint::updateTriangles()
{
  triangles_.clear();
  Eigen::Vector3d coord1, coord2, coord3, normal;
  std::vector<Eigen::Vector3d> triangle;
  for (auto face:polytope_->fullFaces())
  {
    triangle.clear();
    triangle.push_back(face->get_vertex1()->get_coordinates());
    triangle.push_back(face->get_vertex2()->get_coordinates());
    triangle.push_back(face->get_vertex3()->get_coordinates());
    // triangle.push_back(face->get_normal());

    triangles_.push_back(triangle);
  }

}

// void ComputationPoint::updateEdges()
// {
//   edgesPoly_.clear();
//   Eigen::Vector3d edge1, edge2, edge3;
//   for (auto face:polytope_->fullFaces())
//   {
//     edge1 = face->get_edge1()->get_vertex1()->get_coordinates() - face->get_edge1()->get_vertex2()->get_coordinates();
//     edge2 = face->get_edge2()->get_vertex1()->get_coordinates() - face->get_edge2()->get_vertex2()->get_coordinates();
//     edge3 = face->get_edge3()->get_vertex1()->get_coordinates() - face->get_edge3()->get_vertex2()->get_coordinates();  

//     // avoid duplucate edges
//     for (auto edge : edgesPoly_)
//     {
      
//     }
    

//     edgesPoly_.push_back(edge1);
//     edgesPoly_.push_back(edge2);
//     edgesPoly_.push_back(edge3);
//   }

// }

void ComputationPoint::computeOptimalCoMPosition(Eigen::Vector3d currentCoM)
{
  bool ret = comQP_->solve(currentCoM);

  if (!ret) 
    {
      std::cout << "Failed to solve the CoMQP" << std::endl;
      std::cout << "Error code is: " << comQP_->errorCode() << std::endl;
      throw 42;
    }
}

void ComputationPoint::computeLambdaPoints()
{
  for (auto& cptPt: computerPoints_)
    {
      computedPoints_[cptPt.first] = cptPt.second(this);
    }
}

void ComputationPoint::save(std::string fileName)
{
  // for now
  computeLambdaPoints();
  
  std::string suffix = "";
  // auto start = std::chrono::high_resolution_clock::now();

  
  system("mkdir -p /tmp/contactSets/");
  system("mkdir -p /tmp/polytopes/");
  
  std::string contactSetFileName = "/tmp/contactSets/contactSet_" + std::to_string(index_) + suffix + ".xml";
  contactSet_->saveContactSet(contactSetFileName);
  
  std::string polytopeFileName = "/tmp/polytopes/poly_" + std::to_string(index_) + suffix + ".xml";
  polytope_->saveToFile(polytopeFileName);

  // creating the ComputationPoint XMLElement
  tinyxml2::XMLDocument doc;
  auto declaration = doc.NewDeclaration();
  doc.InsertEndChild(declaration);
  
  tinyxml2::XMLElement * xmlComputationPoint = doc.NewElement("compPoint");
  xmlComputationPoint->SetAttribute("index", index_);

  tinyxml2::XMLElement * polyXML = doc.NewElement("poly");
  polyXML->SetAttribute("file_name", polytopeFileName.c_str());
  xmlComputationPoint->InsertEndChild(polyXML);

  tinyxml2::XMLElement * robotXML = doc.NewElement("robot");

  robotXML->SetAttribute("name", contactSet_->get_name().c_str());
  robotXML->SetAttribute("file_name", contactSetFileName.c_str());
  xmlComputationPoint->InsertEndChild(robotXML);

  tinyxml2::XMLElement * timeXML = doc.NewElement("times");
  timeXML->SetAttribute("total", computationTime_);
  timeXML->SetAttribute("LP", polytope_->LPTime());
  timeXML->SetAttribute("init", polytope_->initTime());
  timeXML->SetAttribute("struct", polytope_->structTime());
  xmlComputationPoint->InsertEndChild(timeXML);

  auto addPoint = [&doc, &xmlComputationPoint](Eigen::Vector3d coord, std::string name, std::string color){
    auto ptXML = doc.NewElement("point");
    ptXML->SetAttribute("name", name.c_str());

    ptXML->SetAttribute("x", coord[0]);
    ptXML->SetAttribute("y", coord[1]);
    ptXML->SetAttribute("z", coord[2]);

    ptXML->SetAttribute("color", color.c_str());

    xmlComputationPoint->InsertEndChild(ptXML);
  };

  Eigen::Vector3d point;

  for (auto& cptPt: computedPoints_)
    {
      addPoint(cptPt.second, cptPt.first, computedPointsColor_.at(cptPt.first));
    }

  doc.InsertEndChild(xmlComputationPoint);
  
  // create the folder for the computation points
  system("mkdir -p /tmp/computationPoints/");
  
  std::string computationPointFileName;
  computationPointFileName = "/tmp/computationPoints/computationPoint_" + std::to_string(index_) + suffix + ".xml";
  // save the computation point xml
  doc.SaveFile(computationPointFileName.c_str());
}

// ----- Setters -----

void ComputationPoint::addLambda(std::string name, std::function<Eigen::Vector3d(ComputationPoint*)> computer, std::string color)
{
  computerPoints_[name]=computer;
  computedPointsColor_[name]=color;
}

// ----- Getters -----
std::vector<Eigen::Vector4d> ComputationPoint::constraintPlanes() const
{
  std::vector<Eigen::Vector4d> planes;
  if (computed_)
  {
    planes = polytope_->constraintPlanes();
    
    // cheaty part ^^
    // Eigen::Vector4d zPlane1, zPlane2;
    // zPlane1 << 0, 0, 1, 0.85;
    // zPlane2 << 0, 0, -1, -0.65;
    // planes.push_back(zPlane1);
    // planes.push_back(zPlane2);
  }
  return planes;
}

Eigen::Vector3d ComputationPoint::objectiveCoM(int mode, Eigen::Vector3d currentCoM)
{
  Eigen::Vector3d com;
  Eigen::Vector3d optimCoM, projectedCoM;
  double alpha(0.5); // 1 -> com = projectedCoM, 0 -> com = optimCoM
  // std::cout << std::endl;
  // contactSet_->showContactSet();
  // std::cout << "Computing Objective CoM, mode: " << mode  << std::endl;
  switch (mode)
    {
    case 0: // bary point
      com = polytope_->baryPoint();
      // com[2] = 0.74;
      com[2] = currentCoM[2];
      break;
    case 1: // chebichev
      com = chebichevCenter();
      com[2] = currentCoM[2];
      break;
    case 2: // CoMQp
      com = optimalCoM(currentCoM);
      break;

    case 3: // projected
      com = projectedOptimalCoM(currentCoM);
      break;

    case 4:
      optimCoM = optimalCoM(currentCoM);
      projectedCoM = projectedOptimalCoM(currentCoM);
      com = alpha * projectedCoM + (1-alpha) * optimCoM;
      break;  

    case 5:     
      com = optimalCoM(currentCoM);
      break;
      
    default :
      com = currentCoM;
      break;
    }
  //std::cout << "Objective CoM computed: " << com.transpose() << std::endl;
  return com;
}

Eigen::Vector3d ComputationPoint::optimalCoM(Eigen::Vector3d comTarget)
{
  computeOptimalCoMPosition(comTarget);
  return comQP_->resultCoM();
}

Eigen::Vector3d ComputationPoint::projectedOptimalCoM(Eigen::Vector3d comTarget)
{
  auto com = optimalCoM(comTarget);
  if (projector_->isSet())
    	{
    	  projector_->setPoint(com);
    	  projector_->project();
    	  if (!projector_->isInside())
    	    {
    	      com = projector_->projectedPoint();
    	    }
    	}
      else
    	{
    	  std::cout << "Projector not set" << std::endl;
    	}
  return com;
}

Eigen::Vector3d ComputationPoint::chebichevCenter()
{
  if (!chebichevComputed_)
    {
      chebichevCenter_ = polytope_->chebichevCenter();
      chebichevComputed_ = true;
    }
  return chebichevCenter_;
}

double ComputationPoint::computeDistance(Eigen::Vector4d plane, Eigen::Vector3d pt) const
{
  Eigen::Vector3d dir = plane.head(3);
  double dist = dir.dot(pt) - plane[3];
  return dist;
}

double ComputationPoint::computePotential(Eigen::Vector3d CoM, double thres) const
{
  auto planes = constraintPlanes();

  double pot = 0.0;
  for (auto plane: planes)
    {
      auto dist = computeDistance(plane, CoM);
      if (dist <= -thres)
    	{
    	  pot += log(1/(-dist));
    	}
      else
    	{
    	  pot += log(1/thres);
    	}
    }
  return pot;
}

Eigen::Vector3d ComputationPoint::computeGradient(Eigen::Vector3d CoM, double thres) const
{
  auto planes = constraintPlanes();

  Eigen::Vector3d grad = Eigen::Vector3d::Zero(3,1);
  // sum
  // std::cout << "Point: " << pt.transpose() << std::endl;xs
  for (auto plane: planes)
    {
      double dist = computeDistance(plane, CoM);
      if (dist <= -thres)
    	{
    	  grad -= plane.head(3) / (dist);
    	}
      else
    	{
    	  grad -= plane.head(3) / (thres);
    	}
    }
  // std::cout << "Grad: " << grad.transpose() << " (minDist: "<< minDist << ")" << std::endl;
  return grad;
}
