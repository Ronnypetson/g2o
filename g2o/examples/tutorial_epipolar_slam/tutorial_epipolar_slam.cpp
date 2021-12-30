// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <cmath>

#include "simulator.h"

#include "vertex_epipolar_se3.h"
#include "edge_epipolar_se3.h"
#include "types_tutorial_epipolar_slam.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;


void write_output(
  string filename,
  vector<VertexEpipolarSE3*>& vertices,
  vector<EdgeEpipolarSE3*>& edges){
  // write output
  ofstream fileOutputStream;
  cerr << "Writing into " << filename << endl; // "epipolar_SE3.g2o"
  fileOutputStream.open(filename); // .c_str()

  string vertexTag = "VERTEX_SE3:QUAT"; // Factory::instance()->tag(vertices[0]) + ":QUAT"; //
  string edgeTag = "EDGE_SE3:QUAT"; // Factory::instance()->tag(edges[0]) + ":QUAT"; // 

  ostream& fout = fileOutputStream;
  for (size_t i = 0; i < vertices.size(); ++i) {
    VertexEpipolarSE3* v = vertices[i];
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
  }

  for (size_t i = 0; i < edges.size(); ++i) {
    EdgeEpipolarSE3* e = edges[i];
    VertexEpipolarSE3* from = static_cast<VertexEpipolarSE3*>(e->vertex(0));
    VertexEpipolarSE3* to = static_cast<VertexEpipolarSE3*>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << "0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000";
    fout << endl;
  }
}


void save_scene(
  string pose_filename,
  string landmark_filename,
  vector<VertexEpipolarSE3*>& vertices,
  vector<Eigen::Vector3d>& landmarks){

  ofstream fileOutputStream;
  fileOutputStream.open(pose_filename);
  ostream& fout = fileOutputStream;
  for (size_t i = 0; i < vertices.size(); ++i) {
    VertexEpipolarSE3* v = vertices[i];
    fout << v->estimate().toMatrix() << "\n\n";
  }

  ofstream lm_fileOutputStream;
  lm_fileOutputStream.open(landmark_filename);
  ostream& lm_fout = lm_fileOutputStream;
  for (size_t i = 0; i < landmarks.size(); ++i) {
    lm_fout << landmarks[i](0) << " "
            << landmarks[i](1) << " "
            << landmarks[i](2) << " "
            << 1.0 << "\n";
  }
}


int main()
{
  // TODO simulate different sensor offset
  // simulate a robot observing landmarks while travelling on a grid
  SE3 sensorOffsetTransf(0.0, 0.0, 0.0, -0.0, -0.0, -0.0);
  int numNodes = 5;
  Simulator simulator;
  simulator.simulate(numNodes, sensorOffsetTransf);

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
    g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  // add the parameter representing the sensor offset
  ParameterEpipolarSE3Offset* sensorOffset = new ParameterEpipolarSE3Offset;
  sensorOffset->setOffset(sensorOffsetTransf);
  sensorOffset->setId(0);
  optimizer.addParameter(sensorOffset);

  // adding the odometry to the optimizer
  // first adding all the vertices
  vector<VertexEpipolarSE3*> vertices;
  vector<VertexEpipolarSE3*> trueVertices;
  vector<EdgeEpipolarSE3*> edges;
  cerr << "Optimization: Adding robot poses ... ";
  for (size_t i = 0; i < simulator.poses().size(); ++i) {
    const Simulator::GridPose& p = simulator.poses()[i];
    const SE3& t = p.simulatorPose;
    VertexEpipolarSE3* robot =  new VertexEpipolarSE3;
    VertexEpipolarSE3* trueRobot =  new VertexEpipolarSE3;
    // Set estimate robot pose
    robot->setId(p.id);
    robot->setEstimate(t);
    // Set true robot pose
    trueRobot->setId(p.id);
    trueRobot->setEstimate(p.truePose);
    if (i == 0)
      robot->setFixed(true);
    optimizer.addVertex(robot);
    vertices.push_back(robot);
    trueVertices.push_back(trueRobot);
  }
  // optimizer.activeVertices()[0]->setFixed(true);
  cerr << "done." << endl;

  cerr << "Optimization: add landmark observations ... ";
  for (size_t i = 0; i < simulator.landmarkObservations().size(); ++i) {
    const Simulator::LandmarkEdge& simEdge = simulator.landmarkObservations()[i];
    EdgeEpipolarSE3* landmarkObservation =  new EdgeEpipolarSE3;
    landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
    landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
    landmarkObservation->setMeasurement(simEdge.simulatorMeas);
    landmarkObservation->setInformation(simEdge.information);
    landmarkObservation->setParameterId(0, sensorOffset->id());
    optimizer.addEdge(landmarkObservation);
    edges.push_back(landmarkObservation);
  }
  cerr << "done." << endl;

  vector<Eigen::Vector3d> landmarks;
  for (size_t i = 0; i < simulator.landmarks().size(); ++i){
    const Simulator::Landmark l = simulator.landmarks()[i];
    landmarks.push_back(l.truePose);
  }

  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexEpipolarSE3* firstRobotPose = dynamic_cast<VertexEpipolarSE3*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  write_output("before_epipolar_SE3.g2o", vertices, edges);
  save_scene("poses_SE3.T", "landmarks_R3.lm", vertices, landmarks);
  save_scene("true_poses_SE3.T", "landmarks_R3.lm", trueVertices, landmarks);
  optimizer.initializeOptimization();
  optimizer.optimize(20);
  write_output("after_epipolar_SE3.g2o", vertices, edges);
  save_scene("opt_poses_SE3.T", "opt_landmarks_R3.lm", vertices, landmarks);
  cerr << "done." << endl;

  optimizer.save("tutorial_after.g2o");

  // freeing the graph memory
  optimizer.clear();

  return 0;
}
