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

#ifndef G2O_SIMULATOR_H
#define G2O_SIMULATOR_H

#include "se3.h"
#include "g2o_tutorial_epipolar_slam_api.h"

#include <vector>
#include <map>

namespace g2o {
  namespace tutorial {

    class G2O_TUTORIAL_EPIPOLAR_SLAM_API Simulator {
      public:

        enum G2O_TUTORIAL_EPIPOLAR_SLAM_API MotionType {
          MO_LEFT, MO_RIGHT,
          MO_NUM_ELEMS
        };

        /**
         * \brief simulated landmark
         */
        struct G2O_TUTORIAL_EPIPOLAR_SLAM_API Landmark
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
          int id;
          Eigen::Vector3d truePose;
          Eigen::Vector3d simulatedPose;
          std::vector<int> seenBy;
          Landmark() : id(-1) {}
        };
        typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > LandmarkVector;
        typedef std::vector<Landmark*> LandmarkPtrVector;

        /**
         * simulated pose of the robot
         */
        struct G2O_TUTORIAL_EPIPOLAR_SLAM_API GridPose
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
          int id;
          SE3 truePose;
          SE3 simulatorPose;
          LandmarkPtrVector landmarks;      ///< the landmarks observed by this node
        };
        typedef std::vector<GridPose, Eigen::aligned_allocator<GridPose> >  PosesVector;

        /**
         * \brief point association constraint
         */
        // struct G2O_TUTORIAL_EPIPOLAR_SLAM_API GridEdge
        // {
        //   int from;
        //   int to;
        //   SE3 trueTransf;
        //   SE3 simulatorTransf;
        //   Eigen::Matrix<double, 6, 6> information;
        //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // };
        // typedef std::vector<GridEdge, Eigen::aligned_allocator<GridEdge> >  GridEdgeVector;

        struct G2O_TUTORIAL_EPIPOLAR_SLAM_API LandmarkEdge
        {
          int from;
          int to;
          Eigen::Vector4d trueMeas;
          Eigen::Vector4d simulatorMeas;
          // Eigen::Matrix2d information;
          Eigen::Matrix<double, 1, 1> information;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };
        typedef std::vector<LandmarkEdge, Eigen::aligned_allocator<LandmarkEdge> >  LandmarkEdgeVector;

      public:
        Simulator();
        ~Simulator();

        void simulate(int numPoses, const SE3& sensorOffset = SE3());

        const PosesVector& poses() const { return _poses;}
        const LandmarkVector& landmarks() const { return _landmarks;}
        // const GridEdgeVector& odometry() const { return _odometry;}
        const LandmarkEdgeVector& landmarkObservations() const { return _landmarkObservations;}

      protected:
        PosesVector _poses;
        LandmarkVector _landmarks;
        // GridEdgeVector _odometry;
        LandmarkEdgeVector _landmarkObservations;

        GridPose generateNewPose(const GridPose& prev, const SE3& trueMotion, const Eigen::Vector3d& transNoise, const Eigen::Vector3d& rotNoise);
        SE3 getMotion(int motionDirection, double stepLen);
        SE3 sampleTransformation(const SE3& trueMotion_, const Eigen::Vector3d& transNoise, const Eigen::Vector3d& rotNoise);
    };

  } // end namespace
} // end namespace

#endif
