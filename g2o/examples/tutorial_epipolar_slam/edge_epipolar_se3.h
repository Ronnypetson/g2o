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

#ifndef G2O_TUTORIAL_EDGE_SE3_H
#define G2O_TUTORIAL_EDGE_SE3_H

#include "vertex_epipolar_se3.h"
#include "g2o_tutorial_epipolar_slam_api.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {

  namespace tutorial {

    /**
     * \brief 4D edge between two Vertex3, i.e., the point correspondences
     * Template arguments:
     *  Error dimension,
     *  Measurement type,
     *  Source vertex type,
     *  Target vertex type
     */
    class G2O_TUTORIAL_EPIPOLAR_SLAM_API EdgeEpipolarSE3 : public BaseBinaryEdge<2, Eigen::Vector4d, VertexEpipolarSE3, VertexEpipolarSE3>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeEpipolarSE3();

        void computeError()
        {
          const VertexEpipolarSE3* v0 = static_cast<const VertexEpipolarSE3*>(_vertices[0]);
          const VertexEpipolarSE3* v1 = static_cast<const VertexEpipolarSE3*>(_vertices[1]);
          Eigen::Vector2d p0;
          Eigen::Vector2d p1;
          p0 << _measurement.block<2, 1>(0, 0);
          p1 << _measurement.block<2, 1>(2, 0);
          Eigen::Matrix4d pose_0wrt1 = (v0->estimate() * v1->estimate().inverse()).toMatrix();
          Eigen::Matrix4d pose_1wrt0 = pose_0wrt1.inverse();
          // Optimization for the above line:
          // Eigen::Matrix4d pose_1wrt0 << 0, 0, 0, 0,
          //                               0, 0, 0, 0,
          //                               0, 0, 0, 0,
          //                               0, 0, 0, 1;
          // pose_1wrt0.block<3, 3>(0, 0) = R.transpose();
          // pose_1wrt0.block<3, 1>(0, 3) = -R.transpose() * t;
          // Eigen::Matrix3d R = pose_0wrt1.block<3, 3>(0, 0);
          // Eigen::Vector3d t = pose_0wrt1.block<3, 1>(0, 3);
          Eigen::Matrix3d R = pose_1wrt0.block<3, 3>(0, 0).transpose();
          Eigen::Vector3d t = pose_1wrt0.block<3, 1>(0, 3);
          Eigen::Matrix<double, 2, 3> _p1;
          _p1 << 1.0, 0.0, -p1(0),
                 0.0, 1.0, -p1(1);
          Eigen::Vector2d A, B;
          Eigen::Vector3d hp0;
          hp0 << p0(0), p0(1), 1.0;
          A = _p1 * R * t;
          B = _p1 * R * hp0;
          double d = A.norm() / B.norm();
          // 2D to 3D homogeneous
          Eigen::Vector3d dhp0;
          dhp0 << d * p0(0), d * p0(1), d;
          // 3D to 4D homogeneous
          Eigen::Vector4d Hdhp0, THdhp0;
          Hdhp0.block<3, 1>(0, 0) = dhp0;
          Hdhp0(3) = 1.0;
          // Reproject and compute residual
          THdhp0 = pose_0wrt1 * Hdhp0;
          Eigen::Vector2d piTHdhp0;
          piTHdhp0 = THdhp0.block<2, 1>(0, 0) / THdhp0(2);
          _error = piTHdhp0 - p1;
        }

        void setMeasurement(const Eigen::Vector4d& m){
          /* m has the form (px, py, px_, py_) for the corresponding points
             (px, py) <-> (px_, py_)
          */
          _measurement = m;
          _inverseMeasurement << m(2), m(3), m(0), m(1);
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        Eigen::Vector4d _inverseMeasurement;
    };

  }

} // end namespace

#endif
