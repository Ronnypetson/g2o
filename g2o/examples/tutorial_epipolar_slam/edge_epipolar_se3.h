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

#include <cmath>
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
    class G2O_TUTORIAL_EPIPOLAR_SLAM_API EdgeEpipolarSE3 : public BaseBinaryEdge<1, Eigen::Vector4d, VertexEpipolarSE3, VertexEpipolarSE3>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeEpipolarSE3();

        void computeDiff(){
          const VertexEpipolarSE3* v0 = static_cast<const VertexEpipolarSE3*>(_vertices[0]);
          const VertexEpipolarSE3* v1 = static_cast<const VertexEpipolarSE3*>(_vertices[1]);

          v0_matrix = v0->estimate().toMatrix();
          v1_matrix = v1->estimate().toMatrix();

          pose_0wrt1 = _pose_inverse(v1_matrix) * v0_matrix;
          pose_1wrt0 = _pose_inverse(pose_0wrt1);

          R = pose_0wrt1.block<3, 3>(0, 0);
          t = pose_0wrt1.block<3, 1>(0, 3);

          p0 << _measurement.block<2, 1>(0, 0);
          p1 << _measurement.block<2, 1>(2, 0);

          _p1 << 1.0, 0.0, -p1(0),
                 0.0, 1.0, -p1(1);

          hp0 << p0(0), p0(1), 1.0;
          A = _p1 * t;
          B = _p1 * R * hp0;
          d = A.norm() / B.norm();
          // 2D to 3D homogeneous
          dhp0 << d * p0(0), d * p0(1), d;
          // 3D to 4D homogeneous
          Hdhp0.block<3, 1>(0, 0) = dhp0;
          Hdhp0(3) = 1.0;
          // Reproject and compute residual
          THdhp0 = pose_0wrt1 * Hdhp0;
          piTHdhp0 = THdhp0.block<2, 1>(0, 0) / THdhp0(2);
          diff = piTHdhp0 - p1;
        }

        void computeError()
        {
          computeDiff();
          _error << diff.cwiseAbs().sum();
        }

        void setMeasurement(const Eigen::Vector4d& m){
          /* m has the form (px, py, px_, py_) for the corresponding points
             (px, py) <-> (px_, py_)
          */
          _measurement = m;
          _inverseMeasurement << m(2), m(3), m(0), m(1);
        }

        static double _sign_func(double x)
        {
            if (std::abs(x) > 1E-9){
              if (x > 0)
                return +1.0;
              else
                return -1.0;
            } else {
              if (x > 0)
                return 1E-9;
              else
                return -1E-9;
            }
        }

        Eigen::Matrix<double, 6, 1> linearizeOplusAux(VertexEpipolarSE3* vertex) {
          Eigen::Matrix<double, 6, 1> jacs;
          jacs.fill(0.0);
          if (vertex->fixed()) return jacs;

          constexpr number_t delta = cst(1e-9);
          constexpr number_t scalar = 1 / (2 * delta);

          Eigen::Matrix<double, 6, 1> add_vertex;
          add_vertex.fill(0.0);

          // estimate the jacobian numerically
          // add small step along the unit vector in each dimension
          for (int d = 0; d < 6; ++d) {
            vertex->push();
            add_vertex[d] = delta;
            vertex->oplus(add_vertex.data());
            computeError();
            auto errorBak = this->error();
            vertex->pop();
            vertex->push();
            add_vertex[d] = -delta;
            vertex->oplus(add_vertex.data());
            computeError();
            errorBak -= this->error();
            vertex->pop();
            add_vertex[d] = 0.0;

            jacs(d, 0) = scalar * errorBak(0, 0);
          }  // end dimension
          return jacs;
        }

#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
        virtual void linearizeOplus();
#endif

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        Eigen::Matrix4d _pose_inverse(Eigen::Matrix4d& pose){
          Eigen::Matrix4d _inv = Eigen::Matrix4d::Zero();
          _inv(3, 3) = 1.0;
          Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
          Eigen::Vector3d t = pose.block<3, 1>(0, 3);
          _inv.block<3, 3>(0, 0) = R.transpose();
          _inv.block<3, 1>(0, 3) = -R.transpose() * t;
          return _inv;
        }

        Eigen::Vector4d _inverseMeasurement;

        Eigen::Vector2d p0;
        Eigen::Vector2d p1;

        Eigen::Matrix4d v0_matrix;
        Eigen::Matrix4d v1_matrix;

        Eigen::Matrix4d pose_0wrt1;
        Eigen::Matrix4d pose_1wrt0;

        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        Eigen::Matrix<double, 2, 3> _p1;

        double d;
        Eigen::Vector2d A, B;
        Eigen::Vector3d hp0;

        Eigen::Vector3d dhp0;
        Eigen::Vector4d Hdhp0, THdhp0;
        Eigen::Vector2d piTHdhp0;
        Eigen::Vector2d diff;
        Eigen::Vector2d sign;
    };

  }

} // end namespace

#endif
