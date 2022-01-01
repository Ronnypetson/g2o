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

#include <bits/stdc++.h>
#include "edge_epipolar_se3.h"
#include "g2o/core/eigen_types.h"

using namespace Eigen;

namespace g2o {
  namespace tutorial {

    EdgeEpipolarSE3::EdgeEpipolarSE3() :
      BaseBinaryEdge<1, Eigen::Vector4d, VertexEpipolarSE3, VertexEpipolarSE3>()
    {
    }

#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
    void EdgeEpipolarSE3::linearizeOplus()
    {
      computeDiff();
      int POSE_DIM = 6;
      sign = diff.array().sign();

      Eigen::Matrix3d dPIdX;
      dPIdX << 1.0, 0.0, - THdhp0(0) / THdhp0(2),
               0.0, 1.0, - THdhp0(1) / THdhp0(2),
               0.0, 0.0, 0.0;
      dPIdX = (1.0 / THdhp0(2)) * dPIdX.eval();

      Eigen::Vector4d dLandmark_dXii;
      Eigen::Matrix4d _dExpXi_dXii;
      Eigen::Matrix<double, 3, 4> selector;
      Eigen::Vector4d tselector;
      selector << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0;
      tselector << 0, 0, 0, 1;
      Eigen::Vector2d dA_dXii;
      Eigen::Vector2d dB_dXii;
      double ATA, BTB;
      double dd_dXii;
      Eigen::Vector4d Shp0;
      ATA = A.norm();
      BTB = B.norm();
      Shp0.block<3, 1>(0, 0) = hp0;
      Shp0(3) = 0.0;
      double dr_dXii;
      Eigen::Vector3d dPI_dXii;
      // Jacobians w.r.t v1 (vj)
      for (int i = 0; i < POSE_DIM; i++){
        _dExpXi_dXii = -Sophus::SE3<double>::Dxi_exp_x_matrix_at_0(i);

        dA_dXii = _p1 * selector * _dExpXi_dXii * pose_0wrt1 * tselector;
        dB_dXii = _p1 * selector * _dExpXi_dXii * pose_0wrt1 * selector.transpose() * hp0;

        dd_dXii = std::pow(BTB, -0.5) * std::pow(ATA, -0.5) * A.dot(dA_dXii)
                  - std::pow(ATA, 0.5) * std::pow(BTB, -1.5) * B.dot(dB_dXii);

        dLandmark_dXii = _dExpXi_dXii * pose_0wrt1 * Hdhp0 + pose_0wrt1 * dd_dXii * Shp0;

        dPI_dXii = dPIdX * dLandmark_dXii.block<3, 1>(0, 0);
        // dr_dXii = sign.dot(dPI_dXii.block<2, 1>(0, 0));
        dr_dXii = sign.transpose() * dPI_dXii.block<2, 1>(0, 0);
        _jacobianOplusXj(0, i) = dr_dXii;
      }

      // Jacobians w.r.t v0 (vi)
      for (int i = 0; i < POSE_DIM; i++){
        _dExpXi_dXii = Sophus::SE3<double>::Dxi_exp_x_matrix_at_0(i);

        dA_dXii = _p1 * selector * pose_0wrt1 * _dExpXi_dXii * tselector;
        dB_dXii = _p1 * selector * pose_0wrt1 * _dExpXi_dXii * selector.transpose() * hp0;

        dd_dXii = std::pow(BTB, -0.5) * std::pow(ATA, -0.5) * A.dot(dA_dXii)
                  - std::pow(ATA, 0.5) * std::pow(BTB, -1.5) * B.dot(dB_dXii);

        dLandmark_dXii = pose_0wrt1 * _dExpXi_dXii * Hdhp0 + pose_0wrt1 * dd_dXii * Shp0;

        dPI_dXii = dPIdX * dLandmark_dXii.block<3, 1>(0, 0);
        // dr_dXii = sign.dot(dPI_dXii.block<2, 1>(0, 0));
        dr_dXii = sign.transpose() * dPI_dXii.block<2, 1>(0, 0);
        _jacobianOplusXi(0, i) = dr_dXii;
      }
    }
#endif

    bool EdgeEpipolarSE3::read(std::istream& is)
    {
      Eigen::Vector4d p;
      is >> p(0) >> p(1) >> p(2) >> p(3);
      _measurement = p;
      _inverseMeasurement = measurement().inverse();
      for (int i = 0; i < 1; ++i)
        for (int j = i; j < 1; ++j) {
          is >> information()(i, j);
          if (i != j)
            information()(j, i) = information()(i, j);
        }
      return true;
    }

    bool EdgeEpipolarSE3::write(std::ostream& os) const
    {
      Eigen::Vector4d p = measurement();
      os << p(0) << " " << p(1) << " " << p(2) << " " << p(3);
      for (int i = 0; i < 1; ++i)
        for (int j = i; j < 1; ++j)
          os << " " << information()(i, j);
      return os.good();
    }

  } // end namespace
} // end namespace
