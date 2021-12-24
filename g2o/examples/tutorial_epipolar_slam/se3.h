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

#ifndef G2O_TUTORIAL_SE3_H
#define G2O_TUTORIAL_SE3_H

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"
#include "g2o/core/eigen_types.h"
#include "g2o_tutorial_epipolar_slam_api.h"

#include "sophus/geometry.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace g2o {

  namespace tutorial {

    class G2O_TUTORIAL_EPIPOLAR_SLAM_API SE3 {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        SE3(): _R(0, 0, 0), _t(0, 0, 0){}

        SE3(
          double t0,
          double t1,
          double t2,
          double r0,
          double r1,
          double r2): _R(r0, r1, r2), _t(t0, t1, t2){}

        const Eigen::Vector3d& translation() const {return _t;}

        Eigen::Vector3d& translation() {return _t;}

        const Eigen::Vector3d& rotation() const {return _R;}

        Eigen::Vector3d& rotation() {return _R;}

        SE3 operator * (const SE3& tr3) const{
          SE3 result(*this);
          Eigen::Matrix4d exp_left = Sophus::SE3<double>::exp(result.toVector()).matrix();
          Eigen::Matrix4d exp_right = Sophus::SE3<double>::exp(tr3.toVector()).matrix();
          Eigen::Matrix4d exp_result = exp_left * exp_right;
          Sophus::SE3<double> _result(exp_result);
          Vector6 vec_result = _result.log();
          result._t << vec_result.block<3, 1>(0, 0);
          result._R << vec_result.block<3, 1>(3, 0);
          return result;
        }

        SE3& operator *= (const SE3& tr3){
          SE3 tmp(*this);
          Eigen::Matrix4d exp_result = Sophus::SE3<double>::exp((tmp * tr3).toVector()).matrix();
          Sophus::SE3<double> _result(exp_result);
          Vector6 vec_result = _result.log();
          _t = vec_result.block<3, 1>(0, 0);
          _R = vec_result.block<3, 1>(3, 0);
          return *this;
        }

        Eigen::Vector3d operator * (const Eigen::Vector3d& v) const {
          Eigen::Vector4d v_hom, u_hom;
          v_hom << v(0), v(1), v(2), 1.0;
          SE3 tmp(*this);
          Eigen::Matrix4d exp_T = Sophus::SE3<double>::exp(tmp.toVector()).matrix();
          u_hom = exp_T * v_hom;
          return u_hom.block<3, 1>(0, 0);
        }

        SE3 inverse() const{
          SE3 ret;
          SE3 tmp(*this);
          Eigen::Matrix4d exp_T = Sophus::SE3<double>::exp(tmp.toVector()).matrix();
          exp_T = exp_T.inverse().eval();
          Sophus::SE3<double> _result(exp_T);
          Vector6 vec_result = _result.log();
          ret._t = vec_result.block<3, 1>(0, 0);
          ret._R = vec_result.block<3, 1>(3, 0);
          return ret;
        }

        double operator [](int i) const {
          assert (i >= 0 && i < 6);
          if (i < 3)
            return _t(i);
          return _R(i - 3);
        }

        double& operator [](int i) {
          assert (i >= 0 && i < 6);
          if (i < 3)
            return _t(i);
          return _R(i - 3);
        }

        void fromVector (const Vector6& v){
          *this = SE3(v[0], v[1], v[2], v[3], v[4], v[5]);
        }

        Vector6 toVector() const {
          Vector6 ret;
          for (int i = 0; i < 6; i++){
            ret(i) = (*this)[i];
          }
          return ret;
        }

        Eigen::Matrix4d toMatrix() const {
          SE3 tmp(*this);
          Eigen::Matrix4d exp_T = Sophus::SE3<double>::exp(tmp.toVector()).matrix();
          return exp_T;
        }

      protected:
        Eigen::Vector3d _R;
        Eigen::Vector3d _t;
    };

  } // end namespace
} // end namespace

#endif
