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
          /*
          TODO: implement using Sophus
          */
          SE3 result(*this);
          // Eigen::MatrixXd delta = Eigen::MatrixXd::Zero(6, 1);
          Eigen::MatrixXd exp_left = Sophus::SE3<double>::exp(result.toVector()).matrix();
          Eigen::MatrixXd exp_right = Sophus::SE3<double>::exp(tr3.toVector()).matrix();
          Eigen::MatrixXd exp_result = exp_left * exp_right;
          Sophus::SE3<double> _result(exp_result);
          Vector6 vec_result = _result.log();
          result._t << vec_result(0), vec_result(1), vec_result(2);
          result._R << vec_result(3), vec_result(4), vec_result(5);
          // result._t += _R * tr3._t;
          // result._R.angle() += tr3._R.angle();
          // result._R.angle() = normalize_theta(result._R.angle());
          return result;
        }

        SE3& operator *= (const SE3& tr3){
          // _t += _R * tr3._t;
          // _R.angle() += tr3._R.angle();
          // _R.angle() = normalize_theta(_R.angle());
          return *this;
        }

        Eigen::Vector3d operator * (const Eigen::Vector3d& v) const {
          // return _t + _R * v;
          return v;
        }

        SE3 inverse() const{
          // SE3 ret;
          // ret._R = _R.inverse();
          // ret._R.angle() = normalize_theta(ret._R.angle());
          // ret._t = ret._R * (Eigen::Vector3d(-1 * _t));
          // return ret;
          return *this;
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

        /*
        TODO: change to 6d vector
        */
        void fromVector (const Vector6& v){
          *this = SE3(v[0], v[1], v[2], v[3], v[4], v[5]);
        }

        /*
        TODO: change to 6d vector
        */
        Vector6 toVector() const {
          Vector6 ret;
          for (int i = 0; i < 6; i++){
            ret(i) = (*this)[i];
          }
          return ret;
        }

      protected:
        Eigen::Vector3d _R;
        Eigen::Vector3d _t;
    };

  } // end namespace
} // end namespace

#endif
