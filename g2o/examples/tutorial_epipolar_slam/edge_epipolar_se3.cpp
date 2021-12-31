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

#include "edge_epipolar_se3.h"
#include "g2o/core/eigen_types.h"

using namespace Eigen;

namespace g2o {
  namespace tutorial {

    EdgeEpipolarSE3::EdgeEpipolarSE3() :
      BaseBinaryEdge<1, Eigen::Vector4d, VertexEpipolarSE3, VertexEpipolarSE3>()
    {
    }

// #ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
//     void EdgeEpipolarSE3::linearizeOplus()
//     {
//       const VertexEpipolarSE3* vi     = static_cast<const VertexEpipolarSE3*>(_vertices[0]);
//       const VertexEpipolarSE3* vj     = static_cast<const VertexEpipolarSE3*>(_vertices[1]);
//       // const number_t& x1        = vi->estimate().translation()[0];
//       // const number_t& y1        = vi->estimate().translation()[1];
//       // const number_t& th1       = vi->estimate().rotation().angle();
//       // const number_t& x2        = vj->estimate()[0];
//       // const number_t& y2        = vj->estimate()[1];

//       _jacobianOplusXi( 0 , 0 ) = 0.0;
//     }
// #endif

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
