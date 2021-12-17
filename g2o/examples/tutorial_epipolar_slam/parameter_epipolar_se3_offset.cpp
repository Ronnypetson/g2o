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

#include "parameter_epipolar_se3_offset.h"

#include "vertex_epipolar_se3.h"

namespace g2o {
  namespace tutorial {

    ParameterEpipolarSE3Offset::ParameterEpipolarSE3Offset()
    {
    }

    void ParameterEpipolarSE3Offset::setOffset(const SE3& offset)
    {
      _offset = offset;
      _inverseOffset = offset.inverse();
    }

    bool ParameterEpipolarSE3Offset::read(std::istream& is)
    {
      double t0, t1, t2, r0, r1, r2;
      is >> t0 >> t1 >> t2 >> r0 >> r1 >> r2;
      setOffset(SE3(t0, t1, t2, r0, r1, r2));
      return true;
    }

    bool ParameterEpipolarSE3Offset::write(std::ostream& os) const
    {
      Eigen::Vector3d t = _offset.translation();
      Eigen::Vector3d R = _offset.rotation();
      os << t(0) << " " << t(1) << " " << t(2) << " " << R(0) << " " << R(1) << " " << R(2);
      return os.good();
    }

    void CacheEpipolarSE3Offset::updateImpl()
    {
      const VertexEpipolarSE3* v = static_cast<const VertexEpipolarSE3*>(vertex());
      _n2w = v->estimate() * _offsetParam->offset();
      _w2n = _n2w.inverse();
    }

    bool CacheEpipolarSE3Offset::resolveDependancies()
    {
      _offsetParam = dynamic_cast<ParameterEpipolarSE3Offset*> (_parameters[0]);
      return _offsetParam != 0;
    }

  } // end namespace tutorial
} // end namespace
