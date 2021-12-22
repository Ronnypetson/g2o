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

#include "simulator.h"

#include "g2o/stuff/sampler.h"

#include <map>
#include <iostream>
#include <cmath>
using namespace std;

namespace g2o {
  namespace tutorial {

    using namespace Eigen;

#  ifdef _MSC_VER
    inline double round(double number)
    {
      return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
    }
#  endif

    typedef std::map<int, std::map<int, Simulator::LandmarkPtrVector> > LandmarkGrid;

    Simulator::Simulator()
    {
      time_t seed = time(0);
      Sampler::seedRand(static_cast<unsigned int>(seed));
    }

    Simulator::~Simulator()
    {
    }

    void Simulator::simulate(int numNodes, const SE3& sensorOffset)
    {
      // simulate a robot observing landmarks while travelling on a grid
      int steps = 5;
      double stepLen = 1.0;
      int boundArea = 50;

      double maxSensorRangeLandmarks = 2.5 * stepLen;

      int landMarksPerSquareMeter = 1;
      double observationProb = 0.8;

      int landmarksRange=2;

      Vector3d transNoise(0.01, 0.01, 0.01);
      Vector3d rotNoise(0.01, 0.01, 0.01);
      Vector4d landmarkNoise(0.05, 0.05, 0.05, 0.05);

      Vector2d bound(boundArea, boundArea);

      VectorXd probLimits;
      probLimits.resize(MO_NUM_ELEMS);
      for (int i = 0; i < probLimits.size(); ++i)
        probLimits[i] = (i + 1) / (double) MO_NUM_ELEMS;

      SE3 maxStepTransf(stepLen * steps, 0, 0, 0, 0, 0);
      Simulator::PosesVector& poses = _poses;
      poses.clear();
      LandmarkVector& landmarks = _landmarks;
      landmarks.clear();
      Simulator::GridPose firstPose;
      firstPose.id = 0;
      firstPose.truePose = SE3(0, 0, 0, 0, 0, 0);
      firstPose.simulatorPose = SE3(0, 0, 0, 0, 0, 0);
      poses.push_back(firstPose);
      cerr << "Simulator: sampling nodes ...";
      while ((int)poses.size() < numNodes) {
        // add straight motions
        for (int i = 1; i < steps && (int)poses.size() < numNodes; ++i) {
          Simulator::GridPose nextGridPose = generateNewPose(poses.back(), SE3(stepLen, 0, 0, 0, 0, 0), transNoise, rotNoise);
          poses.push_back(nextGridPose);
        }
        if ((int)poses.size() == numNodes)
          break;

        // sample a new motion direction
        double sampleMove = Sampler::uniformRand(0., 1.);
        int motionDirection = 0;
        while (probLimits[motionDirection] < sampleMove && motionDirection+1 < MO_NUM_ELEMS) {
          motionDirection++;
        }

        SE3 nextMotionStep = getMotion(motionDirection, stepLen);
        Simulator::GridPose nextGridPose = generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);

        // check whether we will walk outside the boundaries in the next iteration
        SE3 nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
        if (fabs( (nextStepFinalPose.translation())(0) ) >= bound[0] || fabs( (nextStepFinalPose.translation())(1) ) >= bound[1]) {
          //cerr << "b";
          // will be outside boundaries using this
          for (int i = 0; i < MO_NUM_ELEMS; ++i) {
            nextMotionStep = getMotion(i, stepLen);
            nextGridPose = generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);
            nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
            if (fabs( (nextStepFinalPose.translation())(0) ) < bound[0] && fabs( (nextStepFinalPose.translation())(1) ) < bound[1])
              break;
          }
        }

        poses.push_back(nextGridPose);
      }
      cerr << "done." << endl;

      // creating landmarks along the trajectory
      cerr << "Simulator: Creating landmarks ... ";
      LandmarkGrid grid;
      for (PosesVector::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        int ccx = (int)round(it->truePose.translation()(0));
        int ccy = (int)round(it->truePose.translation()(1));
        for (int a=-landmarksRange; a<=landmarksRange; a++)
          for (int b=-landmarksRange; b<=landmarksRange; b++){
            int cx=ccx+a;
            int cy=ccy+b;
            LandmarkPtrVector& landmarksForCell = grid[cx][cy];
            if (landmarksForCell.size() == 0) {
              for (int i = 0; i < landMarksPerSquareMeter; ++i) {
                Landmark* l = new Landmark();
                double offx, offy, offz;
                do {
                  offx = Sampler::uniformRand(-0.5*stepLen, 0.5*stepLen);
                  offy = Sampler::uniformRand(-0.5*stepLen, 0.5*stepLen);
                  // offz = Sampler::uniformRand(-0.5*stepLen, 0.5*stepLen);
                } while (hypot_sqr(offx, offy) < 0.25 * 0.25);
                l->truePose[0] = cx + offx;
                l->truePose[1] = cy + offy;
                l->truePose[2] = 0.0;
                landmarksForCell.push_back(l);
              }
            }
          }
      }
      cerr << "done." << endl;

      cerr << "Simulator: Simulating landmark observations for the poses ... ";
      double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
      int globalId = 0;
      for (PosesVector::iterator it = poses.begin(); it != poses.end(); ++it) {
        Simulator::GridPose& pv = *it;
        int cx = (int)round(it->truePose.translation()(0));
        int cy = (int)round(it->truePose.translation()(1));
        int numGridCells = (int)(maxSensorRangeLandmarks) + 1;

        pv.id = globalId++;
        SE3 trueInv = pv.truePose.inverse();

        for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx)
          for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
            LandmarkPtrVector& landmarksForCell = grid[xx][yy];
            if (landmarksForCell.size() == 0)
              continue;
            for (size_t i = 0; i < landmarksForCell.size(); ++i) {
              Landmark* l = landmarksForCell[i];
              double dSqr = hypot_sqr(
                pv.truePose.translation()(0) - l->truePose(0),
                pv.truePose.translation()(1) - l->truePose(1),
                pv.truePose.translation()(2) - l->truePose(2)
                );
              if (dSqr > maxSensorSqr)
                continue;
              double obs = Sampler::uniformRand(0.0, 1.0);
              if (obs > observationProb) // we do not see this one...
                continue;
              if (l->id < 0)
                l->id = globalId++;
              if (l->seenBy.size() == 0) {
                Vector3d trueObservation = trueInv * l->truePose;
                Vector3d observation = trueObservation;
                observation[0] += Sampler::gaussRand(0., landmarkNoise[0]);
                observation[1] += Sampler::gaussRand(0., landmarkNoise[1]);
                observation[2] += Sampler::gaussRand(0., landmarkNoise[2]);
                l->simulatedPose = pv.simulatorPose * observation;
              }
              l->seenBy.push_back(pv.id);
              pv.landmarks.push_back(l);
            }
          }

      }
      cerr << "done." << endl;

      _landmarks.clear();
      _landmarkObservations.clear();
      // add the landmark observations
      {
        cerr << "Simulator: add landmark observations ... ";
        Matrix2d covariance; covariance.fill(0.);
        covariance(0, 0) = landmarkNoise[0] * landmarkNoise[0] + landmarkNoise[2] * landmarkNoise[2];
        covariance(1, 1) = landmarkNoise[1] * landmarkNoise[1] + landmarkNoise[3] * landmarkNoise[3];
        // covariance(0, 0) = 1.0;
        // covariance(1, 1) = 1.0;
        Matrix2d information = covariance.inverse();

        for (size_t i = 0; i < poses.size(); ++i) {
          const GridPose& p = poses[i];
          for (size_t j = 0; j < p.landmarks.size(); ++j) {
            Landmark* l = p.landmarks[j];
            if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
              landmarks.push_back(*l);
            }
          }
        }

        /*
        Take a pose poses[i], take poses[i].landmark[j], take poses[i].landmark[j].seenBy[k]
        compute the observations from the two poses
        */
        for (size_t i = 0; i < poses.size(); ++i) {
          const GridPose& p = poses[i];
          SE3 trueInv = (p.truePose * sensorOffset).inverse();
          for (size_t j = 0; j < p.landmarks.size(); ++j) {
            Landmark* l = p.landmarks[j];

            for (size_t k = 0; k < l->seenBy.size(); ++k) {
              const GridPose& p_tgt = poses[l->seenBy[k]];
              SE3 trueInv_tgt = (p_tgt.truePose * sensorOffset).inverse();

              Vector3d trueObservation = trueInv * l->truePose;
              Vector3d trueObservation_tgt = trueInv_tgt * l->truePose;
              Eigen::Vector4d trueObservation4d, observation4d;

              trueObservation4d(0) = trueObservation(0) / trueObservation(2);
              trueObservation4d(1) = trueObservation(1) / trueObservation(2);
              trueObservation4d(2) = trueObservation_tgt(0) / trueObservation_tgt(2);
              trueObservation4d(3) = trueObservation_tgt(1) / trueObservation_tgt(2);

              observation4d(0) = trueObservation4d(0) + Sampler::gaussRand(0.0, landmarkNoise[0]);
              observation4d(1) = trueObservation4d(1) + Sampler::gaussRand(0.0, landmarkNoise[1]);
              observation4d(2) = trueObservation4d(2) + Sampler::gaussRand(0.0, landmarkNoise[2]);
              observation4d(3) = trueObservation4d(3) + Sampler::gaussRand(0.0, landmarkNoise[3]);

              _landmarkObservations.push_back(LandmarkEdge());
              LandmarkEdge& le = _landmarkObservations.back();

              le.from = p.id;
              le.to = p_tgt.id;
              le.trueMeas = trueObservation4d;
              le.simulatorMeas = observation4d;
              le.information = information;
            }

          }
        }
        cerr << "done." << endl;
      }

      // cleaning up
      for (LandmarkGrid::iterator it = grid.begin(); it != grid.end(); ++it) {
        for (std::map<int, Simulator::LandmarkPtrVector>::iterator itt = it->second.begin(); itt != it->second.end(); ++itt) {
          Simulator::LandmarkPtrVector& landmarks = itt->second;
          for (size_t i = 0; i < landmarks.size(); ++i)
            delete landmarks[i];
        }
      }

    }

    Simulator::GridPose Simulator::generateNewPose(const Simulator::GridPose& prev, const SE3& trueMotion, const Eigen::Vector3d& transNoise, const Eigen::Vector3d& rotNoise)
    {
      Simulator::GridPose nextPose;
      nextPose.id = prev.id + 1;
      nextPose.truePose = prev.truePose * trueMotion;
      SE3 noiseMotion = sampleTransformation(trueMotion, transNoise, rotNoise);
      nextPose.simulatorPose = prev.simulatorPose * noiseMotion;
      return nextPose;
    }

    SE3 Simulator::getMotion(int motionDirection, double stepLen)
    {
      switch (motionDirection) {
        case MO_LEFT:
          // return SE3(0.1 * stepLen, 0, 0.5*M_PI, 0, 0, 0);
          return SE3(stepLen, 0, 0, 0, 0.2, 0);
        case MO_RIGHT:
          // return SE3(0.1 * stepLen, 0, -0.5*M_PI, 0, 0, 0);
          return SE3(stepLen, 0, 0, 0, 0, 0.2);
        default:
          cerr << "Unknown motion direction" << endl;
          // return SE3(0.1 * stepLen, 0, -0.5*M_PI, 0, 0, 0);
          return SE3(stepLen, 0, 0, 0.2, 0, 0);
      }
    }

    SE3 Simulator::sampleTransformation(const SE3& trueMotion_, const Eigen::Vector3d& transNoise, const Eigen::Vector3d& rotNoise)
    {
      Vector6 trueMotion = trueMotion_.toVector();
      SE3 noiseMotion(
          trueMotion[0] + Sampler::gaussRand(0.0, transNoise[0]),
          trueMotion[1] + Sampler::gaussRand(0.0, transNoise[1]),
          trueMotion[2] + Sampler::gaussRand(0.0, transNoise[2]),
          trueMotion[3] + Sampler::gaussRand(0.0, rotNoise[0]),
          trueMotion[4] + Sampler::gaussRand(0.0, rotNoise[1]),
          trueMotion[5] + Sampler::gaussRand(0.0, rotNoise[2])
          );
      return noiseMotion;
    }

  } // end namespace
} // end namespace
