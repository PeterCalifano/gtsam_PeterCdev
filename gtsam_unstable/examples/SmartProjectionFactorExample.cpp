/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file SmartProjectionFactorExample.cpp
* @brief A stereo visual odometry example
* @date May 30, 2014
* @author Stephen Camp
* @author Chris Beall
*/


/**
 * A smart projection factor example based on stereo data
 *  - robot starts at origin
 *  -moves forward, taking periodic stereo measurements
 *  -takes stereo readings of many landmarks
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>

#include <gtsam_unstable/slam/SmartProjectionPoseFactor.h>

#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv){

  typedef SmartProjectionPoseFactor<Pose3, Point3, Cal3_S2> SmartFactor;

  Values initial_estimate;
  NonlinearFactorGraph graph;
  const noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(2,1);

  string calibration_loc = findExampleDataFile("VO_calibration.txt");
  string pose_loc = findExampleDataFile("VO_camera_poses_large.txt");
  string factor_loc = findExampleDataFile("VO_stereo_factors_large.txt");
  
  //read camera calibration info from file
  // focal lengths fx, fy, skew s, principal point u0, v0, baseline b
  double fx, fy, s, u0, v0, b;
  ifstream calibration_file(calibration_loc);
  cout << "Reading calibration info" << endl;
  calibration_file >> fx >> fy >> s >> u0 >> v0 >> b;

  //create stereo camera calibration object
//  const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx,fy,s,u0,v0,b));
  
  const Cal3_S2::shared_ptr K(new Cal3_S2(fx, fy, s, u0, v0));

  ifstream pose_file(pose_loc);
  cout << "Reading camera poses" << endl;
  int pose_id;
  MatrixRowMajor m(4,4);
  //read camera pose parameters and use to make initial estimates of camera poses
  while (pose_file >> pose_id) {
    for (int i = 0; i < 16; i++) {
      pose_file >> m.data()[i];
    }
    initial_estimate.insert(Symbol('x', pose_id), Pose3(m));
  }
  
  // camera and landmark keys
  size_t x, l;

  // pixel coordinates uL, uR, v (same for left/right images due to rectification)
  // landmark coordinates X, Y, Z in camera frame, resulting from triangulation
  double uL, uR, v, X, Y, Z;
  ifstream factor_file(factor_loc);
  cout << "Reading stereo factors" << endl;
  //read stereo measurement details from file and use to create and add GenericStereoFactor objects to the graph representation

  SmartFactor::shared_ptr factor(new SmartFactor());
  size_t current_l = 3;   // TODO: Fix me!

  while (factor_file >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {
//    graph.push_back(
//        GenericStereoFactor<Pose3, Point3>(StereoPoint2(uL, uR, v), model,
//            Symbol('x', x), Symbol('l', l), K));

    if(current_l != l) {
      graph.push_back(factor);
      factor = SmartFactor::shared_ptr(new SmartFactor());
      current_l = l;
    }
    factor->add(Point2(uL,v), Symbol('x',x), model, K);

    //if the landmark variable included in this factor has not yet been added to the initial variable value estimate, add it
//    if (!initial_estimate.exists(Symbol('l', l))) {
//      Pose3 camPose = initial_estimate.at<Pose3>(Symbol('x', x));
//      //transform_from() transforms the input Point3 from the camera pose space, camPose, to the global space
//      Point3 worldPoint = camPose.transform_from(Point3(X, Y, Z));
//      initial_estimate.insert(Symbol('l', l), worldPoint);
//    }
  }

  Pose3 first_pose = initial_estimate.at<Pose3>(Symbol('x',1));
  //constrain the first pose such that it cannot change from its original value during optimization
  // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
  // QR is much slower than Cholesky, but numerically more stable
  graph.push_back(NonlinearEquality<Pose3>(Symbol('x',1),first_pose));

  cout << "Optimizing" << endl;
  //create Levenberg-Marquardt optimizer to optimize the factor graph
  LevenbergMarquardtOptimizer optimizer = LevenbergMarquardtOptimizer(graph, initial_estimate);
  Values result = optimizer.optimize();

  cout << "Final result sample:" << endl;
  Values pose_values = result.filter<Pose3>();
  pose_values.print("Final camera poses:\n");

  return 0;
}