/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Hybrid_City10000.cpp
 * @brief  Example of using hybrid estimation
 *         with multiple odometry measurements.
 * @author Varun Agrawal
 * @date   January 22, 2025
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;

using symbol_shorthand::M;
using symbol_shorthand::X;

// Testing params
const size_t max_loop_count = 1000;  // 2000;  // 200 //2000 //8000

noiseModel::Diagonal::shared_ptr prior_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.0001, 0.0001, 0.0001).finished());

noiseModel::Diagonal::shared_ptr pose_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(3) << 1.0 / 30.0, 1.0 / 30.0, 1.0 / 100.0).finished());

/**
 * @brief Write the results of optimization to filename.
 *
 * @param results The Values object with the final results.
 * @param num_poses The number of poses to write to the file.
 * @param filename The file name to save the results to.
 */
void write_results(const Values& results, size_t num_poses,
                   const std::string& filename = "ISAM2_city10000.txt") {
  ofstream outfile;
  outfile.open(filename);

  for (size_t i = 0; i < num_poses; ++i) {
    Pose2 out_pose = results.at<Pose2>(X(i));

    outfile << out_pose.x() << " " << out_pose.y() << " " << out_pose.theta()
            << std::endl;
  }
  outfile.close();
  std::cout << "output written to " << filename << std::endl;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  ifstream in(findExampleDataFile("T1_city10000_04.txt"));
  // ifstream in("../data/mh_T1_city10000_04.txt"); //Type #1 only
  // ifstream in("../data/mh_T3b_city10000_10.txt"); //Type #3 only
  // ifstream in("../data/mh_T1_T3_city10000_04.txt"); //Type #1 + Type #3

  // ifstream in("../data/mh_All_city10000_groundtruth.txt");

  size_t pose_count = 0, discrete_count = 0;
  size_t index = 0;

  std::list<double> time_list;

  HybridNonlinearISAM isam;

  HybridNonlinearFactorGraph graph;

  Values init_values;
  Values results;

  size_t maxNrHypotheses = 3;

  double x = 0.0;
  double y = 0.0;
  double rad = 0.0;

  Pose2 prior_pose(x, y, rad);

  init_values.insert(X(0), prior_pose);
  pose_count += 1;

  graph.push_back(PriorFactor<Pose2>(X(0), prior_pose, prior_noise_model));

  isam.update(graph, init_values, maxNrHypotheses);

  graph.resize(0);
  init_values.clear();
  results = isam.estimate();

  size_t key_s, key_t;

  clock_t start_time = clock();
  std::string str;
  while (getline(in, str) && index < max_loop_count) {
    vector<string> parts;
    split(parts, str, is_any_of(" "));

    key_s = stoi(parts[1]);
    key_t = stoi(parts[3]);

    int num_measurements = stoi(parts[5]);
    vector<Pose2> pose_array(num_measurements);
    for (int i = 0; i < num_measurements; ++i) {
      x = stod(parts[6 + 3 * i]);
      y = stod(parts[7 + 3 * i]);
      rad = stod(parts[8 + 3 * i]);
      pose_array[i] = Pose2(x, y, rad);
    }

    // Take the first one as the initial estimate
    Pose2 odom_pose = pose_array[0];
    if (key_s == key_t - 1) {  // new X(key)
      init_values.insert(X(key_t), results.at<Pose2>(X(key_s)) * odom_pose);
      pose_count++;
    } else {  // loop
      // index++;
    }

    if (num_measurements == 2) {
      // Add hybrid factor which considers both measurements
      DiscreteKey m(M(discrete_count), num_measurements);
      discrete_count++;

      graph.push_back(DecisionTreeFactor(m, "0.6 0.4"));

      auto f0 = std::make_shared<BetweenFactor<Pose2>>(
          X(key_s), X(key_t), pose_array[0], pose_noise_model);
      auto f1 = std::make_shared<BetweenFactor<Pose2>>(
          X(key_s), X(key_t), pose_array[1], pose_noise_model);
      std::vector<NonlinearFactorValuePair> factors{{f0, 0.0}, {f1, 0.0}};
      // HybridNonlinearFactor mixtureFactor(m, factors);
      HybridNonlinearFactor mixtureFactor(m, {f0, f1});
      graph.push_back(mixtureFactor);

    } else {
      graph.add(BetweenFactor<Pose2>(X(key_s), X(key_t), odom_pose,
                                     pose_noise_model));
    }

    isam.update(graph, init_values, maxNrHypotheses);
    graph.resize(0);
    init_values.clear();
    results = isam.estimate();
    isam.assignment().print("The Discrete Assignment");

    // Print loop index and time taken in processor clock ticks
    if (index % 50 == 0 && key_s != key_t - 1) {
      std::cout << "index: " << index << std::endl;
      std::cout << "acc_time:  " << time_list.back() << std::endl;
    }

    if (key_s == key_t - 1) {
      clock_t cur_time = clock();
      time_list.push_back(cur_time - start_time);
    }

    if (time_list.size() % 100 == 0 && (key_s == key_t - 1)) {
      string step_file_idx = std::to_string(100000 + time_list.size());

      ofstream step_outfile;
      string step_file_name =
          "step_files/HybridISAM_city10000_S" + step_file_idx;
      step_outfile.open(step_file_name + ".txt");
      for (size_t i = 0; i < (key_t + 1); ++i) {
        Pose2 out_pose = results.at<Pose2>(X(i));
        step_outfile << out_pose.x() << " " << out_pose.y() << " "
                     << out_pose.theta() << endl;
      }
      step_outfile.close();
    }

    index += 1;
  }

  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time << endl;

  /// Write results to file
  write_results(results, (key_t + 1), "HybridISAM_city10000.txt");

  ofstream outfile_time;
  std::string time_file_name = "HybridISAM_city10000_time.txt";
  outfile_time.open(time_file_name);
  for (auto acc_time : time_list) {
    outfile_time << acc_time << std::endl;
  }
  outfile_time.close();
  cout << "output " << time_file_name << " file." << endl;

  return 0;
}
