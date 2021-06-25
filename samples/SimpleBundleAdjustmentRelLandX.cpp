//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SimpleBundleAdjustmentRelLand.cpp
/// \brief A sample usage of the STEAM Engine library for a bundle adjustment problem
///        with relative landmarks. Uses dynamic-size matrices (rather than fixed).
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <thread>

#include <lgmath.hpp>
#include <steam.hpp>
#include <steam/data/ParseBA.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Example that loads and solves simple bundle adjustment problems
//////////////////////////////////////////////////////////////////////////////////////////////
void runBundleAdjustment(steam::data::SimpleBaDataset dataset) {
  ///
  /// Setup States
  ///

  // Set a fixed identity transform that will be used to initialize landmarks in their parent frame
  steam::se3::FixedTransformEvaluator::Ptr tf_identity = steam::se3::FixedTransformEvaluator::MakeShared(lgmath::se3::Transformation());

  // Fixed vehicle to camera transform
  steam::se3::TransformEvaluator::Ptr tf_c_v = steam::se3::FixedTransformEvaluator::MakeShared(dataset.T_cv);

  // Ground truth
  std::vector<steam::se3::TransformStateVar::Ptr> poses_gt_k_0;
  std::vector<steam::se3::LandmarkStateVar::Ptr> landmarks_gt;

  // State variable containers (and related data)
  std::vector<steam::se3::TransformStateVar::Ptr> poses_ic_k_0;
  std::vector<steam::se3::LandmarkStateVar::Ptr> landmarks_ic;

  // Record the frame in which the landmark is first seen in order to set up transforms correctly
  std::map<unsigned, unsigned> landmark_map;

  ///
  /// Initialize States
  ///

  // Setup ground-truth poses
  for (unsigned int i = 0; i < dataset.frames_gt.size(); i++) {
    steam::se3::TransformStateVar::Ptr temp(new steam::se3::TransformStateVar(dataset.frames_gt[i].T_k0));
    poses_gt_k_0.push_back(temp);
  }

  // Setup ground-truth landmarks
  for (unsigned int i = 0; i < dataset.land_gt.size(); i++) {
    steam::se3::LandmarkStateVar::Ptr temp(new steam::se3::LandmarkStateVar(dataset.land_gt[i].point));
    landmarks_gt.push_back(temp);
  }

  // Setup poses with initial condition
  for (unsigned int i = 0; i < dataset.frames_ic.size(); i++) {
    steam::se3::TransformStateVar::Ptr temp(new steam::se3::TransformStateVar(dataset.frames_ic[i].T_k0));
    poses_ic_k_0.push_back(temp);
  }

  // Lock first pose (otherwise entire solution is 'floating')
  //  **Note: alternatively we could add a prior to the first pose.
  poses_ic_k_0[0]->setLock(true);

  // Setup relative landmarks
  landmarks_ic.resize(dataset.land_ic.size());
  for (unsigned int i = 0; i < dataset.meas.size(); i++) {

    // Get pose reference
    unsigned int frameIdx = dataset.meas[i].frameID;

    // Get landmark reference
    unsigned int landmarkIdx = dataset.meas[i].landID;

    // Setup landmark if first time
    if (!landmarks_ic[landmarkIdx]) {

      // Get homogeneous point in inertial frame
      Eigen::Vector4d p_0;
      p_0.head<3>() = dataset.land_ic[landmarkIdx].point;
      p_0[3] = 1.0;

      // Get transform between first observation time and inertial frame
      lgmath::se3::Transformation pose_vk_0 = dataset.frames_ic[frameIdx].T_k0/dataset.frames_ic[0].T_k0;

      // Get point in 'local' frame
      Eigen::Vector4d p_vehicle = pose_vk_0 * p_0;

      // Insert the landmark
      landmarks_ic[landmarkIdx] = steam::se3::LandmarkStateVar::Ptr(new steam::se3::LandmarkStateVar(p_vehicle.head<3>()));

      // Keep a record of its 'parent' frame
      landmark_map[landmarkIdx] = frameIdx;
    }
  }

  ///
  /// Setup Cost Terms
  ///

  // steam cost terms
  steam::ParallelizedCostTermCollection::Ptr stereoCostTerms(new steam::ParallelizedCostTermCollection());

  // Setup shared noise and loss function
  steam::BaseNoiseModelX::Ptr sharedCameraNoiseModel(new steam::StaticNoiseModelX(dataset.noise));
  steam::L2LossFunc::Ptr sharedLossFunc(new steam::L2LossFunc());

  // Setup camera intrinsics
  steam::StereoCameraErrorEvalX::CameraIntrinsics::Ptr sharedIntrinsics(
        new steam::StereoCameraErrorEvalX::CameraIntrinsics());
  sharedIntrinsics->b  = dataset.camParams.b;
  sharedIntrinsics->fu = dataset.camParams.fu;
  sharedIntrinsics->fv = dataset.camParams.fv;
  sharedIntrinsics->cu = dataset.camParams.cu;
  sharedIntrinsics->cv = dataset.camParams.cv;

  // Generate cost terms for camera measurements
  for (unsigned int i = 0; i < dataset.meas.size(); i++) {

    // Get pose reference
    unsigned int frameIdx = dataset.meas[i].frameID;

    // Get landmark reference
    unsigned int landmarkIdx = dataset.meas[i].landID;
    steam::se3::LandmarkStateVar::Ptr& landVar = landmarks_ic[landmarkIdx];

    // Construct transform evaluator between two vehicle frames (a and b) that have observations
    steam::se3::TransformEvaluator::Ptr tf_vb_va;
    if(landmark_map[landmarkIdx] == frameIdx) {

        // In this case, the transform remains fixed as an identity transform
        tf_vb_va = tf_identity;

    } else {

      unsigned int firstObsIndex = landmark_map[landmarkIdx];
      steam::se3::TransformEvaluator::Ptr pose_va_0 = steam::se3::TransformStateEvaluator::MakeShared(poses_ic_k_0[firstObsIndex]);
      steam::se3::TransformEvaluator::Ptr pose_vb_0 = steam::se3::TransformStateEvaluator::MakeShared(poses_ic_k_0[frameIdx]);
      tf_vb_va = steam::se3::composeInverse(pose_vb_0, pose_va_0);
    }

    // Compose with camera to vehicle transform
    steam::se3::TransformEvaluator::Ptr tf_cb_va = steam::se3::compose(tf_c_v, tf_vb_va);

    // Construct error function
    steam::StereoCameraErrorEvalX::Ptr errorfunc(new steam::StereoCameraErrorEvalX(
            dataset.meas[i].data, sharedIntrinsics, tf_cb_va, landVar));

    // Construct cost term
    steam::WeightedLeastSqCostTermX::Ptr cost(new steam::WeightedLeastSqCostTermX(errorfunc, sharedCameraNoiseModel, sharedLossFunc));
    stereoCostTerms->add(cost);
  }

  ///
  /// Make Optimization Problem
  ///

  // Initialize problem
  steam::OptimizationProblem problem;

  // Add pose variables
  for (unsigned int i = 1; i < poses_ic_k_0.size(); i++) {
    problem.addStateVariable(poses_ic_k_0[i]);
  }

  // Add landmark variables
  for (unsigned int i = 0; i < landmarks_ic.size(); i++) {
    problem.addStateVariable(landmarks_ic[i]);
  }

  // Add cost terms
  problem.addCostTerm(stereoCostTerms);

  ///
  /// Setup Solver and Optimize
  ///
  typedef steam::DoglegGaussNewtonSolver SolverType;

  // Initialize parameters (enable verbose mode)
  SolverType::Params params;
  params.verbose = true;

  // Make solver
  SolverType solver(&problem, params);

  // Optimize
  solver.optimize();
}

int main(int argc, char** argv) {
  ///
  /// Parse Dataset
  ///

  // Get filename
  std::string filename;
  if (argc < 2) {
    filename = "../include/steam/data/stereo_simulated.txt";
    //filename = "../include/steam/data/stereo_simulated_window1.txt";
    //filename = "../include/steam/data/stereo_simulated_window2.txt";
    std::cout << "Parsing default file: " << filename << std::endl << std::endl;
  } else {
    filename = argv[1];
    std::cout << "Parsing file: " << filename << std::endl << std::endl;
  }

  // Load dataset
  steam::data::SimpleBaDataset dataset = steam::data::parseSimpleBaDataset(filename);
  std::cout << "Problem has: " << dataset.frames_gt.size() << " poses" << std::endl;
  std::cout << "             " << dataset.land_gt.size() << " landmarks" << std::endl;
  std::cout << "            ~" << double(dataset.meas.size())/dataset.frames_gt.size() << " meas per pose" << std::endl << std::endl;

  /// Test single thread execution
  std::cout << "Test single thread execution." << std::endl;
  runBundleAdjustment(dataset);

#ifndef STEAM_USE_OBJECT_POOL
  /// Test multi thread execution
  std::cout << "Test multi thread execution (C++11)." << std::endl;
  std::vector<std::thread> threads;
  for (int i = 1; i <= 10; ++i)
    threads.push_back(std::thread(runBundleAdjustment, dataset));
  for (auto &th : threads)
    th.join();
#endif
}
