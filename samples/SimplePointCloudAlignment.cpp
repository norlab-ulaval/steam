//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SimplePointCloudAlignment.cpp
/// \brief
///
/// \author Yuchen Wu, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include <lgmath.hpp>
#include <steam.hpp>

int main(int argc, char **argv) {
  /// Reference points to align against
  Eigen::Matrix<double, 4, 9> ref_pts;
  // clang-format off
  ref_pts << 0,   1,   1,  0, -1, -1, -1,  0,  1,
             0,   0,   1,  1,  1,  0, -1, -1, -1,
           0.1, 0.2, 0.3,  0,  0,  0,  0,  0,  0,
             1,   1,   1,  1,  1,  1,  1,  1,  1;
  // clang-format on

  /// Ground truth pose
  Eigen::Matrix<double, 6, 1> T_mq_vec;
  T_mq_vec << 1, 1, 1, 1, 1, 1;
  lgmath::se3::Transformation T_mq(T_mq_vec);

  Eigen::Matrix<double, 4, 9> qry_pts = T_mq.inverse().matrix() * ref_pts;

  // Initialize problem
  steam::OptimizationProblem problem;

  // state and evaluator
  using SE3StateVar = steam::se3::SE3StateVar;
  const auto T_mq_var = SE3StateVar::MakeShared(SE3StateVar::T());
  problem.addStateVariable(T_mq_var);

  // shared noise and loss functions
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  const auto noise_model = std::make_shared<steam::StaticNoiseModel<3>>(cov);
  const auto loss_function = std::make_shared<steam::L2LossFunc>();

  // cost terms
  for (int i = 0; i < ref_pts.cols(); i++) {
    // Construct error function
    const auto error_function = steam::p2p::p2pError(
        T_mq_var, ref_pts.block<3, 1>(0, i), qry_pts.block<3, 1>(0, i));
    // Construct cost term
    const auto cost_term = std::make_shared<steam::WeightedLeastSqCostTerm<3>>(
        error_function, noise_model, loss_function);
    // Add cost term
    problem.addCostTerm(cost_term);
  }

  using SolverType = steam::VanillaGaussNewtonSolver;
  SolverType::Params params;
  params.verbose = true;

  // Make solver
  SolverType solver(&problem, params);

  // Optimize
  solver.optimize();

  std::cout << "true T_mq:\n" << T_mq << std::endl;
  std::cout << "estimated T_mq:\n" << T_mq_var->getValue() << std::endl;

  return 0;
}