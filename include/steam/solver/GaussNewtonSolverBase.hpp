//////////////////////////////////////////////////////////////////////////////////////////////
/// \file GaussNewtonSolverBase.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_GAUSS_NEWTON_SOLVER_HPP
#define STEAM_GAUSS_NEWTON_SOLVER_HPP

#include <Eigen/Core>

#include <steam/blockmat/BlockSparseMatrix.hpp>
#include <steam/blockmat/BlockMatrix.hpp>
#include <steam/blockmat/BlockVector.hpp>
#include <steam/problem/StateVector.hpp>
#include <steam/solver/SolverBase.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Reports that the decomposition failed. This is due to poor conditioning, or
///        possibly that the matrix is not positive definite.
//////////////////////////////////////////////////////////////////////////////////////////////
class decomp_failure : public solver_failure
{
public:
  decomp_failure(const std::string& s) : solver_failure(s) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Intermediate solver interface that implements Gauss-Newton related solves
//////////////////////////////////////////////////////////////////////////////////////////////
class GaussNewtonSolverBase : public SolverBase
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  GaussNewtonSolverBase(OptimizationProblem* problem);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Query the covariance related to a single state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::MatrixXd queryCovariance(const steam::StateKey& key);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Query the covariance relating two state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::MatrixXd queryCovariance(const steam::StateKey& rowKey, const steam::StateKey& colKey);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Query a block of covariances
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockMatrix queryCovarianceBlock(const std::vector<steam::StateKey>& keys);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Query a block of covariances
  //////////////////////////////////////////////////////////////////////////////////////////////
  BlockMatrix queryCovarianceBlock(const std::vector<steam::StateKey>& rowKeys,
                                   const std::vector<steam::StateKey>& colKeys);

 protected:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Build the left-hand and right-hand sides of the Gauss-Newton system of equations
  //////////////////////////////////////////////////////////////////////////////////////////////
  void buildGaussNewtonTerms(Eigen::SparseMatrix<double>* approximateHessian,
                             Eigen::VectorXd* gradientVector);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Perform the LLT decomposition on the approx. Hessian matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  void factorizeHessian(const Eigen::SparseMatrix<double>& approximateHessian,
                        bool augmentedHessian = false);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Solve the Gauss-Newton system of equations: A*x = b
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd solveGaussNewton(const Eigen::SparseMatrix<double>& approximateHessian,
                                   const Eigen::VectorXd& gradientVector,
                                   bool augmentedHessian = false);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Find the Cauchy point (used for the Dogleg method).
  ///        The cauchy point is the optimal step length in the gradient descent direction.
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Eigen::VectorXd getCauchyPoint(const Eigen::SparseMatrix<double>& approximateHessian,
                                        const Eigen::VectorXd& gradientVector);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the predicted cost reduction based on the proposed step
  //////////////////////////////////////////////////////////////////////////////////////////////
  static double predictedReduction(const Eigen::SparseMatrix<double>& approximateHessian,
                                   const Eigen::VectorXd& gradientVector,
                                   const Eigen::VectorXd& step);

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Build the system, solve for a step size and direction, and update the state
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool linearizeSolveAndUpdate(double* newCost, double* gradNorm) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief The solver object (stored over iterations to reuse the same pattern)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper> hessianSolver_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Whether or not the pattern of the approx. Hessian has been analyzed by the solver
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool patternInitialized_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Whether or not the last factorization was for the information matrix and if
  ///        if it was successful. *Note that solving LM does not solve the information matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool factorizedInformationSuccesfully_;
};

} // steam

#endif // STEAM_GAUSS_NEWTON_SOLVER_HPP
