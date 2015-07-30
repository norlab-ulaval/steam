//////////////////////////////////////////////////////////////////////////////////////////////
/// \file GaussNewtonSolverBase.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_GAUSS_NEWTON_SOLVER_HPP
#define STEAM_GAUSS_NEWTON_SOLVER_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include <steam/solver/SolverBase.hpp>

#include <steam/StateVector.hpp>
#include <steam/CostTerm.hpp>

#include <steam/sparse/BlockSparseMatrix.hpp>
#include <steam/sparse/BlockVector.hpp>

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

 protected:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Build the left-hand and right-hand sides of the Gauss-Newton system of equations
  //////////////////////////////////////////////////////////////////////////////////////////////
  void buildGaussNewtonTerms();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Solve the Gauss-Newton system of equations: A*x = b
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd solveGaussNewton();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Solve the Levenberg–Marquardt system of equations:
  ///        A*x = b, A = (J^T*J + diagonalCoeff*diag(J^T*J))
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd solveGaussNewtonForLM(double diagonalCoeff);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Find the Cauchy point (used for the Dogleg method).
  ///        The cauchy point is the optimal step length in the gradient descent direction.
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd getCauchyPoint() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the predicted cost reduction based on the proposed step
  //////////////////////////////////////////////////////////////////////////////////////////////
  double predictedReduction(const Eigen::VectorXd& step) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Build the system, solve for a step size and direction, and update the state
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool linearizeSolveAndUpdate(double* newCost) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Gauss-Newton left-hand side (upper triangular only)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::SparseMatrix<double> gaussNewtonLHS;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Gauss-Newton right-hand side
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd gaussNewtonRHS;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Whether or not the pattern of the LHS has been analyzed by the solver
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool patternInitialized_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief The solver object (stored over iterations to reuse the same pattern)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper> solver_;

};

} // steam

#endif // STEAM_GAUSS_NEWTON_SOLVER_HPP
