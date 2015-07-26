//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SolverBase.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_SOLVER_BASE_HPP
#define STEAM_SOLVER_BASE_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <steam/OptimizationProblem.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Reports solver failures (e.g. LLT decomposition fail). Catch and handle properly
//////////////////////////////////////////////////////////////////////////////////////////////
class steam_solver_failure : public std::runtime_error
{
public:
    steam_solver_failure(const std::string& s) : std::runtime_error(s) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Basic solver interface
//////////////////////////////////////////////////////////////////////////////////////////////
class SolverBase
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Basic solver parameters
  //////////////////////////////////////////////////////////////////////////////////////////////
  struct Params {
    Params() : verbose(false), maxIterations(100), absoluteCostThreshold(0.0),
      absoluteCostChangeThreshold(1e-4), relativeCostChangeThreshold(1e-4) {
      // check thresholds greater than zero?
      // check that max iterations is not inf?
      // inf for thresholds is okay? .. maybe not for absolute cost thresh...
    }

    /// Whether the solver should be verbose
    bool verbose; // false

    /// Maximum iterations
    unsigned int maxIterations; // 500

    /// Absolute cost threshold to trigger convergence (cost is less than x)
    double absoluteCostThreshold; // 0.0

    /// Change in cost threshold to trigger convergence (cost went down by less than x)
    double absoluteCostChangeThreshold; // 1e-4

    /// Relative cost threshold to trigger convergence (costChange/oldCost is less than x)
    double relativeCostChangeThreshold; // 1e-4
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Termination causes
  //////////////////////////////////////////////////////////////////////////////////////////////
  enum Termination {
    TERMINATE_NOT_YET_TERMINATED,
    TERMINATE_STEP_UNSUCCESSFUL,
    TERMINATE_MAX_ITERATIONS,
    TERMINATE_CONVERGED_ABSOLUTE_ERROR,
    TERMINATE_CONVERGED_ABSOLUTE_CHANGE,
    TERMINATE_CONVERGED_RELATIVE_CHANGE
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  SolverBase(OptimizationProblem* problem);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not the solver is converged
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool converged() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Perform an iteration of the solver
  //////////////////////////////////////////////////////////////////////////////////////////////
  void iterate();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Perform iterations until convergence
  ///        This function is made to be simple and require no private methods so that users
  ///        can choose to control the loop themselves.
  //////////////////////////////////////////////////////////////////////////////////////////////
  void optimize();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Return termination cause
  //////////////////////////////////////////////////////////////////////////////////////////////
  Termination getTerminationCause();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Return current iteration number
  //////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int getCurrIteration() const;

 protected:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Return previous iteration cost evaluation
  //////////////////////////////////////////////////////////////////////////////////////////////
  double getPrevCost() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get reference to optimization problem
  //////////////////////////////////////////////////////////////////////////////////////////////
  OptimizationProblem& getProblem();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get const reference to optimization problem
  //////////////////////////////////////////////////////////////////////////////////////////////
  const OptimizationProblem& getProblem() const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Build the system, solve for a step size and direction, and update the state
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool linearizeSolveAndUpdate(double* newCost) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Casts parameters to base type (for SolverBase class)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual const SolverBase::Params& getSolverBaseParams() const = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Reference to optimization problem
  //////////////////////////////////////////////////////////////////////////////////////////////
  OptimizationProblem* problem_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Current iteration number
  //////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int currIteration_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Current cost evaluation of the cost terms
  //////////////////////////////////////////////////////////////////////////////////////////////
  double currCost_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Previous cost evaluation of the cost terms
  //////////////////////////////////////////////////////////////////////////////////////////////
  double prevCost_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Whether or not the solver has converged
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool solverConverged_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Termination cause
  //////////////////////////////////////////////////////////////////////////////////////////////
  Termination term_;

};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Print termination cause
//////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& out, const SolverBase::Termination& T);

} // steam

#endif // STEAM_SOLVER_BASE_HPP
