//////////////////////////////////////////////////////////////////////////////////////////////
/// \file VanillaGaussNewtonSolver.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/solver/VanillaGaussNewtonSolver.hpp>

#include <iostream>

#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
VanillaGaussNewtonSolver::VanillaGaussNewtonSolver(OptimizationProblem* problem, const Params& params)
  : GaussNewtonSolverBase(problem), params_(params) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Build the system, solve for a step size and direction, and update the state
//////////////////////////////////////////////////////////////////////////////////////////////
bool VanillaGaussNewtonSolver::linearizeSolveAndUpdate(double* newCost) {

  if (newCost == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input "
                                "'newCost' in linearizeSolveAndUpdate");
  }

  steam::Timer iterTimer;
  steam::Timer timer;
  double buildTime = 0;
  double solveTime = 0;
  double updateTime = 0;

  // Construct system of equations
  timer.reset();
  this->buildGaussNewtonTerms();
  buildTime = timer.milliseconds();

  // Solve system
  timer.reset();
  Eigen::VectorXd perturbation = this->solveGaussNewton();
  solveTime = timer.milliseconds();

  // Apply update
  timer.reset();
  *newCost = this->getProblem().proposeUpdate(perturbation);
  this->getProblem().acceptProposedState();
  updateTime = timer.milliseconds();

  // Print report line if verbose option enabled
  if (params_.verbose) {
    if (this->getCurrIteration() == 1) {
        std::cout  << std::right << std::setw( 4) << std::setfill(' ') << "iter"
                   << std::right << std::setw(12) << std::setfill(' ') << "cost"
                   << std::right << std::setw(12) << std::setfill(' ') << "build (ms)"
                   << std::right << std::setw(12) << std::setfill(' ') << "solve (ms)"
                   << std::right << std::setw(13) << std::setfill(' ') << "update (ms)"
                   << std::right << std::setw(11) << std::setfill(' ') << "time (ms)"
                   << std::endl;
    }
    std::cout << std::right << std::setw(4)  << std::setfill(' ') << this->getCurrIteration()
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(5) << *newCost
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(3) << std::fixed << buildTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(3) << std::fixed << solveTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(13) << std::setfill(' ') << std::setprecision(3) << std::fixed << updateTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(11) << std::setfill(' ') << std::setprecision(3) << std::fixed << iterTimer.milliseconds() << std::resetiosflags(std::ios::fixed)
              << std::endl;
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Casts parameters to base type (for SolverBase class)
//////////////////////////////////////////////////////////////////////////////////////////////
const SolverBase::Params& VanillaGaussNewtonSolver::getSolverBaseParams() const {
  return params_;
}

} // steam
