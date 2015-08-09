//////////////////////////////////////////////////////////////////////////////////////////////
/// \file OptimizationProblem.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_OPTIMIZATION_PROBLEM_HPP
#define STEAM_OPTIMIZATION_PROBLEM_HPP

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <steam/state/StateVector.hpp>
#include <steam/problem/CostTermCollection.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Container for active state variables and cost terms associated with the
///        optimization problem to be solved.
//////////////////////////////////////////////////////////////////////////////////////////////
class OptimizationProblem
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  OptimizationProblem();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Add an 'active' state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  void addStateVariable(const StateVariableBase::Ptr& statevar);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Add a cost term (should depend on active states that were added to the problem)
  //////////////////////////////////////////////////////////////////////////////////////////////
  void addCostTerm(const CostTermX::ConstPtr& costTerm);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Compute the cost from the collection of cost terms
  //////////////////////////////////////////////////////////////////////////////////////////////
  double cost() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get a reference to the state vector
  //////////////////////////////////////////////////////////////////////////////////////////////
  const StateVector& getStateVector() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get a reference to the cost terms
  //////////////////////////////////////////////////////////////////////////////////////////////
  const std::vector<CostTermX::ConstPtr>& getCostTerms() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Fill in the supplied block matrices
  //////////////////////////////////////////////////////////////////////////////////////////////
  void buildGaussNewtonTerms(Eigen::SparseMatrix<double>* approximateHessian,
                             Eigen::VectorXd* gradientVector) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Propose an update to the state vector.
  //////////////////////////////////////////////////////////////////////////////////////////////
  double proposeUpdate(const Eigen::VectorXd& stateStep);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Confirm the proposed state update
  //////////////////////////////////////////////////////////////////////////////////////////////
  void acceptProposedState();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Reject the proposed state update and revert to the previous values
  //////////////////////////////////////////////////////////////////////////////////////////////
  void rejectProposedState();

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Disabled copy constructor. Cost terms contain internal pointers to the state
  ///        variables, making deep copy useless. At this time it is not clear whether a
  ///        shallow copy provides any value.
  //////////////////////////////////////////////////////////////////////////////////////////////
  OptimizationProblem(const OptimizationProblem&);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief collection of nonlinear cost-term factors
  //////////////////////////////////////////////////////////////////////////////////////////////
  //std::vector<CostTermX::ConstPtr> costTerms_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Collection of dynamic-size nonlinear cost-term factors
  //////////////////////////////////////////////////////////////////////////////////////////////
  CostTermCollectionX dynamicCostTerms_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief List of custom-size nonlinear cost-term collections
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<CostTermCollectionBase::ConstPtr> customCostTerms_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief collection of state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  StateVector stateVec_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Since many solvers need to test the state vector after an update before deciding
  ///        if the update is appropriate, we implement a way to 'temporarily' modify the
  ///        state, and then either confirm the change, or reject it and revert to the
  ///        original values. These variables are used to backup the state vector while
  ///        performing a temporary update.
  //////////////////////////////////////////////////////////////////////////////////////////////
  StateVector stateVectorBackup_;
  bool firstBackup_;
  bool pendingProposedState_;
};

} // namespace steam

#endif // STEAM_OPTIMIZATION_PROBLEM_HPP
