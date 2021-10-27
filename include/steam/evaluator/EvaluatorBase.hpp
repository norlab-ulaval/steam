//////////////////////////////////////////////////////////////////////////////////////////////
/// \file EvaluatorBase.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_EVALUATOR_BASE_HPP
#define STEAM_EVALUATOR_BASE_HPP

#include <Eigen/Core>

#include <steam/state/StateVector.hpp>
#include <steam/problem/Jacobian.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Base class that defines the general 'evaluator' interface
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename TYPE,                      // The output 'evaluation' type
          int LHS_DIM       = Eigen::Dynamic, // The LHS dim of the 'most' left-ward
                                              // Jacobian in the evaluation tree
          int INNER_DIM     = Eigen::Dynamic, // The LHS dimension of 'this' evaluators Jacobian
          int MAX_STATE_DIM = Eigen::Dynamic> // The maximum dimension of a single state variable
                                              // perturbation (aka the 'most' RHS Jacobian dim)
class EvaluatorBase
{
 public:

  /// Convenience typedefs
  typedef std::shared_ptr<EvaluatorBase<TYPE,LHS_DIM,INNER_DIM,MAX_STATE_DIM> > Ptr;
  typedef std::shared_ptr<const EvaluatorBase<TYPE,LHS_DIM,INNER_DIM,MAX_STATE_DIM> > ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  EvaluatorBase() {}

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interface for the general 'evaluation'
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual TYPE evaluate() const = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interface for the general 'evaluation', with Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual TYPE evaluate(const Eigen::Matrix<double, LHS_DIM, INNER_DIM>& lhs,
                        std::vector<Jacobian<LHS_DIM, MAX_STATE_DIM> >* jacs) const = 0;
};

} // steam

#endif // STEAM_EVALUATOR_BASE_HPP
