//////////////////////////////////////////////////////////////////////////////////////////////
/// \file WeightedLeastSqCostTerm.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_WEIGHTED_LSQ_COST_TERM_HPP
#define STEAM_WEIGHTED_LSQ_COST_TERM_HPP

#include <steam/problem/CostTermBase.hpp>

#include <steam/evaluator/ErrorEvaluator.hpp>
#include <steam/problem/NoiseModel.hpp>
#include <steam/problem/LossFunctions.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Class that fully defines a nonlinear cost term (or 'factor').
///        Cost terms are composed of an error function, loss function and noise model.
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM, int MAX_STATE_SIZE>
class WeightedLeastSqCostTerm : public CostTermBase
{
public:

  /// Convenience typedefs
  typedef std::shared_ptr<WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE> > Ptr;
  typedef std::shared_ptr<const WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE> > ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  WeightedLeastSqCostTerm(const typename ErrorEvaluator<MEAS_DIM,MAX_STATE_SIZE>::ConstPtr& errorFunction,
           const typename BaseNoiseModel<MEAS_DIM>::ConstPtr& noiseModel,
           const LossFunctionBase::ConstPtr& lossFunc);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the cost of this term. Error is first whitened by the noise model
  ///        and then passed through the loss function, as in:
  ///          cost = loss(sqrt(e^T * cov^{-1} * e))
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double cost() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns the number of cost terms contained by this object (typically 1)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual unsigned int numCostTerms() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not the implementation already uses multi-threading
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isImplParallelized() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Add the contribution of this cost term to the left-hand (Hessian) and right-hand
  ///        (gradient vector) sides of the Gauss-Newton system of equations.
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void buildGaussNewtonTerms(const StateVector& stateVector,
                                     BlockSparseMatrix* approximateHessian,
                                     BlockVector* gradientVector) const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the iteratively reweighted error vector and Jacobians. The error and
  ///        Jacobians are first whitened by the noise model and then weighted by the loss
  ///        function, as in:
  ///              error = sqrt(weight)*sqrt(cov^-1)*rawError
  ///           jacobian = sqrt(weight)*sqrt(cov^-1)*rawJacobian
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double,MEAS_DIM,1> evalWeightedAndWhitened(
      std::vector<Jacobian<MEAS_DIM,MAX_STATE_SIZE> >* outJacobians) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Error evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  typename ErrorEvaluator<MEAS_DIM,MAX_STATE_SIZE>::ConstPtr errorFunction_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Noise model
  //////////////////////////////////////////////////////////////////////////////////////////////
  typename BaseNoiseModel<MEAS_DIM>::ConstPtr noiseModel_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Loss function
  //////////////////////////////////////////////////////////////////////////////////////////////
  LossFunctionBase::ConstPtr lossFunc_;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Typedef for the general dynamic cost term
//////////////////////////////////////////////////////////////////////////////////////////////
typedef WeightedLeastSqCostTerm<Eigen::Dynamic, Eigen::Dynamic> WeightedLeastSqCostTermX;

} // steam

#include <steam/problem/WeightedLeastSqCostTerm-inl.hpp>

#endif // STEAM_WEIGHTED_LSQ_COST_TERM_HPP
