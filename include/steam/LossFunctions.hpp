//////////////////////////////////////////////////////////////////////////////////////////////
/// \file LossFunctions.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_LOSS_FUNCTIONS_HPP
#define STEAM_LOSS_FUNCTIONS_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include <steam/NoiseModel.hpp>

// to do
// loss functions must implement  cost (rho) and weight (w)
// l2 -- e^2, e, 1
// huber -- if abs(e) < k, e^2, e, 1 : else k*(abs(e)-e/2) - e/2, k*sgn(e), k/abs(e)
// cauchy -- e^2/2 * log(1+e^2/c), e/(1+(e/c)^2), 1/(1+(e/c)^2)

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Base loss function class
//////////////////////////////////////////////////////////////////////////////////////////////
class LossFunction
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<LossFunction> Ptr;
  typedef boost::shared_ptr<const LossFunction> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  LossFunction() {}

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Cost function (basic evaluation of the loss function)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double cost(double whitened_error_norm) const = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double weight(double whitened_error_norm) const = 0;

};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief 'L2' loss function
//////////////////////////////////////////////////////////////////////////////////////////////
class L2LossFunc : public LossFunction
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<L2LossFunc> Ptr;
  typedef boost::shared_ptr<const L2LossFunc> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  L2LossFunc() {}

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Cost function (basic evaluation of the loss function)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double cost(double whitened_error_norm) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double weight(double whitened_error_norm) const;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Huber loss function class
//////////////////////////////////////////////////////////////////////////////////////////////
class HuberLossFunc : public LossFunction
{
 public:

  /// Convenience typedefs
  typedef boost::shared_ptr<HuberLossFunc> Ptr;
  typedef boost::shared_ptr<const HuberLossFunc> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor -- k is the `threshold' based on number of std devs (1-3 is typical)
  //////////////////////////////////////////////////////////////////////////////////////////////
  HuberLossFunc(double k) : k_(k) {}

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Cost function (basic evaluation of the loss function)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double cost(double whitened_error_norm) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double weight(double whitened_error_norm) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Huber constant
  //////////////////////////////////////////////////////////////////////////////////////////////
  double k_;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Huber loss function class
//////////////////////////////////////////////////////////////////////////////////////////////
class DcsLossFunc : public LossFunction
{
 public:

  /// Convenience typedefs
  typedef boost::shared_ptr<DcsLossFunc> Ptr;
  typedef boost::shared_ptr<const DcsLossFunc> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor -- k is the `threshold' based on number of std devs (1-3 is typical)
  //////////////////////////////////////////////////////////////////////////////////////////////
  DcsLossFunc(double k) : k2_(k*k) {}

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Cost function (basic evaluation of the loss function)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double cost(double whitened_error_norm) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual double weight(double whitened_error_norm) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Huber constant
  //////////////////////////////////////////////////////////////////////////////////////////////
  double k2_;
};

} // steam

#endif // STEAM_LOSS_FUNCTIONS_HPP
