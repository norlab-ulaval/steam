//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformEvalOperations.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_TRANSFORM_EVALUATOR_OPERATIONS_HPP
#define STEAM_TRANSFORM_EVALUATOR_OPERATIONS_HPP

#include <Eigen/Core>

#include <steam/evaluator/EvaluatorBase.hpp>
#include <steam/evaluator/TransformEvaluators.hpp>
#include <steam/state/LandmarkStateVar.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for the composition of transformation matrices
//////////////////////////////////////////////////////////////////////////////////////////////
class ComposeTransformEvaluator : public TransformEvaluator
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<ComposeTransformEvaluator> Ptr;
  typedef boost::shared_ptr<const ComposeTransformEvaluator> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  ComposeTransformEvaluator(const TransformEvaluator::ConstPtr& transform1, const TransformEvaluator::ConstPtr& transform2);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const TransformEvaluator::ConstPtr& transform1, const TransformEvaluator::ConstPtr& transform2);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant transformation matrix (transform1*transform2)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate(std::vector<Jacobian>* jacs) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> evaluateJacobians() const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief First transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  TransformEvaluator::ConstPtr transform1_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Second transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  TransformEvaluator::ConstPtr transform2_;

};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for the inverse of a transformation matrix
//////////////////////////////////////////////////////////////////////////////////////////////
class InverseTransformEvaluator : public TransformEvaluator
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<InverseTransformEvaluator> Ptr;
  typedef boost::shared_ptr<const InverseTransformEvaluator> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  InverseTransformEvaluator(const TransformEvaluator::ConstPtr& transform);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const TransformEvaluator::ConstPtr& transform);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant transformation matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate(std::vector<Jacobian>* jacs) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> evaluateJacobians() const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  TransformEvaluator::ConstPtr transform_;

};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for the logarithmic map of a transformation matrix
//////////////////////////////////////////////////////////////////////////////////////////////
class LogMapEvaluator : public Vector6dEvaluator
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<LogMapEvaluator> Ptr;
  typedef boost::shared_ptr<const LogMapEvaluator> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  LogMapEvaluator(const TransformEvaluator::ConstPtr& transform);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const TransformEvaluator::ConstPtr& transform);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,6,1> evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 6x1 vector belonging to the se(3) algebra and relevant Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double,6,1> evaluate(std::vector<Jacobian>* jacs) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 6x1 vector belonging to the se(3) algebra and relevant Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual std::pair<Eigen::Matrix<double,6,1>, JacobianTreeNode::ConstPtr> evaluateJacobians() const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  TransformEvaluator::ConstPtr transform_;

};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for the composition of a transformation evaluator and landmark state
//////////////////////////////////////////////////////////////////////////////////////////////
class ComposeLandmarkEvaluator : public Vector4dEvaluator
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<ComposeLandmarkEvaluator> Ptr;
  typedef boost::shared_ptr<const ComposeLandmarkEvaluator> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  ComposeLandmarkEvaluator(const TransformEvaluator::ConstPtr& transform, const se3::LandmarkStateVar::ConstPtr& landmark);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const TransformEvaluator::ConstPtr& transform, const se3::LandmarkStateVar::ConstPtr& landmark);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the point transformed by the transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Vector4d evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the point transformed by the transform evaluator and relevant Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Vector4d evaluate(std::vector<Jacobian>* jacs) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 6x1 vector belonging to the se(3) algebra and relevant Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual std::pair<Eigen::Vector4d, JacobianTreeNode::ConstPtr> evaluateJacobians() const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  TransformEvaluator::ConstPtr transform_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Landmark state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  se3::LandmarkStateVar::ConstPtr landmark_;

};


/// Quick Ops

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compose two transform evaluators
//////////////////////////////////////////////////////////////////////////////////////////////
static TransformEvaluator::Ptr compose(const TransformEvaluator::ConstPtr& transform1,
                                       const TransformEvaluator::ConstPtr& transform2) {
  return ComposeTransformEvaluator::MakeShared(transform1, transform2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compose a transform evaluator and landmark state variable
//////////////////////////////////////////////////////////////////////////////////////////////
static Vector4dEvaluator::Ptr compose(const TransformEvaluator::ConstPtr& transform, const se3::LandmarkStateVar::ConstPtr& landmark) {
  return ComposeLandmarkEvaluator::MakeShared(transform, landmark);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Invert a transform evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
static TransformEvaluator::Ptr inverse(const TransformEvaluator::ConstPtr& transform) {
  return InverseTransformEvaluator::MakeShared(transform);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Take the 'logarithmic map' of a transformation evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
static Vector6dEvaluator::Ptr tran2vec(const TransformEvaluator::ConstPtr& transform) {
  return LogMapEvaluator::MakeShared(transform);
}

} // se3
} // steam

#endif // STEAM_TRANSFORM_EVALUATOR_OPERATIONS_HPP
