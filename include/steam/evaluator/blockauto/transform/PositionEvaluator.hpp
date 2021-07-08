//////////////////////////////////////////////////////////////////////////////////////////////
/// \file PositionEvaluator.hpp
///
/// \author Kai van Es, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_POSITION_EVALUATOR_HPP
#define STEAM_POSITION_EVALUATOR_HPP

#include <Eigen/Core>

#include <steam/evaluator/blockauto/transform/TransformEvaluator.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for the position component of a transformation matrix
///
/// *Note that we fix MAX_STATE_DIM to 6. Typically the performance benefits of fixed size
///  matrices begin to die if larger than 6x6. Size 6 allows for transformation matrices
///  and 6D velocities. If you have a state larger than this, consider writing an
///  error evaluator that extends from ErrorEvaluatorX.
//////////////////////////////////////////////////////////////////////////////////////////////
class PositionEvaluator : public BlockAutomaticEvaluator<Eigen::Matrix<double, 3, 1>, 3, 6> {
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<PositionEvaluator> Ptr;
  typedef boost::shared_ptr<const PositionEvaluator> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  PositionEvaluator(const TransformEvaluator::ConstPtr &transform);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const TransformEvaluator::ConstPtr &transform);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Adds references (shared pointers) to active state variables to the map output
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void getActiveStateVariables(
      std::map<unsigned int, steam::StateVariableBase::Ptr> *outStates) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double, 3, 1> evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra and
  ///        sub-tree of evaluations
  ///
  /// ** Note that the returned pointer belongs to the memory pool EvalTreeNode<TYPE>::pool,
  ///    and should be given back to the pool, rather than being deleted.
  //////////////////////////////////////////////////////////////////////////////////////////////
#ifdef STEAM_USE_OBJECT_POOL
  virtual EvalTreeNode<Eigen::Matrix<double, 3, 1>> *evaluateTree() const;
#else
  virtual EvalTreeNode<Eigen::Matrix<double, 3, 1>>::Ptr evaluateTree() const;
#endif

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the Jacobian tree
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void appendBlockAutomaticJacobians(const Eigen::MatrixXd &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                                             std::vector<Jacobian<> > *outJacobians) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Fixed-size evaluations of the Jacobian tree
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double, 1, 3> &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                                             std::vector<Jacobian<1, 6> > *outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double, 2, 3> &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                                             std::vector<Jacobian<2, 6> > *outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double, 3, 3> &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                                             std::vector<Jacobian<3, 6> > *outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double, 4, 3> &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                                             std::vector<Jacobian<4, 6> > *outJacobians) const;

  virtual void appendBlockAutomaticJacobians(const Eigen::Matrix<double, 6, 3> &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                                             EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                                             std::vector<Jacobian<6, 6> > *outJacobians) const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Implementation for Block Automatic Differentiation
  //////////////////////////////////////////////////////////////////////////////////////////////
  template<int LHS_DIM, int INNER_DIM, int MAX_STATE_SIZE>
  void appendJacobiansImpl(const Eigen::Matrix<double, LHS_DIM, INNER_DIM> &lhs,
#ifdef STEAM_USE_OBJECT_POOL
                           EvalTreeNode<Eigen::Matrix<double, 3, 1> > *evaluationTree,
#else
                           EvalTreeNode<Eigen::Matrix<double, 3, 1> >::Ptr evaluationTree,
#endif
                           std::vector<Jacobian<LHS_DIM, MAX_STATE_SIZE> > *outJacobians) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Transform evaluator
  //////////////////////////////////////////////////////////////////////////////////////////////
  TransformEvaluator::ConstPtr transform_;

};


} // se3
} // steam

#endif //STEAM_POSITION_EVALUATOR_HPP
