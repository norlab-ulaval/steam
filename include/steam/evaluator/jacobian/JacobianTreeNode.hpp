//////////////////////////////////////////////////////////////////////////////////////////////
/// \file JacobianTreeNode.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_JACOBIAN_TREE_NODE_HPP
#define STEAM_JACOBIAN_TREE_NODE_HPP

#include <vector>
#include <Eigen/Dense>
#include <steam/evaluator/jacobian/Jacobian.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Simple structure to hold Jacobian information
//////////////////////////////////////////////////////////////////////////////////////////////
class JacobianTreeNode
{

 public:

  /// Convenience typedefs
  typedef boost::shared_ptr<JacobianTreeNode > Ptr;
  typedef boost::shared_ptr<const JacobianTreeNode > ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  JacobianTreeNode();

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get the Jacobians with respect to leaf state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  std::vector<Jacobian> getJacobians() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Traverse the Jacobian tree and calculate the Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void append(std::vector<Jacobian>* outJacobians) const = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Traverse the Jacobian tree and calculate the Jacobians, pre-multiplied by lhs
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual void append(const Eigen::MatrixXd& lhs, std::vector<Jacobian>* outJacobians) const = 0;

 protected:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Go through vector of Jacobians and check for Jacobians which are with respect to
  ///        the same state variable, and merge them.
  ///
  /// For efficiency, specify a hintIndex, which specifies that Jacobians before hintIndex
  /// cannot be multiples of eachother.
  //////////////////////////////////////////////////////////////////////////////////////////////
  static void merge(std::vector<Jacobian>* outJacobians, unsigned int hintIndex);

};

} // steam

#endif // STEAM_JACOBIAN_TREE_NODE_HPP
