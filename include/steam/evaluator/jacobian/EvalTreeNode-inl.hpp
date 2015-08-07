//////////////////////////////////////////////////////////////////////////////////////////////
/// \file EvalTreeNode-inl.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/jacobian/EvalTreeNode.hpp>
#include <boost/make_shared.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE>
EvalTreeNode<TYPE>::EvalTreeNode() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE>
EvalTreeNode<TYPE>::EvalTreeNode(const TYPE& value)
  : EvalTreeNodeBase(), value_(value) {
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get current value
/////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE>
const TYPE& EvalTreeNode<TYPE>::getValue() const {
  return value_;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Set current value
/////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE>
void EvalTreeNode<TYPE>::setValue(const TYPE& value) {
  value_ = value;
}

} // steam