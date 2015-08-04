//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformEvalOperations.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/TransformEvalOperations.hpp>

#include <steam/evaluator/jacobian/JacobianTreeBranchNode.hpp>
#include <steam/evaluator/jacobian/JacobianTreeLeafNode.hpp>

#include <lgmath.hpp>

namespace steam {
namespace se3 {

/// Compose

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeTransformEvaluator::ComposeTransformEvaluator(const TransformEvaluator::ConstPtr& transform1,
                                                     const TransformEvaluator::ConstPtr& transform2)
  : transform1_(transform1), transform2_(transform2) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeTransformEvaluator::Ptr ComposeTransformEvaluator::MakeShared(const TransformEvaluator::ConstPtr& transform1,
                                                                     const TransformEvaluator::ConstPtr& transform2) {
  return ComposeTransformEvaluator::Ptr(new ComposeTransformEvaluator(transform1, transform2));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool ComposeTransformEvaluator::isActive() const {
  return transform1_->isActive() || transform2_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix (transform1*transform2)
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::se3::Transformation ComposeTransformEvaluator::evaluate() const {
  return transform1_->evaluate()*transform2_->evaluate();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::se3::Transformation ComposeTransformEvaluator::evaluate(std::vector<Jacobian>* jacs) const {

  // Evaluate
  std::vector<Jacobian> jacs1;
  lgmath::se3::Transformation transform1 = transform1_->evaluate(&jacs1);

  std::vector<Jacobian> jacs2;
  lgmath::se3::Transformation transform2 = transform2_->evaluate(&jacs2);

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();
  jacs->reserve(jacs1.size() + jacs2.size());

  // Jacobians 1
  jacs->insert(jacs->end(), jacs1.begin(), jacs1.end());
  unsigned int jacs1size = jacs->size();

  // Jacobians 2
  for (unsigned int j = 0; j < jacs2.size(); j++) {

    // Check if a jacobian w.r.t this state variable exists in 'other' half of transform evaluators
    // ** If a jacobian exists, we must add to it rather than creating a new entry
    unsigned int k = 0;
    for (; k < jacs1size; k++) {
      if (jacs->at(k).key.equals(jacs2[j].key)) {
        break;
      }
    }

    // Add jacobian information
    if (k < jacs1size) {
      // Add to existing entry
      jacs->at(k).jac += transform1.adjoint() * jacs2[j].jac; // todo .noalias() +=
    } else {
      // Create new entry
      jacs->push_back(Jacobian());
      Jacobian& jacref = jacs->back();
      jacref.key = jacs2[j].key;
      jacref.jac = transform1.adjoint() * jacs2[j].jac;
    }
  }

  // Return composition
  return transform1*transform2;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> ComposeTransformEvaluator::evaluateJacobians() const {

  // Evaluate
  std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> transform1 = transform1_->evaluateJacobians();
  std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> transform2 = transform2_->evaluateJacobians();

  // Init Jacobian node (null)
  JacobianTreeBranchNode::Ptr jacobianNode;

  // Check if evaluator is active
  if (this->isActive()) {

    // Make Jacobian node
    jacobianNode = JacobianTreeBranchNode::Ptr(new JacobianTreeBranchNode());

    // Check if transform1 is active
    if (transform1_->isActive()) {

      // Check for nullptr
      if (!transform1.second) {
        throw std::runtime_error("Active evaluator provided null JacobianTreeNode.");
      }

      // Add Jacobian
      jacobianNode->add(Eigen::Matrix<double,6,6>::Identity(), transform1.second);
    }

    // Check if transform2 is active
    if (transform2_->isActive()) {

      // Check for nullptr
      if (!transform2.second) {
        throw std::runtime_error("Active evaluator provided null JacobianTreeNode.");
      }

      // Add Jacobian
      jacobianNode->add(transform1.first.adjoint(), transform2.second);
    }
  }

  // Return
  return std::make_pair(transform1.first*transform2.first, jacobianNode);
}

/// Inverse

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
InverseTransformEvaluator::InverseTransformEvaluator(const TransformEvaluator::ConstPtr& transform) : transform_(transform) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
InverseTransformEvaluator::Ptr InverseTransformEvaluator::MakeShared(const TransformEvaluator::ConstPtr& transform) {
  return InverseTransformEvaluator::Ptr(new InverseTransformEvaluator(transform));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool InverseTransformEvaluator::isActive() const {
  return transform_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::se3::Transformation InverseTransformEvaluator::evaluate() const {
  return transform_->evaluate().inverse();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
//////////////////////////////////////////////////////////////////////////////////////////////
lgmath::se3::Transformation InverseTransformEvaluator::evaluate(std::vector<Jacobian>* jacs) const {

  // Evaluate
  std::vector<Jacobian> jacsCopy;
  lgmath::se3::Transformation transformInverse = transform_->evaluate(&jacsCopy).inverse();

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();
  jacs->resize(jacsCopy.size());

  // Jacobians
  for (unsigned int j = 0; j < jacsCopy.size(); j++) {
    Jacobian& jacref = jacs->at(j);
    jacref.key = jacsCopy[j].key;
    jacref.jac = -transformInverse.adjoint() * jacsCopy[j].jac;
  }

  return transformInverse;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant transformation matrix, and Jacobians w.r.t. state variables
//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> InverseTransformEvaluator::evaluateJacobians() const {

  // Evaluate
  std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> transform = transform_->evaluateJacobians();
  lgmath::se3::Transformation transformInverse = transform.first.inverse();

  // Init Jacobian node (null)
  JacobianTreeBranchNode::Ptr jacobianNode;

  // Check if evaluator is active
  if (this->isActive()) {

    // Make Jacobian node
    jacobianNode = JacobianTreeBranchNode::Ptr(new JacobianTreeBranchNode());

    // Check for nullptr
    if (!transform.second) {
      throw std::runtime_error("Active evaluator provided null JacobianTreeNode.");
    }

    // Add Jacobian
    jacobianNode->add(-transformInverse.adjoint(), transform.second);
  }

  // Return
  return std::make_pair(transformInverse, jacobianNode);
}

/// Log map

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
LogMapEvaluator::LogMapEvaluator(const TransformEvaluator::ConstPtr& transform) : transform_(transform) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
LogMapEvaluator::Ptr LogMapEvaluator::MakeShared(const TransformEvaluator::ConstPtr& transform) {
  return LogMapEvaluator::Ptr(new LogMapEvaluator(transform));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool LogMapEvaluator::isActive() const {
  return transform_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the resultant 6x1 vector belonging to the se(3) algebra
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> LogMapEvaluator::evaluate() const {
  return transform_->evaluate().vec();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 6x1 vector belonging to the se(3) algebra and relevant Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> LogMapEvaluator::evaluate(std::vector<Jacobian>* jacs) const {

  // Evaluate
  std::vector<Jacobian> jacsCopy;
  Eigen::Matrix<double,6,1> vec = transform_->evaluate(&jacsCopy).vec();

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();
  jacs->resize(jacsCopy.size());

  // Jacobians
  Eigen::Matrix<double,6,6> jacInv = lgmath::se3::vec2jacinv(vec);
  for (unsigned int j = 0; j < jacsCopy.size(); j++) {
    Jacobian& jacref = jacs->at(j);
    jacref.key = jacsCopy[j].key;
    jacref.jac = jacInv * jacsCopy[j].jac;
  }

  return vec;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 6x1 vector belonging to the se(3) algebra and relevant Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<Eigen::Matrix<double,6,1>, JacobianTreeNode::ConstPtr> LogMapEvaluator::evaluateJacobians() const {

  // Evaluate
  std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> transform = transform_->evaluateJacobians();
  Eigen::Matrix<double,6,1> vec = transform.first.vec();

  // Init Jacobian node (null)
  JacobianTreeBranchNode::Ptr jacobianNode;

  // Check if evaluator is active
  if (this->isActive()) {

    // Make Jacobian node
    jacobianNode = JacobianTreeBranchNode::Ptr(new JacobianTreeBranchNode());

    // Check for nullptr
    if (!transform.second) {
      throw std::runtime_error("Active evaluator provided null JacobianTreeNode.");
    }

    // Add Jacobian
    jacobianNode->add(lgmath::se3::vec2jacinv(vec), transform.second);
  }

  // Return
  return std::make_pair(vec, jacobianNode);
}


/// Landmark

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeLandmarkEvaluator::ComposeLandmarkEvaluator(const TransformEvaluator::ConstPtr& transform,
                                                   const se3::LandmarkStateVar::ConstPtr& landmark)
  : landmark_(landmark) {

  // Check if landmark has a reference frame and create pose evaluator
  if(landmark_->hasReferenceFrame()) {
    transform_ = ComposeTransformEvaluator::MakeShared(transform, InverseTransformEvaluator::MakeShared(landmark_->getReferenceFrame()));
  } else {
    transform_ = transform;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pseudo constructor - return a shared pointer to a new instance
//////////////////////////////////////////////////////////////////////////////////////////////
ComposeLandmarkEvaluator::Ptr ComposeLandmarkEvaluator::MakeShared(const TransformEvaluator::ConstPtr& transform,
                                                                   const se3::LandmarkStateVar::ConstPtr& landmark) {
  return ComposeLandmarkEvaluator::Ptr(new ComposeLandmarkEvaluator(transform, landmark));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool ComposeLandmarkEvaluator::isActive() const {
  return transform_->isActive() || !landmark_->isLocked();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the point transformed by the transform evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector4d ComposeLandmarkEvaluator::evaluate() const {
  return transform_->evaluate()*landmark_->getValue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the point transformed by the transform evaluator and relevant Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector4d ComposeLandmarkEvaluator::evaluate(std::vector<Jacobian>* jacs) const {

  // Evaluate
  std::vector<Jacobian> transformJacs;
  lgmath::se3::Transformation transform = transform_->evaluate(&transformJacs);

  Eigen::Vector4d point_in_c = transform * landmark_->getValue();

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();
  jacs->reserve(transformJacs.size() + 1);

  // 4 x 6 Transformation Jacobians
  for (unsigned int j = 0; j < transformJacs.size(); j++) {
    jacs->push_back(Jacobian());
    Jacobian& jacref = jacs->back();
    jacref.key = transformJacs[j].key;
    jacref.jac = lgmath::se3::point2fs(point_in_c.head<3>()) * transformJacs[j].jac;
  }

  // 4 x 3 Landmark Jacobian
  if(!landmark_->isLocked()) {
    jacs->push_back(Jacobian());
    Jacobian& jacref = jacs->back();
    jacref.key = landmark_->getKey();
    jacref.jac = transform.matrix() * Eigen::Matrix<double,4,3>::Identity();
    // todo... jac = transform.matrix().block<4,3>(0,0);
  }

  // Return error
  return point_in_c;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the point transformed by the transform evaluator and relevant Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
std::pair<Eigen::Vector4d, JacobianTreeNode::ConstPtr> ComposeLandmarkEvaluator::evaluateJacobians() const {

  // Evaluate
  std::pair<lgmath::se3::Transformation, JacobianTreeNode::ConstPtr> transform = transform_->evaluateJacobians();
  Eigen::Vector4d point_in_c = transform.first * landmark_->getValue();

  // Init Jacobian node (null)
  JacobianTreeBranchNode::Ptr jacobianNode;

  // Check if evaluator is active
  if (this->isActive()) {

    // Make Jacobian node
    jacobianNode = JacobianTreeBranchNode::Ptr(new JacobianTreeBranchNode());

    // Check if transform is active
    if (transform_->isActive()) {

      // Check for nullptr
      if (!transform.second) {
        throw std::runtime_error("Active evaluator provided null JacobianTreeNode.");
      }

      // Add Jacobian
      jacobianNode->add(lgmath::se3::point2fs(point_in_c.head<3>()), transform.second);
    }

    // Check if state is locked
    if (!landmark_->isLocked()) {

      // Make leaf node for Landmark
      JacobianTreeLeafNode::Ptr landmarkNode(new JacobianTreeLeafNode(landmark_));

      // Add Jacobian
      jacobianNode->add(transform.first.matrix() * Eigen::Matrix<double,4,3>::Identity(), landmarkNode);
      // todo... jac = transform.first.matrix().block<4,3>(0,0);
    }

  }

  // Return
  return std::make_pair(point_in_c, jacobianNode);
}

} // se3
} // steam
