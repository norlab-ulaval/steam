#include "steam/trajectory/const_acc/pose_interpolator.hpp"

#include "steam/evaluable/se3/evaluables.hpp"
#include "steam/evaluable/vspace/evaluables.hpp"
#include "steam/trajectory/const_acc/evaluable/compose_curlyhat_evaluator.hpp"
#include "steam/trajectory/const_acc/helper.hpp"
#include "steam/trajectory/const_vel/evaluable/jinv_velocity_evaluator.hpp"

namespace steam {
namespace traj {
namespace const_acc {

PoseInterpolator::Ptr PoseInterpolator::MakeShared(
    const Time& time, const Variable::ConstPtr& knot1,
    const Variable::ConstPtr& knot2) {
  return std::make_shared<PoseInterpolator>(time, knot1, knot2);
}

PoseInterpolator::PoseInterpolator(const Time& time,
                                   const Variable::ConstPtr& knot1,
                                   const Variable::ConstPtr& knot2)
    : knot1_(knot1), knot2_(knot2) {
  // Calculate time constants
  const double T = (knot2->time() - knot1->time()).seconds();
  const double tau = (time - knot1->time()).seconds();
  const double kappa = (knot2->time() - time).seconds();

  // Q and Transition matrix
  const auto Q_tau = getQ(tau);
  const auto Qinv_T = getQinv(T);
  const auto Tran_kappa = getTran(kappa);
  const auto Tran_tau = getTran(tau);
  const auto Tran_T = getTran(T);

  // Calculate interpolation values
  Eigen::Matrix<double, 3, 3> Omega(Q_tau * Tran_kappa.transpose() * Qinv_T);
  Eigen::Matrix<double, 3, 3> Lambda(Tran_tau - Omega * Tran_T);

  // construct computation graph
  const auto T1 = knot1_->pose();
  const auto w1 = knot1_->velocity();
  const auto dw1 = knot1_->acceleration();
  const auto T2 = knot2_->pose();
  const auto w2 = knot2_->velocity();
  const auto dw2 = knot2_->acceleration();

  using namespace steam::se3;
  using namespace steam::vspace;

  // clang-format off

  // Get relative matrix info
  const auto T_21 = compose_rinv(T2, T1);
  // Get se3 algebra of relative matrix
  const auto xi_21 = tran2vec(T_21);
  //
  const auto gamma11 = w1;
  const auto gamma12 = dw1;
  const auto gamma20 = xi_21;
  const auto gamma21 = const_vel::jinv_velocity(xi_21, w2);
  const auto gamma22 = add<6>(smult<6>(compose_curlyhat(const_vel::jinv_velocity(xi_21, w2), w2), -0.5), const_vel::jinv_velocity(xi_21, dw2));

  // pose
  const auto _t1 = smult<6>(gamma11, Lambda(0, 1));
  const auto _t2 = smult<6>(gamma12, Lambda(0, 2));
  const auto _t3 = smult<6>(gamma20, Omega(0, 0));
  const auto _t4 = smult<6>(gamma21, Omega(0, 1));
  const auto _t5 = smult<6>(gamma22, Omega(0, 2));
  const auto xi_i1 = add<6>(_t1, add<6>(_t2, add<6>(_t3, add<6>(_t4, _t5))));

  // calculate interpolated relative transformation matrix
  const auto T_i1 = vec2tran(xi_i1);
  // compose to get global transform
  T_i0_ = compose(T_i1, T1);

  // clang-format on
}

bool PoseInterpolator::active() const {
  return knot1_->pose()->active() || knot1_->velocity()->active() ||
         knot1_->acceleration()->active() || knot2_->pose()->active() ||
         knot2_->velocity()->active() || knot2_->acceleration()->active();
}

void PoseInterpolator::getRelatedVarKeys(KeySet& keys) const {
  knot1_->pose()->getRelatedVarKeys(keys);
  knot1_->velocity()->getRelatedVarKeys(keys);
  knot1_->acceleration()->getRelatedVarKeys(keys);
  knot2_->pose()->getRelatedVarKeys(keys);
  knot2_->velocity()->getRelatedVarKeys(keys);
  knot2_->acceleration()->getRelatedVarKeys(keys);
}

auto PoseInterpolator::value() const -> OutType { return T_i0_->value(); }

auto PoseInterpolator::forward() const -> Node<OutType>::Ptr {
  return T_i0_->forward();
}

void PoseInterpolator::backward(const Eigen::MatrixXd& lhs,
                                const Node<OutType>::Ptr& node,
                                Jacobians& jacs) const {
  return T_i0_->backward(lhs, node, jacs);
}

}  // namespace const_acc
}  // namespace traj
}  // namespace steam