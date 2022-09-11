#pragma once

#include <Eigen/Core>

#include "steam/problem/cost_term/weighted_least_sq_cost_term.hpp"
#include "steam/problem/problem.hpp"
#include "steam/solver/covariance.hpp"
#include "steam/trajectory/const_acc/variable.hpp"
#include "steam/trajectory/interface.hpp"
#include "steam/trajectory/time.hpp"

namespace steam {
namespace traj {
namespace const_acc {

class Interface : public traj::Interface {
 public:
  using Ptr = std::shared_ptr<Interface>;
  using ConstPtr = std::shared_ptr<const Interface>;

  using PoseType = lgmath::se3::Transformation;
  using VelocityType = Eigen::Matrix<double, 6, 1>;
  using AccelerationType = Eigen::Matrix<double, 6, 1>;
  using CovType = Eigen::Matrix<double, 18, 18>;

  static Ptr MakeShared(const Eigen::Matrix<double, 6, 1>& Qc_diag =
                            Eigen::Matrix<double, 6, 1>::Ones());

  Interface(const Eigen::Matrix<double, 6, 1>& Qc_diag =
                Eigen::Matrix<double, 6, 1>::Ones());

  void add(const Time& time, const Evaluable<PoseType>::Ptr& T_k0,
           const Evaluable<VelocityType>::Ptr& w_0k_ink,
           const Evaluable<AccelerationType>::Ptr& dw_0k_ink);

  // clang-format off
  Evaluable<PoseType>::ConstPtr getPoseInterpolator(const Time& time) const;
  Evaluable<VelocityType>::ConstPtr getVelocityInterpolator(const Time& time) const;

  void addPosePrior(const Time& time, const PoseType& T_k0, const Eigen::Matrix<double, 6, 6>& cov);
  void addVelocityPrior(const Time& time, const VelocityType& w_0k_ink, const Eigen::Matrix<double, 6, 6>& cov);
  void addAccelerationPrior(const Time& time, const VelocityType& dw_0k_ink, const Eigen::Matrix<double, 6, 6>& cov);
  // clang-format on

  void addPriorCostTerms(Problem& problem) const;

 private:
  Eigen::Matrix<double, 6, 1> Qc_diag_;
  std::map<Time, Variable::Ptr> knot_map_;
  WeightedLeastSqCostTerm<6>::Ptr pose_prior_factor_ = nullptr;
  WeightedLeastSqCostTerm<6>::Ptr vel_prior_factor_ = nullptr;
  WeightedLeastSqCostTerm<6>::Ptr acc_prior_factor_ = nullptr;
};

}  // namespace const_acc
}  // namespace traj
}  // namespace steam