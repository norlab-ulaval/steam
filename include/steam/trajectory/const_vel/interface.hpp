#pragma once

#include <Eigen/Core>

#include "steam/problem/OptimizationProblem.hpp"
#include "steam/problem/cost_term/weighted_least_sq_cost_term.hpp"
#include "steam/trajectory/const_vel/variable.hpp"
#include "steam/trajectory/interface.hpp"
#include "steam/trajectory/time.hpp"
#include "steam/solver/GaussNewtonSolverBase.hpp"

namespace steam {
namespace traj {
namespace const_vel {

/**
 * \brief The trajectory class wraps a set of state variables to provide an
 * interface that allows for continuous-time pose interpolation.
 */
class Interface : public traj::Interface {
 public:
  /// Shared pointer typedefs for readability
  using Ptr = std::shared_ptr<Interface>;
  using ConstPtr = std::shared_ptr<const Interface>;

  using PoseType = lgmath::se3::Transformation;
  using VelocityType = Eigen::Matrix<double, 6, 1>;
  using CovType = Eigen::Matrix<double, 12, 12>;

  static Ptr MakeShared(const bool allowExtrapolation = false);
  static Ptr MakeShared(const Eigen::Matrix<double, 6, 6>& Qc_inv,
                        const bool allowExtrapolation = false);

  /**
   * \brief Constructor
   *        Note that the weighting matrix, Qc, should be provided if prior
   *        factors are needed for estimation. Without Qc the interpolation
   *        methods can be used safely.
   */
  Interface(const bool allowExtrapolation = false);
  Interface(const Eigen::Matrix<double, 6, 6>& Qc_inv,
            const bool allowExtrapolation = false);

  /** \brief Add a new knot */
  void add(const Variable::Ptr& knot);
  void add(const Time& time, const Evaluable<PoseType>::Ptr& T_k0,
           const Evaluable<VelocityType>::Ptr& w_0k_ink);
  // void add(const Time& time, const Evaluable<PoseType>::Ptr& T_k0,
  //          const Evaluable<VelocityType>::Ptr& w_0k_ink, const CovType& cov);

  /** \brief Get transform evaluator */
  Evaluable<PoseType>::ConstPtr getPoseInterpolator(const Time& time) const;

  /** \brief Get velocity evaluator */
  Evaluable<VelocityType>::ConstPtr getVelocityInterpolator(
      const Time& time) const;

  /** \brief Get state covariance using interpolation/extrapolation */
  Eigen::MatrixXd getCovariance(GaussNewtonSolverBase& solver,
      const Time& time);

  /**
   * \brief Queries Hessian for knot covariance and neighbouring (prev. & next)
   * cross-covariances (if they exist) and stores in the corresponding knots.
   */
  void queryKnotCovariance(GaussNewtonSolverBase& solver,
      std::map<Time, Variable::Ptr>::iterator it);

  /** \brief Reset covariance queries s.t. saved covariances are not re-used */
  void resetCovarianceQueries();

  /** \brief Set to save queried knot covariances */
  void setSaveCovariances(const bool& flag);

  /** \brief Interpolate covariance between two knot times */
  Eigen::MatrixXd interpCovariance(const Time& time, const Variable::Ptr& knot1,
      const Variable::Ptr& knot2) const;

  /** \brief Interpolate covariance between two knot times */
  Eigen::MatrixXd extrapCovariance(const Time& time,
    const Variable::Ptr& endKnot) const;

  /**
   * \brief Add a unary pose prior factor at a knot time. Note that only a
   * single pose prior should exist on a trajectory, adding a second will
   * overwrite the first.
   */
  void addPosePrior(const Time& time, const PoseType& T_k0,
                    const Eigen::Matrix<double, 6, 6>& cov);

  /**
   * \brief Add a unary velocity prior factor at a knot time. Note that only a
   * single velocity prior should exist on a trajectory, adding a second will
   * overwrite the first.
   */
  void addVelocityPrior(const Time& time, const VelocityType& w_0k_ink,
                        const Eigen::Matrix<double, 6, 6>& cov);

  /**
   * \brief Get binary cost terms associated with the prior for active parts of
   * the trajectory
   */
  void addPriorCostTerms(OptimizationProblem& problem) const override;

  /**
   * \brief Compute inverse covariance of prior factor
   */
  Eigen::Matrix<double, 12, 12> computeQinv(const double& deltatime) const;

 protected:
  /** \brief Ordered map of knots */
  Eigen::Matrix<double, 6, 6> Qc_inv_;

  /** \brief Allow for extrapolation */
  const bool allowExtrapolation_;

  /**
   * \brief Save queried knot covariances. Setting to true will make repeated
   * covariance interp./extrap. queries with the same knots more efficient. However,
   * resetCovarianceQueries() must manually be called to reset the saved covariances
   * if the optimization problem is modified and the knot covariances need updating.
   * Default setting is false.
   */
  bool saveCovariances_ = false;

  /** \brief Pose prior */
  std::vector<WeightedLeastSqCostTerm<6>::Ptr> pose_prior_factors_;

  /** \brief Velocity prior */
  std::vector<WeightedLeastSqCostTerm<6>::Ptr> velocity_prior_factors_;

  /** \brief Ordered map of knots */
  std::map<Time, Variable::Ptr> knotMap_;
};

}  // namespace const_vel
}  // namespace traj
}  // namespace steam