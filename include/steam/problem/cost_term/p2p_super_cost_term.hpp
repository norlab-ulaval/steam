#pragma once

#include "steam/evaluable/se3/se3_state_var.hpp"
#include "steam/evaluable/state_var.hpp"
#include "steam/evaluable/vspace/vspace_state_var.hpp"
#include "steam/problem/cost_term/base_cost_term.hpp"
#include "steam/problem/loss_func/loss_funcs.hpp"
#include "steam/problem/problem.hpp"
#include "steam/trajectory/const_acc/interface.hpp"
#include "steam/trajectory/time.hpp"

namespace steam {

struct P2PMatch {
  double timestamp = 0;
  Eigen::Vector3d reference = Eigen::Vector3d::Zero();  // map frame
  Eigen::Vector3d normal = Eigen::Vector3d::Ones();     // map frame
  Eigen::Vector3d query = Eigen::Vector3d::Zero();      // robot frame

  P2PMatch(double timestamp_, Eigen::Vector3d reference_,
           Eigen::Vector3d normal_, Eigen::Vector3d query_)
      : timestamp(timestamp_),
        reference(reference_),
        normal(normal_),
        query(query_) {}
};

class P2PSuperCostTerm : public BaseCostTerm {
 public:
  enum class LOSS_FUNC { L2, DCS, CAUCHY, GM };

  struct Options {
    int num_threads = 1;
    LOSS_FUNC p2p_loss_func = LOSS_FUNC::CAUCHY;
    double p2p_loss_sigma = 0.1;
    Eigen::Matrix4d T_sr = Eigen::Matrix4d::Identity();
  };

  using Ptr = std::shared_ptr<P2PSuperCostTerm>;
  using ConstPtr = std::shared_ptr<const P2PSuperCostTerm>;

  using PoseType = lgmath::se3::Transformation;
  using VelType = Eigen::Matrix<double, 6, 1>;
  using AccType = Eigen::Matrix<double, 6, 1>;

  using Interface = steam::traj::const_acc::Interface;

  using Variable = steam::traj::const_acc::Variable;

  using Time = steam::traj::Time;

  using Matrix18d = Eigen::Matrix<double, 18, 18>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  static Ptr MakeShared(const Interface::ConstPtr &interface, const Time &time1,
                        const Time &time2, Options options);
  P2PSuperCostTerm(const Interface::ConstPtr &interface, const Time &time1,
                   const Time &time2, Options options);

  /** \brief Compute the cost to the objective function */
  double cost() const override;

  /** \brief Get keys of variables related to this cost term */
  void getRelatedVarKeys(KeySet &keys) const override;

  void setP2PMatches(std::vector<P2PMatch> *p2p_matches);

  /**
   * \brief Add the contribution of this cost term to the left-hand (Hessian)
   * and right-hand (gradient vector) sides of the Gauss-Newton system of
   * equations.
   */
  void buildGaussNewtonTerms(const StateVector &state_vec,
                             BlockSparseMatrix *approximate_hessian,
                             BlockVector *gradient_vector) const override;

 private:
  const Interface::ConstPtr interface_;
  const Time &time1_;
  const Time &time2_;
  Variable::ConstPtr knot1_;
  Variable::ConstPtr knot2_;
  Options options_;
  Matrix18d Qinv_T_ = Matrix18d::Identity();
  Matrix18d Tran_T_ = Matrix18d::Identity();
  std::map<double, std::pair<Matrix18d, Matrix18d>> interp_mats_;

  std::vector<P2PMatch> *p2p_matches_ = nullptr;
  std::map<double, std::vector<int>> p2p_match_bins_;

  BaseLossFunc::Ptr p2p_loss_func_ = L2LossFunc::MakeShared();

  void initialize_interp_matrices_();
};

}  // namespace steam
