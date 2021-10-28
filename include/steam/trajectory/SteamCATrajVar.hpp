//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SteamCATrajVar.hpp
///
/// \author Tim Tang, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_CA_TRAJECTORY_VAR_HPP
#define STEAM_CA_TRAJECTORY_VAR_HPP

#include <Eigen/Core>

#include <steam/common/Time.hpp>
#include <steam/evaluator/blockauto/transform/TransformEvaluator.hpp>
#include <steam/state/VectorSpaceStateVar.hpp>

#include <steam/trajectory/SteamTrajVar.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This class wraps a pose and velocity evaluator to act as a discrete-time trajectory
///        state variable for continuous-time trajectory estimation.
//////////////////////////////////////////////////////////////////////////////////////////////
class SteamCATrajVar : public SteamTrajVar
{
 public:

  /// Shared pointer typedefs for readability
  typedef std::shared_ptr<SteamCATrajVar> Ptr;
  typedef std::shared_ptr<const SteamCATrajVar> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  SteamCATrajVar(const steam::Time& time, const se3::TransformEvaluator::Ptr& T_k0,
                 const VectorSpaceStateVar::Ptr& w_0k_ink,
                 const VectorSpaceStateVar::Ptr& acceleration);

  SteamCATrajVar(const steam::Time& time, const se3::TransformEvaluator::Ptr& T_k0,
                 const VectorSpaceStateVar::Ptr& w_0k_ink,
                 const VectorSpaceStateVar::Ptr& acceleration,
                 const Eigen::Matrix<double,18,18> cov);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get velocity state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  const VectorSpaceStateVar::Ptr& getAcceleration() const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Generalized 6D acceleration state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  VectorSpaceStateVar::Ptr acceleration_;
};

} // se3
} // steam

#endif // STEAM_CA_TRAJECTORY_VAR_HPP
