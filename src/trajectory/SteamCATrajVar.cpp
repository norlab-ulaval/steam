//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SteamCATrajVar.cpp
///
/// \author Tim Tang, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/trajectory/SteamCATrajVar.hpp>

#include <lgmath.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
SteamCATrajVar::SteamCATrajVar(const steam::Time& time,
             const se3::TransformEvaluator::Ptr& T_k0,
             const VectorSpaceStateVar::Ptr& w_0k_ink,
             const VectorSpaceStateVar::Ptr& acceleration)
  : SteamTrajVar(time, T_k0, w_0k_ink), acceleration_(acceleration) {

  // Check velocity input
  if (w_0k_ink->getPerturbDim() != 6) {
    throw std::invalid_argument("invalid velocity size");
  }

  // Check acceleration input
  if (acceleration->getPerturbDim() != 6) {
    throw std::invalid_argument("invalid acceleration size");
  }
}

SteamCATrajVar::SteamCATrajVar(const steam::Time& time,
             const se3::TransformEvaluator::Ptr& T_k0,
             const VectorSpaceStateVar::Ptr& w_0k_ink,
             const VectorSpaceStateVar::Ptr& acceleration,
             const Eigen::Matrix<double,18,18> cov)
  : SteamCATrajVar(time, T_k0, w_0k_ink, acceleration) { cov_set_=true; cov_=cov; }

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get acceleration state variable
//////////////////////////////////////////////////////////////////////////////////////////////
const VectorSpaceStateVar::Ptr& SteamCATrajVar::getAcceleration() const {
  return acceleration_;
}

} // se3
} // steam
