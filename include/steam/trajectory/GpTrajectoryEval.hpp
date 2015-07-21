//////////////////////////////////////////////////////////////////////////////////////////////
/// \file GpTrajectoryEval.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_GP_TRAJECTORY_EVAL_HPP
#define STEAM_GP_TRAJECTORY_EVAL_HPP

#include <Eigen/Core>

#include <steam/trajectory/GpTrajectory.hpp>

namespace steam {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Simple transform evaluator for a transformation state variable
//////////////////////////////////////////////////////////////////////////////////////////////
class GpTrajectoryEval : public TransformEvaluator
{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  GpTrajectoryEval(const Time& time, const GpTrajectory::Knot::ConstPtr& knot1,
                   const GpTrajectory::Knot::ConstPtr& knot2);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Pseudo constructor - return a shared pointer to a new instance
  //////////////////////////////////////////////////////////////////////////////////////////////
  static Ptr MakeShared(const Time& time, const GpTrajectory::Knot::ConstPtr& knot1,
                        const GpTrajectory::Knot::ConstPtr& knot2);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the transformation matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the transformation matrix and Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual lgmath::se3::Transformation evaluate(std::vector<Jacobian>* jacs) const;

 private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief First (earlier) knot
  //////////////////////////////////////////////////////////////////////////////////////////////
  GpTrajectory::Knot::ConstPtr knot1_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Second (later) knot
  //////////////////////////////////////////////////////////////////////////////////////////////
  GpTrajectory::Knot::ConstPtr knot2_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Interpolation coefficients
  //////////////////////////////////////////////////////////////////////////////////////////////
  double psi11_;
  double psi12_;
  double psi21_;
  double psi22_;
  double lambda11_;
  double lambda12_;
  double lambda21_;
  double lambda22_;

};

} // se3
} // steam

#endif // STEAM_GP_TRAJECTORY_EVAL_HPP