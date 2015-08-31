//////////////////////////////////////////////////////////////////////////////////////////////
/// \file StereoCameraErrorEvalXX.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_STEREO_CAMERA_ERROR_EVALUATOR_DYNAMIC_HPP
#define STEAM_STEREO_CAMERA_ERROR_EVALUATOR_DYNAMIC_HPP

#include <steam/evaluator/ErrorEvaluator.hpp>
#include <steam/state/LandmarkStateVar.hpp>
#include <steam/evaluator/TransformEvalOperations.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Stereo camera error function evaluator
//////////////////////////////////////////////////////////////////////////////////////////////
class StereoCameraErrorEvalX : public ErrorEvaluatorX
{
public:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Simple structure to hold the basic camera intrinsics
  //////////////////////////////////////////////////////////////////////////////////////////////
  struct CameraIntrinsics {

    /// Convenience typedefs
    typedef boost::shared_ptr<CameraIntrinsics> Ptr;
    typedef boost::shared_ptr<const CameraIntrinsics> ConstPtr;

    /// \brief Stereo baseline
    double b;

    /// \brief Focal length in the u-coordinate (horizontal)
    double fu;

    /// \brief Focal length in the v-coordinate (vertical)
    double fv;

    /// \brief Focal center offset in the u-coordinate (horizontal)
    double cu;

    /// \brief Focal center offset in the v-coordinate (vertical)
    double cv;
  };

  /// Convenience typedefs
  typedef boost::shared_ptr<StereoCameraErrorEvalX> Ptr;
  typedef boost::shared_ptr<const StereoCameraErrorEvalX> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  StereoCameraErrorEvalX(const Eigen::Vector4d& meas,
                        const CameraIntrinsics::ConstPtr& intrinsics,
                        const se3::TransformEvaluator::ConstPtr& T_cam_landmark,
                        const se3::LandmarkStateVar::Ptr& landmark);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 4-d measurement error (ul vl ur vr)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::VectorXd evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 4-d measurement error (ul vl ur vr) and Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::VectorXd evaluate(const Eigen::MatrixXd& lhs, std::vector<Jacobian<> >* jacs) const;

private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Camera model
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector4d cameraModel(const Eigen::Vector4d& point) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Camera model Jacobian
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix4d cameraModelJacobian(const Eigen::Vector4d& point) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Measurement coordinates extracted from images (ul vl ur vr)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector4d meas_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Camera instrinsics
  //////////////////////////////////////////////////////////////////////////////////////////////
  CameraIntrinsics::ConstPtr intrinsics_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Point evaluator (evaluates the point transformed into the camera frame)
  //////////////////////////////////////////////////////////////////////////////////////////////
  se3::ComposeLandmarkEvaluator::ConstPtr eval_;

};

} // steam

#endif // STEAM_STEREO_CAMERA_ERROR_EVALUATOR_DYNAMIC_HPP
