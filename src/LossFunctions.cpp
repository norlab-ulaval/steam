//////////////////////////////////////////////////////////////////////////////////////////////
/// \file LossFunctions.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/LossFunctions.hpp>

namespace steam {


//////////////////////////////////////////////////////////////////////////////////////////////
/// L2 Loss Function
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Cost function (basic evaluation of the loss function)
//////////////////////////////////////////////////////////////////////////////////////////////
double L2LossFunc::cost(double whitened_error_norm) const {
  return 0.5*whitened_error_norm*whitened_error_norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
//////////////////////////////////////////////////////////////////////////////////////////////
double L2LossFunc::weight(double whitened_error_norm) const {
  return 1.0;
}


//////////////////////////////////////////////////////////////////////////////////////////////
/// Huber Loss Function
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Cost function (basic evaluation of the loss function)
//////////////////////////////////////////////////////////////////////////////////////////////
double HuberLossFunc::cost(double whitened_error_norm) const {
  double e2 = whitened_error_norm*whitened_error_norm;
  double abse = fabs(whitened_error_norm); // should already be positive anyway ...
  if (abse <= k_) {
    return 0.5*e2;
  } else {
    return 0.5*k_*abse;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
//////////////////////////////////////////////////////////////////////////////////////////////
double HuberLossFunc::weight(double whitened_error_norm) const {
  double abse = fabs(whitened_error_norm); // should already be positive anyway ...
  if (abse <= k_) {
    return 1.0;
  } else {
    return k_/abse;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// DCS Loss Function
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Cost function (basic evaluation of the loss function)
//////////////////////////////////////////////////////////////////////////////////////////////
double DcsLossFunc::cost(double whitened_error_norm) const {
  double e2 = whitened_error_norm*whitened_error_norm;
  if (e2 <= k_) {
    return 0.5*e2;
  } else {
    return 2.0*k_*e2/(k_+e2) - 0.5*k_;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Weight for iteratively reweighted least-squares (influence function div. by error)
//////////////////////////////////////////////////////////////////////////////////////////////
double DcsLossFunc::weight(double whitened_error_norm) const {
  double e2 = whitened_error_norm*whitened_error_norm;
  if (e2 <= k_) {
    return 1.0;
  } else {
    double kpe2 = k_+e2;
    return 4.0*k_*k_/(kpe2*kpe2);
  }
}

} // steam
