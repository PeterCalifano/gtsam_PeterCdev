/**
 *  @file   PseudorangeFactor.cpp
 *  @author Sammy Guo
 *  @brief  Implementation file for GNSS Pseudorange factor
 *  @date   January 18, 2026
 **/

#include "PseudorangeFactor.h"

namespace {

/// Speed of light in a vacuum (m/s):
constexpr double CLIGHT = 299792458.0;

}  // namespace

namespace gtsam {

//***************************************************************************
PseudorangeFactor::PseudorangeFactor(const Key receiverPositionKey,
                                     const Key receiverClockBiasKey,
                                     const double measuredPseudorange,
                                     const Point3& satellitePosition,
                                     const double satelliteClockBias,
                                     const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias} {}

//***************************************************************************
void PseudorangeFactor::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool PseudorangeFactor::equals(const NonlinearFactor& expected,
                               double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(pseudorange_, e->pseudorange_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector PseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias) const {
  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = receiverPosition - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - pseudorange_;

  // Compute associated derivatives:
  if (HreceiverPos) {
    if (range < std::numeric_limits<double>::epsilon()) {
      *HreceiverPos = Matrix13::Zero();
    } else {
      *HreceiverPos = (position_difference / range).transpose();
    }
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * CLIGHT;
  }

  return Vector1(error);
}

//***************************************************************************
DifferentialPseudorangeFactor::DifferentialPseudorangeFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey,
           differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias} {}

//***************************************************************************
void DifferentialPseudorangeFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool DifferentialPseudorangeFactor::equals(const NonlinearFactor& expected,
                                           double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(pseudorange_, e->pseudorange_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector DifferentialPseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClock_bias,
    const double& differentialCorrection, OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HdifferentialCorrection) const {
  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = receiverPosition - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiverClock_bias - satClkBias_);
  const double error = rho - pseudorange_ - differentialCorrection;

  // Compute associated derivatives:
  if (HreceiverPos) {
    if (range < std::numeric_limits<double>::epsilon()) {
      *HreceiverPos = Matrix13::Zero();
    } else {
      *HreceiverPos = (position_difference / range).transpose();
    }
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * CLIGHT;
  }

  if (HdifferentialCorrection) {
    *HdifferentialCorrection = -I_1x1;
  }

  return Vector1(error);
}
}  // namespace gtsam
