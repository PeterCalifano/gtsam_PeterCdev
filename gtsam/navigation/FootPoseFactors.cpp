/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file FootPoseFactors.cpp
 * @brief Direct relative-foot pose residuals and their analytic Jacobians.
 * @author Pietro Califano
 */

#include <gtsam/navigation/FootPoseFactors.h>

#include <stdexcept>

namespace gtsam {
namespace {

void validateMeasurement(const Pose3& measurement,
                         const SharedNoiseModel& model, size_t dimension) {
  if (!model || model->dim() != dimension ||
      !measurement.matrix().allFinite()) {
    throw std::invalid_argument(
        "Foot pose factor requires finite FK pose and matching noise "
        "dimension.");
  }
}

Vector6 poseResidual(const Pose3& measured, const Pose3& worldPReference,
                     const Pose3& worldPFoot, Matrix6* H1, Matrix6* H2) {
  Matrix6 predictedHReference, predictedHFoot;
  const Pose3 predicted =
      worldPReference.between(worldPFoot, H1 ? &predictedHReference : nullptr,
                              H2 ? &predictedHFoot : nullptr);
  const bool derivatives = H1 || H2;
  Matrix6 errorHPredicted, logHError;
  const Pose3 error =
      measured.between(predicted, {}, derivatives ? &errorHPredicted : nullptr);
  const Vector6 residual =
      Pose3::Logmap(error, derivatives ? &logHError : nullptr);

  // Differentiate the full Logmap chain, including at nonzero residuals.
  if (derivatives) {
    const Matrix6 residualHPredicted = logHError * errorHPredicted;
    if (H1) *H1 = residualHPredicted * predictedHReference;
    if (H2) *H2 = residualHPredicted * predictedHFoot;
  }
  return residual;
}

}  // namespace

FootPoseFactor::FootPoseFactor(Key referenceKey, Key footKey,
                               const Pose3& measuredReferencePFoot,
                               const SharedNoiseModel& model)
    : Base(model, referenceKey, footKey),
      measuredReferencePFoot_(measuredReferencePFoot) {
  validateMeasurement(measuredReferencePFoot, model, 6);
}

NonlinearFactor::shared_ptr FootPoseFactor::clone() const {
  return std::make_shared<FootPoseFactor>(*this);
}

bool FootPoseFactor::equals(const NonlinearFactor& other, double tol) const {
  const auto* factor = dynamic_cast<const FootPoseFactor*>(&other);
  return factor && Base::equals(other, tol) &&
         measuredReferencePFoot_.equals(factor->measuredReferencePFoot_, tol);
}

Vector6 FootPoseFactor::evaluateErrorFixedSize(const Pose3& worldPReference,
                                               const Pose3& worldPFoot,
                                               Matrix6* H1, Matrix6* H2) const {
  return poseResidual(measuredReferencePFoot_, worldPReference, worldPFoot, H1,
                      H2);
}

Vector FootPoseFactor::evaluateError(const Pose3& worldPReference,
                                     const Pose3& worldPFoot,
                                     OptionalMatrixType H1,
                                     OptionalMatrixType H2) const {
  Matrix6 referenceH, footH;
  const Vector6 residual =
      evaluateErrorFixedSize(worldPReference, worldPFoot,
                             H1 ? &referenceH : nullptr, H2 ? &footH : nullptr);
  if (H1) *H1 = referenceH;
  if (H2) *H2 = footH;
  return residual;
}

FootPoseVelocityFactor::FootPoseVelocityFactor(
    Key imuKey, Key velocityKey, Key biasKey, Key footKey,
    const Pose3& measuredImuPFoot, const Vector3& measuredFootVelocity,
    const Vector3& measuredOmega, const SharedNoiseModel& model)
    : Base(model, imuKey, velocityKey, biasKey, footKey),
      measuredImuPFoot_(measuredImuPFoot),
      measuredFootVelocity_(measuredFootVelocity),
      measuredOmega_(measuredOmega) {
  validateMeasurement(measuredImuPFoot, model, 9);
  if (!measuredFootVelocity.allFinite() || !measuredOmega.allFinite()) {
    throw std::invalid_argument(
        "Foot pose factor requires finite velocity and gyro data.");
  }
}

NonlinearFactor::shared_ptr FootPoseVelocityFactor::clone() const {
  return std::make_shared<FootPoseVelocityFactor>(*this);
}

bool FootPoseVelocityFactor::equals(const NonlinearFactor& other,
                                    double tol) const {
  const auto* factor = dynamic_cast<const FootPoseVelocityFactor*>(&other);
  return factor && Base::equals(other, tol) &&
         measuredImuPFoot_.equals(factor->measuredImuPFoot_, tol) &&
         traits<Vector3>::Equals(measuredFootVelocity_,
                                 factor->measuredFootVelocity_, tol) &&
         traits<Vector3>::Equals(measuredOmega_, factor->measuredOmega_, tol);
}

Vector FootPoseVelocityFactor::evaluateError(
    const Pose3& worldPImu, const Vector3& worldVelocity,
    const imuBias::ConstantBias& bias, const Pose3& worldPFoot,
    OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3,
    OptionalMatrixType H4) const {
  Matrix6 poseHImu, poseHFoot;
  Vector9 residual;
  residual.head<6>() =
      poseResidual(measuredImuPFoot_, worldPImu, worldPFoot,
                   H1 ? &poseHImu : nullptr, H4 ? &poseHFoot : nullptr);

  // The no-slip lever arm is the measured foot origin, not a graph landmark.
  Matrix3 velocityHRotation, velocityHWorld;
  const Vector3 imuVelocity = worldPImu.rotation().unrotate(
      worldVelocity, H1 ? &velocityHRotation : nullptr,
      H2 ? &velocityHWorld : nullptr);
  const Point3& q = measuredImuPFoot_.translation();
  residual.tail<3>() = imuVelocity +
                       (measuredOmega_ - bias.gyroscope()).cross(q) +
                       measuredFootVelocity_;

  if (H1) {
    H1->setZero(9, 6);
    H1->topRows<6>() = poseHImu;
    H1->block<3, 3>(6, 0) = velocityHRotation;
  }
  if (H2) {
    H2->setZero(9, 3);
    H2->bottomRows<3>() = velocityHWorld;
  }
  if (H3) {
    H3->setZero(9, 6);
    H3->block<3, 3>(6, 3) = skewSymmetric(q);
  }
  if (H4) {
    H4->setZero(9, 6);
    H4->topRows<6>() = poseHFoot;
  }
  return residual;
}

}  // namespace gtsam
