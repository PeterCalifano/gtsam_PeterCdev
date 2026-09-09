/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file FootPoseFactors.h
 * @brief Relative-foot pose and joint pose/no-slip factors.
 * @author Pietro Califano
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam {

/**
 * Relative-foot pose observation over reference-frame and contact-episode world
 * poses. The residual is Log(Z_RF^-1 T_WR^-1 T_WF), ordered rotation,
 * translation. Noise is specified in that right-local measurement tangent, not
 * world axes. The factor contains no sole geometry or corner measurements. The
 * caller expresses FK in the frame represented by referenceKey; this may be the
 * robot base or the IMU, without introducing an additional graph key.
 */
class GTSAM_EXPORT FootPoseFactor : public NoiseModelFactorN<Pose3, Pose3> {
  using Base = NoiseModelFactorN<Pose3, Pose3>;

 public:
  using Base::evaluateError;

  /** Construct with a copied FK pose and shared six-dimensional noise model.
   * @param referenceKey Reference-frame pose in world coordinates.
   * @param footKey Contact-episode foot pose in world coordinates.
   * @param measuredReferencePFoot FK foot pose in reference-frame coordinates.
   * @param model Noise in rotation/translation residual order.
   * @throws std::invalid_argument For null/wrong-sized noise or nonfinite data.
   */
  FootPoseFactor(Key referenceKey, Key footKey,
                 const Pose3& measuredReferencePFoot,
                 const SharedNoiseModel& model);

  /// Copy the factor, retaining the shared noise model.
  NonlinearFactor::shared_ptr clone() const override;
  /// Compare keys, noise, and measurement.
  bool equals(const NonlinearFactor& other, double tol = 1e-9) const override;

  /** Return the unwhitened pose residual and optional right-local Jacobians.
   * H1 and H2 differentiate the reference and foot world poses respectively.
   * This fixed-size entry point also serves the inertial-contact bridge.
   */
  Vector6 evaluateErrorFixedSize(const Pose3& worldPReference,
                                 const Pose3& worldPFoot, Matrix6* H1 = nullptr,
                                 Matrix6* H2 = nullptr) const;
  /// GTSAM dynamic-matrix adapter for evaluateErrorFixedSize().
  Vector evaluateError(const Pose3& worldPReference, const Pose3& worldPFoot,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override;

 private:
  Pose3 measuredReferencePFoot_;
};

/**
 * Joint relative-foot pose and foot-origin no-slip observation.
 * Rows are [pose rotation, pose translation, IMU-frame no-slip velocity].
 * The final residual is R_WI^T v_WI + (omega_I - b_g) x q_I + q_dot_I,
 * where q_I is the FK pose translation. Full 9-D noise retains pose/velocity
 * cross covariance. Angular velocity is measured; only IMU bias is estimated.
 */
class GTSAM_EXPORT FootPoseVelocityFactor
    : public NoiseModelFactorN<Pose3, Vector3, imuBias::ConstantBias, Pose3> {
  using Base = NoiseModelFactorN<Pose3, Vector3, imuBias::ConstantBias, Pose3>;

 public:
  using Base::evaluateError;

  /** Construct from copied FK and gyro data, sharing the noise model.
   * @param imuKey IMU world-pose key.
   * @param velocityKey IMU world-velocity key.
   * @param biasKey IMU bias key, ordered accelerometer then gyroscope.
   * @param footKey Contact-episode foot world-pose key.
   * @param measuredImuPFoot FK foot pose in IMU coordinates.
   * @param measuredFootVelocity Derivative of relative foot-origin position in
   * IMU axes.
   * @param measuredOmega Raw gyroscope measurement in IMU axes.
   * @param model Joint nine-dimensional residual noise.
   * @throws std::invalid_argument For null/wrong-sized noise or nonfinite data.
   */
  FootPoseVelocityFactor(Key imuKey, Key velocityKey, Key biasKey, Key footKey,
                         const Pose3& measuredImuPFoot,
                         const Vector3& measuredFootVelocity,
                         const Vector3& measuredOmega,
                         const SharedNoiseModel& model);

  /// Copy the factor, retaining the shared noise model.
  NonlinearFactor::shared_ptr clone() const override;
  /// Compare keys, noise, and all measurements.
  bool equals(const NonlinearFactor& other, double tol = 1e-9) const override;
  /// Return the unwhitened residual; optional Jacobians follow state argument
  /// order.
  Vector evaluateError(const Pose3& worldPImu, const Vector3& worldVelocity,
                       const imuBias::ConstantBias& bias,
                       const Pose3& worldPFoot, OptionalMatrixType H1,
                       OptionalMatrixType H2, OptionalMatrixType H3,
                       OptionalMatrixType H4) const override;

 private:
  Pose3 measuredImuPFoot_;
  Vector3 measuredFootVelocity_;
  Vector3 measuredOmega_;
};

}  // namespace gtsam
