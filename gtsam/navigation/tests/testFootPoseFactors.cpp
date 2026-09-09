/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testFootPoseFactors.cpp
 * @brief Relative-foot residual, Jacobian, and joint noise regressions.
 * @author Pietro Califano
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/FootPoseFactors.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <limits>

using namespace gtsam;

namespace {
const Pose3 imu(Rot3::RzRyRx(0.2, -0.3, 0.1), Point3(1.0, -0.2, 0.7));
const Pose3 foot(Rot3::RzRyRx(-0.1, 0.15, 0.3), Point3(0.5, 0.4, 0.0));
const Pose3 measured(Rot3::RzRyRx(0.12, 0.08, -0.2), Point3(0.2, -0.1, -0.6));
const Vector3 velocity(0.3, -0.2, 0.1), rate(0.1, 0.2, -0.1),
    omega(0.2, -0.1, 0.3);
const imuBias::ConstantBias bias(Vector3(0.01, 0.02, 0.03),
                                 Vector3(-0.01, 0.02, 0.01));
}  // namespace

TEST(FootPoseFactors, PoseJacobiansAtNonzeroResidual) {
  FootPoseFactor factor(0, 1, measured, noiseModel::Unit::Create(6));
  const auto error = [&](const Pose3& a, const Pose3& b) -> Vector {
    return factor.evaluateError(a, b);
  };
  Matrix H1, H2;
  const Vector residual = factor.evaluateError(imu, foot, H1, H2);
  EXPECT(assert_equal(Pose3::Logmap(measured.inverse() * imu.inverse() * foot),
                      residual, 1e-12));
  EXPECT(assert_equal(
      numericalDerivative21<Vector, Pose3, Pose3>(error, imu, foot), H1, 1e-6));
  EXPECT(assert_equal(
      numericalDerivative22<Vector, Pose3, Pose3>(error, imu, foot), H2, 1e-6));
  Matrix6 onlyFoot;
  EXPECT(assert_equal(
      residual, factor.evaluateErrorFixedSize(imu, foot, nullptr, &onlyFoot),
      1e-12));
  EXPECT(assert_equal(H2, onlyFoot, 1e-12));
  EXPECT(assert_equal(Vector(Vector6::Zero()),
                      factor.evaluateErrorFixedSize(imu, imu * measured),
                      1e-12));
  EXPECT(factor.equals(*factor.clone()));
  EXPECT(!factor.equals(
      FootPoseFactor(0, 1, Pose3(), noiseModel::Unit::Create(6))));
}

TEST(FootPoseFactors, PoseReferenceFrameConversion) {
  const Pose3 worldPBase = imu;
  const Pose3 basePImu(Rot3::RzRyRx(0.1, -0.05, 0.2), Point3(0.1, 0.0, 0.2));
  const Pose3 measuredBasePFoot = measured;
  const Pose3 worldPImu = worldPBase * basePImu;
  const Pose3 measuredImuPFoot = basePImu.inverse() * measuredBasePFoot;
  const auto noise = noiseModel::Unit::Create(6);
  const FootPoseFactor baseFactor(0, 1, measuredBasePFoot, noise);
  const FootPoseFactor imuFactor(0, 1, measuredImuPFoot, noise);

  // A fixed change of reference frame cancels inside Z^-1 T_reference^-1
  // T_foot. The right-local measurement tangent is unchanged by this left
  // composition.
  const Vector expected = baseFactor.evaluateError(worldPBase, foot);
  EXPECT(
      assert_equal(expected, imuFactor.evaluateError(worldPImu, foot), 1e-12));
}

TEST(FootPoseFactors, JointJacobiansAndOptionalOutputs) {
  FootPoseVelocityFactor factor(0, 1, 2, 3, measured, rate, omega,
                                noiseModel::Unit::Create(9));
  const auto error = [&](const Pose3& a, const Vector3& v,
                         const imuBias::ConstantBias& b,
                         const Pose3& f) -> Vector {
    return factor.evaluateError(a, v, b, f);
  };
  std::array<Matrix, 4> H;
  const Vector residual =
      factor.evaluateError(imu, velocity, bias, foot, H[0], H[1], H[2], H[3]);
  EXPECT(assert_equal(
      (numericalDerivative41<Vector, Pose3, Vector3, imuBias::ConstantBias,
                             Pose3>(error, imu, velocity, bias, foot)),
      H[0], 1e-6));
  EXPECT(assert_equal(
      (numericalDerivative42<Vector, Pose3, Vector3, imuBias::ConstantBias,
                             Pose3>(error, imu, velocity, bias, foot)),
      H[1], 1e-6));
  EXPECT(assert_equal(
      (numericalDerivative43<Vector, Pose3, Vector3, imuBias::ConstantBias,
                             Pose3>(error, imu, velocity, bias, foot)),
      H[2], 1e-6));
  EXPECT(assert_equal(
      (numericalDerivative44<Vector, Pose3, Vector3, imuBias::ConstantBias,
                             Pose3>(error, imu, velocity, bias, foot)),
      H[3], 1e-6));

  for (size_t i = 0; i < H.size(); ++i) {
    Matrix actual;
    std::array<Matrix*, 4> requested{};
    requested[i] = &actual;
    EXPECT(assert_equal(
        residual,
        factor.evaluateError(imu, velocity, bias, foot, requested[0],
                             requested[1], requested[2], requested[3]),
        1e-12));
    EXPECT(assert_equal(H[i], actual, 1e-12));
  }
  EXPECT(factor.equals(*factor.clone()));
  EXPECT(!factor.equals(FootPoseVelocityFactor(0, 1, 2, 3, measured, rate,
                                               Vector3::Zero(),
                                               noiseModel::Unit::Create(9))));

  // Cancel the measured relative motion to make the foot origin stationary.
  const Vector3 stationaryVelocity = -imu.rotation().rotate(
      (omega - bias.gyroscope()).cross(measured.translation()) + rate);
  EXPECT(assert_equal(
      Vector9::Zero(),
      factor.evaluateError(imu, stationaryVelocity, bias, imu * measured),
      1e-12));
}

TEST(FootPoseFactors, JointNoisePreservesCrossCovariance) {
  Matrix9 covariance = Matrix9::Identity();
  covariance(2, 7) = covariance(7, 2) = 0.4;
  FootPoseVelocityFactor factor(0, 1, 2, 3, measured, rate, omega,
                                noiseModel::Gaussian::Covariance(covariance));
  Values values;
  values.insert(0, imu);
  values.insert(1, velocity);
  values.insert(2, bias);
  values.insert(3, foot);
  const Vector residual = factor.evaluateError(imu, velocity, bias, foot);
  DOUBLES_EQUAL(0.5 * residual.dot(covariance.llt().solve(residual)),
                factor.error(values), 1e-12);
  EXPECT(std::abs(factor.error(values) - 0.5 * residual.squaredNorm()) > 1e-6);
}

TEST(FootPoseFactors, RejectsInvalidMeasurements) {
  CHECK_EXCEPTION(FootPoseFactor(0, 1, measured, nullptr),
                  std::invalid_argument);
  CHECK_EXCEPTION(FootPoseFactor(0, 1, measured, noiseModel::Unit::Create(9)),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      FootPoseVelocityFactor(0, 1, 2, 3, measured, rate, omega, nullptr),
      std::invalid_argument);
  CHECK_EXCEPTION(FootPoseVelocityFactor(0, 1, 2, 3, measured, rate, omega,
                                         noiseModel::Unit::Create(6)),
                  std::invalid_argument);
  Vector3 bad = rate;
  bad.x() = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(
      FootPoseFactor(0, 1, Pose3(Rot3(), bad), noiseModel::Unit::Create(6)),
      std::invalid_argument);
  CHECK_EXCEPTION(FootPoseVelocityFactor(0, 1, 2, 3, measured, bad, omega,
                                         noiseModel::Unit::Create(9)),
                  std::invalid_argument);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
