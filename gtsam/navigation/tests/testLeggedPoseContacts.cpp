/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testLeggedPoseContacts.cpp
 * @brief Direct foot-pose contact startup, covariance, and episode regressions.
 * @author Pietro Califano
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/navigation/LeggedEstimator.h>

#include <algorithm>
#include <limits>

using namespace gtsam;

namespace {

LeggedEstimatorParams makeParams() {
  LeggedEstimatorParams params;
  params.preintegrationParams = PreintegrationParams::MakeSharedU(9.81);
  params.preintegrationParams->setGyroscopeCovariance(I_3x3 * 1e-4);
  params.preintegrationParams->setAccelerometerCovariance(I_3x3 * 1e-4);
  params.preintegrationParams->setIntegrationCovariance(I_3x3 * 1e-4);
  // Pose packets are already IMU-relative. This nonidentity extrinsic must
  // not be applied to them again.
  params.body_P_imu =
      Pose3(Rot3::RzRyRx(0.2, -0.1, 0.3), Point3(0.3, 0.1, 0.2));
  return params;
}

NavState priorState() {
  return NavState(Rot3::RzRyRx(0.2, -0.1, 0.3), Point3(0.1, -0.2, 1.2),
                  Vector3(0.2, 0.1, -0.1));
}

Matrix9 priorCovariance() {
  Matrix9 basis = I_9x9;
  basis(0, 3) = 0.3;
  basis(1, 7) = -0.2;
  basis(5, 8) = 0.4;
  return 0.01 * basis * basis.transpose();
}

std::vector<FootPoseContactMeasurement> poseContacts(
    bool withVelocity = false) {
  std::vector<FootPoseContactMeasurement> contacts(2);
  for (size_t foot = 0; foot < contacts.size(); ++foot) {
    auto& contact = contacts[foot];
    contact.foot = foot;
    contact.imuPFoot = Pose3(Rot3::RzRyRx(-0.1, 0.2, 0.1),
                             Point3(0.1, foot == 0 ? 0.2 : -0.2, -0.5));
    contact.poseCovariance = I_6x6 * 0.002;
    if (withVelocity) {
      contact.velocity = FootPoseVelocityMeasurement{};
      contact.velocity->angularVelocityImu = Vector3(0.1, -0.2, 0.3);
      contact.velocity->footVelocityImu = Vector3(-0.1, 0.05, 0.02);
      Matrix9 basis = I_9x9;
      basis(0, 6) = 0.3;
      basis(4, 8) = -0.2;
      contact.velocity->covariance = 0.002 * basis * basis.transpose();
    }
  }
  return contacts;
}

}  // namespace

/* ************************************************************************* */
TEST(LeggedPoseContacts, BipedStartupRetainsJointPriorAndUsesFkOnce) {
  for (const auto engine :
       {LeggedFixedLagEngine::Batch, LeggedFixedLagEngine::Incremental}) {
    const auto params = makeParams();
    const NavState state = priorState();
    const Matrix9 covariance = priorCovariance();
    LeggedFixedLagSmoother ordinary(state, Matrix::Zero(3, 2), covariance,
                                    params, 0.2, {}, engine);
    LeggedCombinedFixedLagSmoother combined(
        state, Matrix::Zero(3, 2), covariance, params, 0.2, {}, engine);

    auto checkStartup = [&](auto& estimator) {
      auto contacts = poseContacts();
      estimator.processPoseContacts({contacts[0]});
      CHECK_EXCEPTION(estimator.navigationStateCovariance(), std::logic_error);
      estimator.predict(Vector3::Ones(), Vector3::Zero(), 0.1);
      EXPECT(assert_equal(state.pose(), Pose3(estimator.estimate().rotation(),
                                              estimator.estimate().x(0))));

      // Unsorted packets must pair each pose with its own physical foot.
      std::reverse(contacts.begin(), contacts.end());
      estimator.processPoseContacts(contacts);
      const Matrix posterior = estimator.navigationStateCovariance();
      EXPECT(assert_equal(Matrix(covariance),
                          Matrix(posterior.topLeftCorner<9, 9>()), 1e-9));
      EXPECT(assert_equal(
          state.pose(),
          Pose3(estimator.estimate().rotation(), estimator.estimate().x(0)),
          1e-9));
      EXPECT(assert_equal(state.velocity(), estimator.estimate().x(1), 1e-9));
      for (const auto& contact : contacts) {
        const Point3 expected = (state.pose() * contact.imuPFoot).translation();
        EXPECT(assert_equal(expected, estimator.estimate().x(2 + contact.foot),
                            1e-9));
      }
    };
    checkStartup(ordinary);
    checkStartup(combined);
  }
}

/* ************************************************************************* */
TEST(LeggedPoseContacts, NavStateAndSplitStateUseTheSameJointMeasurement) {
  for (const auto engine :
       {LeggedFixedLagEngine::Batch, LeggedFixedLagEngine::Incremental}) {
    for (const bool withVelocity : {false, true}) {
      const auto params = makeParams();
      LeggedFixedLagSmoother ordinary(priorState(), Matrix::Zero(3, 2),
                                      priorCovariance(), params, 0.2, {},
                                      engine);
      LeggedCombinedFixedLagSmoother combined(priorState(), Matrix::Zero(3, 2),
                                              priorCovariance(), params, 0.2,
                                              {}, engine);
      auto contacts = poseContacts(withVelocity);
      ordinary.processPoseContacts(contacts);
      combined.processPoseContacts(contacts);

      // Repeated observations constrain the existing anchor. Nonzero pose
      // errors and pose/velocity noise cross terms exercise the adapter chain.
      contacts[0].imuPFoot = contacts[0].imuPFoot.retract(
          (Vector6() << 0.02, -0.01, 0.03, 0.01, 0.02, -0.01).finished());
      ordinary.processPoseContacts(contacts);
      combined.processPoseContacts(contacts);
      // A single incremental update uses different NavState/Pose3 retractions
      // and cached linearizations. Only the converged batch comparison is
      // tight.
      const double tolerance =
          engine == LeggedFixedLagEngine::Batch ? 1e-7 : 5e-6;
      EXPECT(assert_equal(ordinary.estimate(), combined.estimate(), tolerance));
      EXPECT(assert_equal(ordinary.navigationStateCovariance(),
                          combined.navigationStateCovariance(), tolerance));
      if (withVelocity) {
        CHECK((ordinary.estimate().x(1) - priorState().velocity()).norm() >
              0.01);
      }
    }
  }
}

/* ************************************************************************* */
TEST(LeggedPoseContacts, TerrainConstraintIsExplicitAtStartup) {
  for (const auto engine :
       {LeggedFixedLagEngine::Batch, LeggedFixedLagEngine::Incremental}) {
    const auto params = makeParams();
    LeggedFixedLagSmoother ordinary(priorState(), Matrix::Zero(3, 2),
                                    priorCovariance(), params, 0.2, {}, engine);
    LeggedCombinedFixedLagSmoother combined(priorState(), Matrix::Zero(3, 2),
                                            priorCovariance(), params, 0.2, {},
                                            engine);
    auto checkTerrain = [&](auto& estimator) {
      estimator.turnHeightPriorOn(0.0);
      estimator.processPoseContacts(poseContacts());
      CHECK(estimator.estimate().x(0).z() < priorState().position().z() - 0.01);
      CHECK(estimator.navigationStateCovariance()(5, 5) <
            priorCovariance()(5, 5));
    };
    checkTerrain(ordinary);
    checkTerrain(combined);
  }
}

/* ************************************************************************* */
TEST(LeggedPoseContacts, InvalidPacketsDoNotChangeTheContactEpisode) {
  auto params = makeParams();
  params.useFullContactInitialization = false;
  LeggedFixedLagSmoother ordinary(priorState(), Matrix::Zero(3, 2),
                                  priorCovariance(), params, 0.2);
  LeggedCombinedFixedLagSmoother combined(priorState(), Matrix::Zero(3, 2),
                                          priorCovariance(), params, 0.2);
  auto checkValidation = [&](auto& estimator) {
    auto contacts = poseContacts();
    estimator.processPoseContacts(contacts);
    const auto state = estimator.estimate();
    const Matrix covariance = estimator.navigationStateCovariance();
    CHECK_EXCEPTION(estimator.processPoseContacts({contacts[0], contacts[0]}),
                    std::invalid_argument);
    auto invalid = contacts[0];
    invalid.foot = 2;
    CHECK_EXCEPTION(estimator.processPoseContacts({invalid}),
                    std::invalid_argument);
    invalid = contacts[0];
    invalid.imuPFoot =
        Pose3(Rot3(), Point3(std::numeric_limits<double>::quiet_NaN(), 0, 0));
    CHECK_EXCEPTION(estimator.processPoseContacts({invalid}),
                    std::invalid_argument);
    for (const bool withVelocity : {false, true}) {
      for (int defect = 0; defect < 3; ++defect) {
        invalid = poseContacts(withVelocity)[0];
        Matrix covariance = withVelocity ? Matrix(invalid.velocity->covariance)
                                         : Matrix(invalid.poseCovariance);
        if (defect == 0) covariance(0, 0) = -1.0;
        if (defect == 1) covariance(0, 1) += 0.5;
        if (defect == 2)
          covariance(0, 0) = std::numeric_limits<double>::infinity();
        if (withVelocity)
          invalid.velocity->covariance = covariance;
        else
          invalid.poseCovariance = covariance;
        CHECK_EXCEPTION(estimator.processPoseContacts({invalid}),
                        std::invalid_argument);
      }
    }
    invalid = poseContacts(true)[0];
    invalid.velocity->angularVelocityImu.x() =
        std::numeric_limits<double>::infinity();
    CHECK_EXCEPTION(estimator.processPoseContacts({invalid}),
                    std::invalid_argument);
    CHECK_EXCEPTION(estimator.processCorrelatedContacts({}),
                    std::invalid_argument);
    CHECK_EXCEPTION(
        estimator.processContacts({ContactMeasurement(0, Vector3::Zero())}),
        std::invalid_argument);
    EXPECT(assert_equal(state, estimator.estimate(), 1e-12));
    EXPECT(
        assert_equal(covariance, estimator.navigationStateCovariance(), 1e-12));

    // Changing representation is safe only with a new key. Ending an episode
    // and an explicit touchdown are both valid ways to obtain one.
    estimator.processContacts({ContactMeasurement(0, Vector3::Zero(), true)});
    CHECK_EXCEPTION(estimator.processPoseContacts({contacts[0]}),
                    std::invalid_argument);
    contacts[0].touchdown = true;
    estimator.processPoseContacts({contacts[0]});
    estimator.processPoseContacts({});
    estimator.processContacts({ContactMeasurement(0, Vector3::Zero())});
    estimator.processContacts({});
    contacts[0].touchdown = false;
    estimator.processPoseContacts({contacts[0]});
    CHECK(estimator.navigationStateCovariance().allFinite());
  };
  checkValidation(ordinary);
  checkValidation(combined);
}

/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
