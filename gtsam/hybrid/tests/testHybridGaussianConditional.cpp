/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridGaussianConditional.cpp
 * @brief   Unit tests for HybridGaussianConditional class
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>

#include <memory>
#include <vector>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

// Common constants
static const Key modeKey = M(0);
static const DiscreteKey mode(modeKey, 2);
static const VectorValues vv{{Z(0), Vector1(4.9)}, {X(0), Vector1(5.0)}};
static const DiscreteValues assignment0{{M(0), 0}}, assignment1{{M(0), 1}};
static const HybridValues hv0{vv, assignment0};
static const HybridValues hv1{vv, assignment1};

/* ************************************************************************* */
namespace equal_constants {
// Create a simple HybridGaussianConditional
const double commonSigma = 2.0;
const std::vector<GaussianConditional::shared_ptr> conditionals{
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             commonSigma),
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             commonSigma)};
const HybridGaussianConditional hybrid_conditional(mode, conditionals);
}  // namespace equal_constants

/* ************************************************************************* */
/// Check that invariants hold
TEST(HybridGaussianConditional, Invariants) {
  using namespace equal_constants;

  // Check that the conditional (negative log) normalization constant is the min
  // of all constants which are all equal, in this case, hence:
  const double K = hybrid_conditional.negLogConstant();
  EXPECT_DOUBLES_EQUAL(K, conditionals[0]->negLogConstant(), 1e-8);
  EXPECT_DOUBLES_EQUAL(K, conditionals[1]->negLogConstant(), 1e-8);

  EXPECT(HybridGaussianConditional::CheckInvariants(hybrid_conditional, hv0));
  EXPECT(HybridGaussianConditional::CheckInvariants(hybrid_conditional, hv1));
}

/* ************************************************************************* */
/// Check LogProbability.
TEST(HybridGaussianConditional, LogProbability) {
  using namespace equal_constants;
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    EXPECT_DOUBLES_EQUAL(conditionals[mode]->logProbability(vv),
                         hybrid_conditional.logProbability(hv), 1e-8);
  }
}

/* ************************************************************************* */
/// Check error.
TEST(HybridGaussianConditional, Error) {
  using namespace equal_constants;
  auto actual = hybrid_conditional.errorTree(vv);

  // Check result.
  DiscreteKeys discrete_keys{mode};
  std::vector<double> leaves = {conditionals[0]->error(vv),
                                conditionals[1]->error(vv)};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);

  EXPECT(assert_equal(expected, actual, 1e-6));

  // Check for non-tree version.
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    EXPECT_DOUBLES_EQUAL(conditionals[mode]->error(vv),
                         hybrid_conditional.error(hv), 1e-8);
  }
}

/* ************************************************************************* */
/// Check that the likelihood is proportional to the conditional density given
/// the measurements.
TEST(HybridGaussianConditional, Likelihood) {
  using namespace equal_constants;

  // Compute likelihood
  auto likelihood = hybrid_conditional.likelihood(vv);

  // Check that the hybrid conditional error and the likelihood error are the
  // same.
  EXPECT_DOUBLES_EQUAL(hybrid_conditional.error(hv0), likelihood->error(hv0),
                       1e-8);
  EXPECT_DOUBLES_EQUAL(hybrid_conditional.error(hv1), likelihood->error(hv1),
                       1e-8);

  // Check that likelihood error is as expected, i.e., just the errors of the
  // individual likelihoods, in the `equal_constants` case.
  std::vector<DiscreteKey> discrete_keys = {mode};
  std::vector<double> leaves = {conditionals[0]->likelihood(vv)->error(vv),
                                conditionals[1]->likelihood(vv)->error(vv)};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);
  EXPECT(assert_equal(expected, likelihood->errorTree(vv), 1e-6));

  // Check that the ratio of probPrime to evaluate is the same for all modes.
  std::vector<double> ratio(2);
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    ratio[mode] =
        std::exp(-likelihood->error(hv)) / hybrid_conditional.evaluate(hv);
  }
  EXPECT_DOUBLES_EQUAL(ratio[0], ratio[1], 1e-8);
}

/* ************************************************************************* */
namespace mode_dependent_constants {
// Create a HybridGaussianConditional with mode-dependent noise models.
// 0 is low-noise, 1 is high-noise.
const std::vector<GaussianConditional::shared_ptr> conditionals{
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             0.5),
    GaussianConditional::sharedMeanAndStddev(Z(0), I_1x1, X(0), Vector1(0.0),
                                             3.0)};
const HybridGaussianConditional hybrid_conditional(mode, conditionals);
}  // namespace mode_dependent_constants

/* ************************************************************************* */
// Create a test for continuousParents.
TEST(HybridGaussianConditional, ContinuousParents) {
  using namespace mode_dependent_constants;
  const KeyVector continuousParentKeys = hybrid_conditional.continuousParents();
  // Check that the continuous parent keys are correct:
  EXPECT(continuousParentKeys.size() == 1);
  EXPECT(continuousParentKeys[0] == X(0));

  EXPECT(HybridGaussianConditional::CheckInvariants(hybrid_conditional, hv0));
  EXPECT(HybridGaussianConditional::CheckInvariants(hybrid_conditional, hv1));
}

/* ************************************************************************* */
/// Check error with mode dependent constants.
TEST(HybridGaussianConditional, Error2) {
  using namespace mode_dependent_constants;
  auto actual = hybrid_conditional.errorTree(vv);

  // Check result.
  DiscreteKeys discrete_keys{mode};
  double negLogConstant0 = conditionals[0]->negLogConstant();
  double negLogConstant1 = conditionals[1]->negLogConstant();
  double minErrorConstant = std::min(negLogConstant0, negLogConstant1);

  // Expected error is e(X) + log(sqrt(|2πΣ|)).
  // We normalize log(sqrt(|2πΣ|)) with min(negLogConstant)
  // so it is non-negative.
  std::vector<double> leaves = {
      conditionals[0]->error(vv) + negLogConstant0 - minErrorConstant,
      conditionals[1]->error(vv) + negLogConstant1 - minErrorConstant};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);

  EXPECT(assert_equal(expected, actual, 1e-6));

  // Check for non-tree version.
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    EXPECT_DOUBLES_EQUAL(conditionals[mode]->error(vv) +
                             conditionals[mode]->negLogConstant() -
                             minErrorConstant,
                         hybrid_conditional.error(hv), 1e-8);
  }
}

/* ************************************************************************* */
/// Check that the likelihood is proportional to the conditional density given
/// the measurements.
TEST(HybridGaussianConditional, Likelihood2) {
  using namespace mode_dependent_constants;

  // Compute likelihood
  auto likelihood = hybrid_conditional.likelihood(vv);

  // Check that the hybrid conditional error and the likelihood error are as
  // expected, this invariant is the same as the equal noise case:
  EXPECT_DOUBLES_EQUAL(hybrid_conditional.error(hv0), likelihood->error(hv0),
                       1e-8);
  EXPECT_DOUBLES_EQUAL(hybrid_conditional.error(hv1), likelihood->error(hv1),
                       1e-8);

  // Check the detailed JacobianFactor calculation for mode==1.
  {
    // We have a JacobianFactor
    const auto [gf1, _] = (*likelihood)(assignment1);
    const auto jf1 = std::dynamic_pointer_cast<JacobianFactor>(gf1);
    CHECK(jf1);

    // Check that the JacobianFactor error with constants is equal to the
    // conditional error:
    EXPECT_DOUBLES_EQUAL(hybrid_conditional.error(hv1),
                         jf1->error(hv1) + conditionals[1]->negLogConstant() -
                             hybrid_conditional.negLogConstant(),
                         1e-8);
  }

  // Check that the ratio of probPrime to evaluate is the same for all modes.
  std::vector<double> ratio(2);
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    ratio[mode] =
        std::exp(-likelihood->error(hv)) / hybrid_conditional.evaluate(hv);
  }
  EXPECT_DOUBLES_EQUAL(ratio[0], ratio[1], 1e-8);
}

/* ************************************************************************* */
namespace two_mode_measurement {
// Create a two key conditional:
const DiscreteKeys modes{{M(1), 2}, {M(2), 2}};
const std::vector<GaussianConditional::shared_ptr> gcs = {
    GaussianConditional::sharedMeanAndStddev(Z(0), Vector1(1), 1),
    GaussianConditional::sharedMeanAndStddev(Z(0), Vector1(2), 2),
    GaussianConditional::sharedMeanAndStddev(Z(0), Vector1(3), 3),
    GaussianConditional::sharedMeanAndStddev(Z(0), Vector1(4), 4)};
const HybridGaussianConditional::Conditionals conditionals(modes, gcs);
const auto hgc =
    std::make_shared<HybridGaussianConditional>(modes, conditionals);
}  // namespace two_mode_measurement

/* ************************************************************************* */
// Test pruning a HybridGaussianConditional with two discrete keys, based on a
// DecisionTreeFactor with 3 keys:
TEST(HybridGaussianConditional, Prune) {
  using two_mode_measurement::hgc;

  DiscreteKeys keys = two_mode_measurement::modes;
  keys.push_back({M(3), 2});
  {
    for (size_t i = 0; i < 8; i++) {
      std::vector<double> potentials{0, 0, 0, 0, 0, 0, 0, 0};
      potentials[i] = 1;
      const DecisionTreeFactor decisionTreeFactor(keys, potentials);
      // Prune the HybridGaussianConditional
      const auto pruned =
          hgc->prune(DiscreteConditional(keys.size(), decisionTreeFactor));
      // Check that the pruned HybridGaussianConditional has 1 conditional
      EXPECT_LONGS_EQUAL(1, pruned->nrComponents());
    }
  }
  {
    const std::vector<double> potentials{0, 0, 0.5, 0,  //
                                         0, 0, 0.5, 0};
    const DecisionTreeFactor decisionTreeFactor(keys, potentials);

    const auto pruned =
        hgc->prune(DiscreteConditional(keys.size(), decisionTreeFactor));

    // Check that the pruned HybridGaussianConditional has 2 conditionals
    EXPECT_LONGS_EQUAL(2, pruned->nrComponents());

    // Check that the minimum negLogConstant is set correctly
    EXPECT_DOUBLES_EQUAL(
        hgc->conditionals()({{M(1), 0}, {M(2), 1}})->negLogConstant(),
        pruned->negLogConstant(), 1e-9);
  }
  {
    const std::vector<double> potentials{0.2, 0, 0.3, 0,  //
                                         0,   0, 0.5, 0};
    const DecisionTreeFactor decisionTreeFactor(keys, potentials);

    const auto pruned =
        hgc->prune(DiscreteConditional(keys.size(), decisionTreeFactor));

    // Check that the pruned HybridGaussianConditional has 3 conditionals
    EXPECT_LONGS_EQUAL(3, pruned->nrComponents());

    // Check that the minimum negLogConstant is correct
    EXPECT_DOUBLES_EQUAL(hgc->negLogConstant(), pruned->negLogConstant(), 1e-9);
  }
}

/* *************************************************************************
 * This test verifies the behavior of the restrict method in different
 * scenarios:
 * - When no restrictions are applied.
 * - When one parent is restricted.
 * - When two parents are restricted.
 * - When the restriction results in a Gaussian conditional.
 */
TEST(HybridGaussianConditional, Restrict) {
  // Create a HybridConditional with two discrete parents P(z0|m0,m1)
  const auto hc =
      std::make_shared<HybridConditional>(two_mode_measurement::hgc);

  const auto same =
      std::dynamic_pointer_cast<HybridConditional>(hc->restrict({}));
  CHECK(same);
  EXPECT(same->isHybrid());
  EXPECT(same->asHybrid()->nrComponents() == 4);

  const auto oneParent =
      std::dynamic_pointer_cast<HybridConditional>(hc->restrict({{M(1), 0}}));
  CHECK(oneParent);
  EXPECT(oneParent->isHybrid());
  EXPECT(oneParent->asHybrid()->nrComponents() == 2);

  const auto oneParent2 = std::dynamic_pointer_cast<HybridConditional>(
      hc->restrict({{M(7), 0}, {M(1), 0}}));
  CHECK(oneParent2);
  EXPECT(oneParent2->isHybrid());
  EXPECT(oneParent2->asHybrid()->nrComponents() == 2);

  const auto gaussian = std::dynamic_pointer_cast<HybridConditional>(
      hc->restrict({{M(1), 0}, {M(2), 1}}));
  CHECK(gaussian);
  EXPECT(gaussian->asGaussian());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* *************************************************************************
 */
