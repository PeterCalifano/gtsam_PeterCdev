/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridBayesNet.cpp
 * @brief  A Bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Shangjie Xue
 * @author Frank Dellaert
 * @date   January 2022
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/discrete/TableDistribution.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>

#include <memory>

// In Wrappers we have no access to this so have a default ready
static std::mt19937_64 kRandomNumberGenerator(42);

namespace gtsam {

/* ************************************************************************* */
void HybridBayesNet::print(const std::string &s,
                           const KeyFormatter &formatter) const {
  Base::print(s, formatter);
}

/* ************************************************************************* */
bool HybridBayesNet::equals(const This &bn, double tol) const {
  return Base::equals(bn, tol);
}

/* ************************************************************************* */
// The implementation is: build the entire joint into one factor and then prune.
// TODO(Frank): This can be quite expensive *unless* the factors have already
// been pruned before. Another, possibly faster approach is branch and bound
// search to find the K-best leaves and then create a single pruned conditional.
HybridBayesNet HybridBayesNet::prune(
    size_t maxNrLeaves, const std::optional<double> &deadModeThreshold) const {
#if GTSAM_HYBRID_TIMING
  gttic_(HybridPruning);
#endif
  // Collect all the discrete conditionals. Could be small if already pruned.
  const DiscreteBayesNet marginal = discreteMarginal();

  // Multiply into one big conditional. NOTE: possibly quite expensive.
  DiscreteConditional joint;
  for (auto &&conditional : marginal) {
    joint = joint * (*conditional);
  }

  // Initialize the resulting HybridBayesNet.
  HybridBayesNet result;

  // Prune the joint. NOTE: imperative and, again, possibly quite expensive.
  DiscreteConditional pruned = joint;
  pruned.prune(maxNrLeaves);

  DiscreteValues deadModesValues;
  // If we have a dead mode threshold and discrete variables left after pruning,
  // then we run dead mode removal.
  if (deadModeThreshold.has_value() && pruned.keys().size() > 0) {
#if GTSAM_HYBRID_TIMING
    gttic_(DeadModeRemoval);
#endif

    DiscreteMarginals marginals(DiscreteFactorGraph{pruned});
    for (auto dkey : pruned.discreteKeys()) {
      Vector probabilities = marginals.marginalProbabilities(dkey);

      int index = -1;
      auto threshold = (probabilities.array() > *deadModeThreshold);
      // If atleast 1 value is non-zero, then we can find the index
      // Else if all are zero, index would be set to 0 which is incorrect
      if (!threshold.isZero()) {
        threshold.maxCoeff(&index);
      }

      if (index >= 0) {
        deadModesValues.emplace(dkey.first, index);
      }
    }

    // Remove the modes (imperative)
    pruned.removeDiscreteModes(deadModesValues);

    GTSAM_PRINT(deadModesValues);

#if GTSAM_HYBRID_TIMING
    gttoc_(DeadModeRemoval);
#endif
  }

  /* To prune, we visitWith every leaf in the HybridGaussianConditional.
   * For each leaf, using the assignment we can check the discrete decision tree
   * for 0.0 probability, then just set the leaf to a nullptr.
   *
   * We can later check the HybridGaussianConditional for just nullptrs.
   */

  // Go through all the Gaussian conditionals in the Bayes Net and prune them as
  // per pruned discrete joint.
  for (auto &&conditional : *this) {
    if (auto hgc = conditional->asHybrid()) {
      // Prune the hybrid Gaussian conditional!
      auto prunedHybridGaussianConditional = hgc->prune(pruned);
      if (!prunedHybridGaussianConditional) {
        GTSAM_PRINT(marginal);
        GTSAM_PRINT(pruned);
        throw std::runtime_error(
            "A HybridGaussianConditional had all its conditionals pruned");
      }

      if (deadModeThreshold.has_value()) {
        const auto &discreteParents =
            prunedHybridGaussianConditional->discreteKeys();
        DiscreteValues deadParentValues;
        DiscreteKeys liveParents;
        for (const auto &key : discreteParents) {
          auto it = deadModesValues.find(key.first);
          if (it != deadModesValues.end())
            deadParentValues[key.first] = it->second;
          else
            liveParents.emplace_back(key);
        }
        // If so then we just get the corresponding Gaussian conditional:
        if (deadParentValues.size() == discreteParents.size()) {
          // print on how many discreteParents we are choosing:
          result.push_back(
              prunedHybridGaussianConditional->choose(deadParentValues));
        } else if (liveParents.size() > 0) {
          auto newTree = prunedHybridGaussianConditional->factors();
          for (auto &&[key, value] : deadModesValues) {
            newTree = newTree.choose(key, value);
          }
          result.emplace_shared<HybridGaussianConditional>(liveParents,
                                                           newTree);
        } else {
          // Add as-is
          result.push_back(prunedHybridGaussianConditional);
        }
      } else {
        // Type-erase and add to the pruned Bayes Net fragment.
        result.push_back(prunedHybridGaussianConditional);
      }

    } else if (auto gc = conditional->asGaussian()) {
      // Add the non-HybridGaussianConditional conditional
      result.push_back(gc);
    }
    // We ignore DiscreteConditional as they are already pruned and added.
  }

#if GTSAM_HYBRID_TIMING
  gttoc_(HybridPruning);
#endif

  if (deadModeThreshold.has_value()) {
    /*
      If the pruned discrete conditional has any keys left,
      we add it to the HybridBayesNet.
      If not, it means it is an orphan so we don't add this pruned joint,
      and instead add only the marginals below.
    */
    if (pruned.keys().size() > 0) {
      result.emplace_shared<DiscreteConditional>(pruned);
    }

    // Add the marginals for future factors
    // for (auto &&[key, _] : deadModesValues) {
    //   result.push_back(
    //       std::dynamic_pointer_cast<DiscreteConditional>(marginals(key)));
    // }

  } else {
    result.emplace_shared<DiscreteConditional>(pruned);
  }

  return result;
}

/* ************************************************************************* */
DiscreteBayesNet HybridBayesNet::discreteMarginal() const {
  DiscreteBayesNet result;
  for (auto &&conditional : *this) {
    if (auto dc = conditional->asDiscrete()) {
      result.push_back(dc);
    }
  }
  return result;
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (auto &&conditional : *this) {
    if (auto gm = conditional->asHybrid()) {
      // If conditional is hybrid, select based on assignment.
      gbn.push_back(gm->choose(assignment));
    } else if (auto gc = conditional->asGaussian()) {
      // If continuous only, add Gaussian conditional.
      gbn.push_back(gc);
    } else if (auto dc = conditional->asDiscrete()) {
      // If conditional is discrete-only, we simply continue.
      continue;
    }
  }

  return gbn;
}

/* ************************************************************************* */
DiscreteValues HybridBayesNet::mpe() const {
  // Collect all the discrete factors to compute MPE
  DiscreteFactorGraph discrete_fg;

  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      if (auto dtc = conditional->asDiscrete<TableDistribution>()) {
        // The number of keys should be small so should not
        // be expensive to convert to DiscreteConditional.
        discrete_fg.push_back(DiscreteConditional(dtc->nrFrontals(),
                                                  dtc->toDecisionTreeFactor()));
      } else {
        discrete_fg.push_back(conditional->asDiscrete());
      }
    }
  }

  return discrete_fg.optimize();
}

/* ************************************************************************* */
HybridValues HybridBayesNet::optimize() const {
  // Solve for the MPE
  DiscreteValues mpe = this->mpe();

  // Given the MPE, compute the optimal continuous values.
  return HybridValues(optimize(mpe), mpe);
}

/* ************************************************************************* */
VectorValues HybridBayesNet::optimize(const DiscreteValues &assignment) const {
  GaussianBayesNet gbn = choose(assignment);

  // Check if there exists a nullptr in the GaussianBayesNet
  // If yes, return an empty VectorValues
  if (std::find(gbn.begin(), gbn.end(), nullptr) != gbn.end()) {
    return VectorValues();
  }
  return gbn.optimize();
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(const HybridValues &given,
                                    std::mt19937_64 *rng) const {
  DiscreteBayesNet dbn;
  for (auto &&conditional : *this) {
    if (conditional->isDiscrete()) {
      // If conditional is discrete-only, we add to the discrete Bayes net.
      dbn.push_back(conditional->asDiscrete());
    }
  }
  // Sample a discrete assignment.
  const DiscreteValues assignment = dbn.sample(given.discrete());
  // Select the continuous Bayes net corresponding to the assignment.
  GaussianBayesNet gbn = choose(assignment);
  // Sample from the Gaussian Bayes net.
  VectorValues sample = gbn.sample(given.continuous(), rng);
  return {sample, assignment};
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(std::mt19937_64 *rng) const {
  HybridValues given;
  return sample(given, rng);
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample(const HybridValues &given) const {
  return sample(given, &kRandomNumberGenerator);
}

/* ************************************************************************* */
HybridValues HybridBayesNet::sample() const {
  return sample(&kRandomNumberGenerator);
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesNet::errorTree(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> result(0.0);

  // Iterate over each conditional.
  for (auto &&conditional : *this) {
    result = result + conditional->errorTree(continuousValues);
  }

  return result;
}

/* ************************************************************************* */
double HybridBayesNet::negLogConstant(
    const std::optional<DiscreteValues> &discrete) const {
  double negLogNormConst = 0.0;
  // Iterate over each conditional.
  for (auto &&conditional : *this) {
    if (discrete.has_value()) {
      if (auto gm = conditional->asHybrid()) {
        negLogNormConst += gm->choose(*discrete)->negLogConstant();
      } else if (auto gc = conditional->asGaussian()) {
        negLogNormConst += gc->negLogConstant();
      } else if (auto dc = conditional->asDiscrete()) {
        negLogNormConst += dc->choose(*discrete)->negLogConstant();
      } else {
        throw std::runtime_error(
            "Unknown conditional type when computing negLogConstant");
      }
    } else {
      negLogNormConst += conditional->negLogConstant();
    }
  }
  return negLogNormConst;
}

/* ************************************************************************* */
AlgebraicDecisionTree<Key> HybridBayesNet::discretePosterior(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> errors = this->errorTree(continuousValues);
  AlgebraicDecisionTree<Key> p =
      errors.apply([](double error) { return exp(-error); });
  return p / p.sum();
}

/* ************************************************************************* */
double HybridBayesNet::evaluate(const HybridValues &values) const {
  return exp(logProbability(values));
}

/* ************************************************************************* */
HybridGaussianFactorGraph HybridBayesNet::toFactorGraph(
    const VectorValues &measurements) const {
  HybridGaussianFactorGraph fg;

  // For all nodes in the Bayes net, if its frontal variable is in measurements,
  // replace it by a likelihood factor:
  for (auto &&conditional : *this) {
    if (conditional->frontalsIn(measurements)) {
      if (auto gc = conditional->asGaussian()) {
        fg.push_back(gc->likelihood(measurements));
      } else if (auto gm = conditional->asHybrid()) {
        fg.push_back(gm->likelihood(measurements));
      } else {
        throw std::runtime_error("Unknown conditional type");
      }
    } else {
      fg.push_back(conditional);
    }
  }
  return fg;
}

}  // namespace gtsam
