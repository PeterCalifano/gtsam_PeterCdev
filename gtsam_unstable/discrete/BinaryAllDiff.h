/*
 * BinaryAllDiff.h
 * @brief Binary "all-different" constraint
 * @date Feb 6, 2012
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam_unstable/discrete/Constraint.h>
#include <gtsam_unstable/discrete/Domain.h>

namespace gtsam {

/**
 * Binary AllDiff constraint
 * Returns 1 if values for two keys are different, 0 otherwise.
 */
class BinaryAllDiff : public Constraint {
  size_t cardinality0_, cardinality1_;  /// cardinality

 public:
  /// Constructor
  BinaryAllDiff(const DiscreteKey& key1, const DiscreteKey& key2)
      : Constraint(key1.first, key2.first),
        cardinality0_(key1.second),
        cardinality1_(key2.second) {}

  // print
  void print(
      const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    std::cout << s << "BinaryAllDiff on " << formatter(keys_[0]) << " and "
              << formatter(keys_[1]) << std::endl;
  }

  /// equals
  bool equals(const DiscreteFactor& other, double tol) const override {
    if (!dynamic_cast<const BinaryAllDiff*>(&other))
      return false;
    else {
      const BinaryAllDiff& f(static_cast<const BinaryAllDiff&>(other));
      return (cardinality0_ == f.cardinality0_) &&
             (cardinality1_ == f.cardinality1_);
    }
  }

  /// Calculate value
  double evaluate(const Assignment<Key>& values) const override {
    return (double)(values.at(keys_[0]) != values.at(keys_[1]));
  }

  /// Convert into a decisiontree
  DecisionTreeFactor toDecisionTreeFactor() const override {
    DiscreteKeys keys;
    keys.push_back(DiscreteKey(keys_[0], cardinality0_));
    keys.push_back(DiscreteKey(keys_[1], cardinality1_));
    std::vector<double> table;
    for (size_t i1 = 0; i1 < cardinality0_; i1++)
      for (size_t i2 = 0; i2 < cardinality1_; i2++) table.push_back(i1 != i2);
    DecisionTreeFactor converted(keys, table);
    return converted;
  }

  /// Multiply into a decisiontree
  DecisionTreeFactor operator*(const DecisionTreeFactor& f) const override {
    // TODO: can we do this more efficiently?
    return toDecisionTreeFactor() * f;
  }

  /*
   * Ensure Arc-consistency by checking every possible value of domain j.
   * @param j domain to be checked
   * @param (in/out) domains all domains, but only domains->at(j) will be checked.
   * @return true if domains->at(j) was changed, false otherwise.
   */
  bool ensureArcConsistency(Key j, Domains* domains) const override {
    throw std::runtime_error(
        "BinaryAllDiff::ensureArcConsistency not implemented");
    return false;
  }

  /// Partially apply known values
  Constraint::shared_ptr partiallyApply(const DiscreteValues&) const override {
    throw std::runtime_error("BinaryAllDiff::partiallyApply not implemented");
  }

  /// Partially apply known values, domain version
  Constraint::shared_ptr partiallyApply(
      const Domains&) const override {
    throw std::runtime_error("BinaryAllDiff::partiallyApply not implemented");
  }
};

}  // namespace gtsam
