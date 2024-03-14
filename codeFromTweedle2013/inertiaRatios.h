#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <isam/Factor.h>
#include <isam/Node.h>
#include <Eigen/Geometry>
#include "NodeExmap.h"

namespace isam {
class inertiaRatios {
  std::ostream& operator<<(std::ostream& out, const inertiaRatios& p) {
    p.write(out);
    return out;
  }
  /* k1 = ln(J11 / J22)
    k2 = ln(J22 / J33)  */
  double _k1;
  double _k2;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW static const int dim = 2;
  static const char* name() { return "inertiaRatios"; }
  inertiaRatios() {
    _k1 = 0;
    _k2 = 0;
  }
  inertiaRatios(const double& k1, const double& k2) {
    _k1 = k1;
    _k2 = k2;
  }
  Eigen::Matrix3d getJ() const {
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    double Jscale = 1.0;
    // 0.0116; --> ?
    J(0, 0) = exp(_k1);
    J(1, 1) = 1.0;
    J(2, 2) = exp(-_k2);
    J *= Jscale;
    return J;
  }
  Eigen::VectorXd x() const {
    Eigen::Vector2d x;
    x(0) = _k1;
    x(1) = _k2;
    return x;
  }
  void setState(Eigen::VectorXd x) {
    _k1 = x(0);
    _k2 = x(1);
  }
  inertiaRatios exmap(const Eigen::Vector2d& ∆) {
    inertiaRatios res = *this;
    res._k1 += ∆(0);
    res._k2 += ∆(1);
    return res;
  }
  inertiaRatios exmap_reset(const Eigen::Vector2d& ∆) {
    inertiaRatios res = *this;
    res._k1 += ∆(0);
    res._k2 += ∆(1);
    return res;
  }
  Eigen::VectorXd vector() const {
    Eigen::Vector2d tmp;
    tmp << _k1, _k2;
    return tmp;
  }
  void set(const Eigen::Vector2d& v) {
    _k1 = v(0);
    _k2 = v(1);
  }
  void write(std::ostream& out) const {
    Eigen::Matrix3d Jcurr = getJ();
    out << std::endl
        << "inertaRatios x: " << x().transpose() << std::endl
        << Jcurr(0, 0) << " , " << Jcurr(1, 1) << " , " << Jcurr(2, 2) << std::endl;
  }
};
typedef NodeExmapT<inertiaRatios> inertiaRatios_Node;
/* * Prior on inertiaRatios. */
class inertiaRatios_Factor : public FactorT<inertiaRatios> {
 public:
  inertiaRatios_Node* _ir_node;
  inertiaRatios_Factor(inertiaRatios_Node* ir_node, const inertiaRatios& prior, const Noise& noise)
      : FactorT<inertiaRatios>("inertiaRatios_Factor", 2, noise, prior), _ir_node(ir_node) {
    _nodes.resize(1);
    _nodes[0] = ir_node;
  }
  void initialize() {
    if (!_ir_node->initialized()) {
      inertiaRatios predict = _measure;
      _ir_node->init(predict);
    }
  }
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    inertiaRatios ir = _ir_node->value(s);
    Eigen::VectorXd err = ir.vector() - _measure.vector();
    return err;
  }
};
}  // namespace isam