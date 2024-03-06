// Tweddle's comment: code modified from Node.h
// This header defines the class NodeExmapT with inheritance from Node

// modified from Node.h - tweddle
#pragma once
#include <isam/Element.h>
#include <isam/Node.h>
#include <isam/Noise.h>
#include <Eigen/Dense>
#include <list>

namespace isam {
template <class T > 15 class NodeExmapT : public Node {
 protected:
  18 T* _value;  // current estimate 19 T* _value0; // linearization point 20 21 public: 22 23
  NodeExmapT() : Node(T::name(), T::dim) {
    _value = NULL;
    _value0 = NULL;
  }
  NodeExmapT(const char* name) : Node(name, T::dim) {
    _value = NULL;
    _value0 = NULL;
  }
  virtual : NodeExmapT() {
    delete _value;
    delete _value0;
  }
  void init(const T& t) {
    delete _value;
    delete _value0;
    _value = new T(t);
    _value0 = new T(t);
  }
  bool initialized() const { return _value != NULL; }

  T value(Selector s = ESTIMATE) const { return (s == ESTIMATE) ? *_value : *_value0; }
  T value0() const { return *_value0; }
  Eigen::VectorXd vector(Selector s = ESTIMATE) const {
    return (s == ESTIMATE) ? _value->vector() : _value0->vector();
  }
  Eigen::VectorXd vector0() const { return _value0->vector(); }
  void update(const Eigen::VectorXd& v) { _value->set(v); }
  void update0(const Eigen::VectorXd& v) { _value0->set(v); }
  void linpoint_to_estimate() { *_value = *_value0; }
  void estimate_to_linpoint() { *_value0 = *_value; }
  void swap_estimates() {
    T tmp = *_value;
    *_value = *_value0;
    *_value0 = tmp;
  }
  / void apply_exmap(const Eigen::VectorXd& v) { *_value = _value0->exmap(v); }
  // void
  self_exmap(const Eigen::VectorXd& v) { *_value0 = _value0->exmap(v); }
  void apply_exmap(const Eigen::VectorXd& v);
  void self_exmap(const Eigen::VectorXd& v) { *_value0 = _value0->exmap(v); }
  void rezero() {
    _value->rezero();
    _value0->rezero();
  }
  void write(std::ostream& out) const {
    out << name() << "_Node " << _id;
    if (_value != NULL) {
      out << " " << value();
    }
  }
};

}  // namespace isam
// snippet code goes "elsewhere" for compilation
template <class T>
void NodeExmapT<T>::apply_exmap(const Eigen::VectorXd& v) {
  *_value = _value0->exmap_reset(v);
  // update factor noise
  std::list<Factor*> factor_list = this->factors();
  for (std::list<Factor*>::iterator it = factor_list.begin(); it != factor_list.end(); it++) {
    Factor* factor = *it;
    dynamicPose3d_NL_dynamicPose3d_NL_Factor* dynamic_factor;
    dynamic_factor = dynamic_cast<dynamicPose3d_NL_dynamicPose3d_NL_Factor*>(factor);
    if (dynamic_factor != 0) {
      if (dynamic_factor->checkPose1(this)) {
        std::cout << "Found Dynamic Factor in apply_exmap(), adjusting noise " << std::endl;
        Eigen::MatrixXd sqrtinf = dynamic_factor->get_sqrtinf();
        Noise newnoise = isam::SqrtInformation(sqrtinf);
        dynamic_factor->setNoise(newnoise);
      }
    }
  }
}