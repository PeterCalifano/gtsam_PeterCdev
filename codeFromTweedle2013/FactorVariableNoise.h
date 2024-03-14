#pragma once
#include <isam/Element.h>  
#include <isam/Node.h>
#include <isam/Jacobian.h>
#include <isam/Noise.h>
#include <isam/numericalDiff.h>

#include <isam/util.h>
#include <math.h>  // for sqrt
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace isam {

// Generic template for easy instantiation of new factors 20 template <class
T > class FactorVarNoiseT : public Factor {
  /* Not a const variable  *
  This is important because it allows the factor's uncertainty to be updated in
  real-time  */
  Noise _noise_variable;
  cost_func_t* ptr_cost_func_local;

 protected:
  const T _measure;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW FactorVarNoiseT(const char* name,
                                                  int dim,
                                                  const Noise& noise,
                                                  const T& measure)
      : Factor(name, dim, noise), _measure(measure) {
    _noise_variable = noise;
    ptr_cost_func_local = NULL;
  }
  virtual void setNoise(Noise& newNoise) { _noise_variable = newNoise; }

  virtual void set_cost_function(cost_func_t* ptr) { ptr_cost_func_local = ptr; }
  virtual Eigen::VectorXd error(Selector s = ESTIMATE) const {
    Eigen::VectorXd err = _noise_variable.sqrtinf() * basic_error(s);
    optional modified cost function if (*ptr_cost_func_local) {
      for (int i = 0; i < err.size(); i++) {
        double val = err(i);
        err(i) = ((val0) ? 1. : (-1.)) * sqrt((*ptr_cost_func_local)(val));
      }
    }
    return err;
  }
  virtual const Eigen::MatrixXd& sqrtinf() const { return _noise_variable.sqrtinf(); }
  const T& measurement() const { return _measure; }
  void write(std::ostream& out) const {
    Factor::write(out);
    out << " " << _measure << " " << noise_to_string(_noise_variable);
  }
};

}  // namespace isam