#pragma once 2 3 #include < vector> 4 #include < string> 5 6 #include < \
    math.h>  // for sqrt 7 #include <Eigen/Dense> 8 9 #include<isam/util.h> 10
             // #include<isam/Jacobian.h> 11 #include<isam/Element.h> 12 #include<isam/Node.h> 13
             // #include<isam/Noise.h> 14 #include<isam/numericalDiff.h> 15 16 namespace isam { 17
             // 18 19 // Generic template for easy instantiation of new factors 20 template <class
             // T> 21 class FactorVarNoiseT : public Factor { 22 23 /* Not a const variable 24 *
             // This is important because it allows the factor's uncertainty to be updated in
             // real-time 25 */ 26 Noise _noise_variable; 27 cost_func_t *ptr_cost_func_local; 28
             // protected: 29 30 const T _measure; 31 32 public: 33 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
             // 34 35 FactorVarNoiseT(const char* name, int dim, const Noise& noise, const T&
             // measure) : Factor(name, dim, noise), _measure(measure) { 36 _noise_variable = noise;
             // 37 ptr_cost_func_local = NULL; 38 } 39 40 virtual void setNoise(Noise& newNoise) {
             // 41 _noise_variable = newNoise; 42 }

virtual void set_cost_function(cost_func_t* ptr) { ptr_cost_func_local = ptr; }
45 46 virtual Eigen::VectorXd error(Selector s = ESTIMATE) const {
  47 Eigen::VectorXd err = _noise_variable.sqrtinf() * basic_error(s);
  48  // optional modified cost function 49 if (*ptr_cost_func_local) { 50 for (int i=0;
      // i<err.size(); i++) { 51 double val = err(i); 52 err(i) = ((val0)?1.:(-1.)) *
      // sqrt((*ptr_cost_func_local)(val)); 53 } 54 } 55 return err; 56 } 57 58 virtual const
      // Eigen::MatrixXd& sqrtinf() const {return _noise_variable.sqrtinf();} 59 60 const T&
      // measurement() const { 61 return _measure; 62 } 63 64 void write(std::ostream &out) const {
      // 65 Factor::write(out); 66 out << " " << _measure << " " <<
      // noise_to_string(_noise_variable); 67 } 68 69 }; 70 71 72 }