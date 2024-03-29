// STL includes
#include <vector>

// Eigen includes
#include <Eigen/Dense>

// Aliases
using Eigen::VectorXd;

class nonlinMotionModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructors
  nonlinMotionModel() = default;
  nonlinMotionModel(double hStepSize);

  // Destructor
  ~nonlinMotionModel();

  // Integrator functions
  // TODO: convert RK45, RK78, RK4 from previous codes (and DACE)

  // Dynamics model
  virtual VectorXd nonlinDyn(VectorXd xState) = 0;  // Dynamic model as pure virtual (Derived class
                                                    // requires implementation)

  // Getters

  // Setters
  void nonlinMotionModel::setStepSize(double hStepSize);

 private:
  // Data members
  double hInitStepSize_ = 1.0;  // Default step size of integration
};
