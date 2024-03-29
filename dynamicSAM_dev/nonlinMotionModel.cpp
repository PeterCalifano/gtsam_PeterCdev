// STL includes
#include <vector>

// Eigen includes
#include <Eigen/Dense>

// Custom includes
#include "nonlinMotionModel.h"

// Setter of Step size
void nonlinMotionModel::setStepSize(double hStepSize) { hInitStepSize_ = hStepSize; }
