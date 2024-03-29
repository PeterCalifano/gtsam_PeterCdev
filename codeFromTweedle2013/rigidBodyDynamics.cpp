#include "rigidBodyDynamics.h"

// Rigid Body Dynamics constructor
rigidBodyDynamics::rigidBodyDynamics(isam::inertiaRatios ir, double sigma_v, double sigma_w) {
  _ir = ir;
  _qref << 0, 0, 0, 1;
  _r = Vector3d::Zero();
  _v = Vector3d::Zero();
  _a = Vector3d::Zero();
  _w = Vector3d::Zero();
  setMassProperties(ir);
  setCovProperties(sigma_v, sigma_w);
}
// Setter of Mass properties
void rigidBodyDynamics::setMassProperties(isam::inertiaRatios ir) { _ir = ir; }

// Setter of Covariance variables
void rigidBodyDynamics::setCovProperties(double sigma_v, double sigma_w) {
  _sigma_v = sigma_v;
  _sigma_w = sigma_w;
  _Q = Matrix<double, 6, 6>::Zero();
  _Q.block<3, 3>(0, 0) = _sigma_v * _sigma_v * Matrix<double, 3, 3>::Identity();
  _Q.block<3, 3>(3, 3) = _sigma_w * _sigma_w * Matrix<double, 3, 3>::Identity();
}

// Update of reference quaternion from error state with reset of error state (MRP)?
void rigidBodyDynamics::reset_qref() {
  Vector3d a_ = _a;
  Vector4d qref_ = _qref;
  _qref = addQuaternionError(a_, qref_);
  _a = Vector3d::Zero();
}

// Update of global quaternion state without reset of error state (MRP)
Vector4d rigidBodyDynamics::qTotal() const {
  Vector3d a_ = _a;
  Vector4d qref_ = _qref;
  return addQuaternionError(a_, qref_);
};

// Dynamic model for the rigid body dynamics + kinematics
VectorXd rigidBodyDynamics::f(VectorXd x) {
  Vector3d dr, dv, da, dw;
  Matrix<double, 12, 12> lambda, dLambda;
  VectorXd vec_dLambda;
  VectorXd dx(90);
  Vector3d r = x.segment<3>(0);
  Vector3d v = x.segment<3>(3);
  Vector3d a = x.segment<3>(6);
  Vector3d w = x.segment<3>(9);
  MatrixXd Bw = getBw();  // Map of process noise to state space
  Matrix3d J = _ir.getJ();

  // Nonlinear State Model \dot x = f(x)
  /* 60 * \mathbf{\dot r} = \mathbf{v}  */
  dr = v;

  /* 65 * \mathbf{\dot v} = 0 66 */
  dv = Vector3d::Zero();
  /* 70 *
  // \frac{d \mathbf{a}_p}{dt} = 71 * \frac{1}{2}\left(\mathbf{[\omega \times]} + 72 *
  // \mathbf{\omega} \cdot \mathbf{\bar q} \right) \mathbf{a}_p + 73 * \frac{2 q_4}{1+q_4}
  // \mathbf{\omega} 74 */
  double c1, c2, c3;
  c1 = 0.5;
  c2 = 0.125 * w.dot(a);
  c3 = 1 - a.dot(a) / 16;
  da = -c1 * w.cross(a) + c2 * a + c3 * w;
  /* 83 * \dot
  // \mathbf{w} = -\mathbf{J}ˆ{-1} \mathbf{\omega} \times \mathbf{J} \mathbf{\ omega} 84 */
  dw = -J.inverse() * w.cross(J * w);

  // Covariance Propagation according to Lyapunov function
  // see Brown & Hwang pg 204 90 91
  // Compute Linear transition matrix
  Matrix<double, 12, 12> A = Matrix<double, 12, 12>::Zero();
  // position derivative
  A.block<3, 3>(0, 3) = Matrix<double, 3, 3>::Identity();

  // mrp kinematics
  A.block<3, 3>(6, 6) = -0.5 * crossProductMat(w) + w.dot(a) / 8 * Matrix3d::Identity();
  A.block<3, 3>(6, 9) = (1 - a.dot(a / 16)) * Matrix3d::Identity();

  // angular velocity dynamics 102 A.block<3,3>(9,9) = - J.inverse() * crossProductMat(w) * J;
  lambda = vec2symmMat(x.segment<78>(12));
  dLambda = A * lambda + lambda * A.transpose() + Bw * _Q * Bw.transpose();
  vec_dLambda = symmMat2Vec(dLambda);

  // write to dx 108 dx.segment<3>(0) = dr; 109 dx.segment<3>(3) = dv; 110 dx.segment<3>(6) =
  da;
  dx.segment<3>(9) = dw;
  dx.segment<78>(12) = vec_dLambda;
  return dx;
}

// Function to get Skew symmetric matrix
Matrix3d rigidBodyDynamics::crossProductMat(Vector3d vec) {
  Matrix3d M = Matrix3d::Zero();
  M(0, 1) = -vec(2);
  M(0, 2) = vec(1);
  M(1, 0) = vec(2);
  M(1, 2) = -vec(0);
  M(2, 0) = -vec(1);
  M(2, 1) = vec(0);
  return M;
}
// Function to convert a symmetric matrix to a vector
VectorXd rigidBodyDynamics::symmMat2Vec(Matrix<double, 12, 12> M) {
  VectorXd v(78);
  int count = 0;
  for (int row = 0; row < 12; row++) {
    for (int col = row; col < 12; col++) {
      v(count) = M(row, col);
      count++;
    }
  }
  return v;
}

// Function to convert a vector to a symmetric matrix
Matrix<double, 12, 12> rigidBodyDynamics::vec2symmMat(VectorXd v) {
  Matrix<double, 12, 12> M = Matrix<double, 12, 12>::Zero();
  int count = 0;
  for (int row = 0; row < 12; row++) {
    for (int col = row; col < 12; col++) {
      M(row, col) = v(count);
      M(col, row) = v(count);
      count++;
    }
  }
  return M;
}
// State vector x getter
VectorXd rigidBodyDynamics::x() const {
  VectorXd x(12);
  x.segment<3>(0) = _r;
  x.segment<3>(3) = _v;
  x.segment<3>(6) = _a;
  x.segment<3>(9) = _w;
  return x;
}
// State vector (x,q) setter (Translation + rotation)
void rigidBodyDynamics::setState(VectorXd x, Vector4d q) {
  _r = x.segment<3>(0);
  _v = x.segment<3>(3);
  _a = x.segment<3>(6);
  _w = x.segment<3>(9);
  _qref = q / q.norm();
}
// State vector x setter (Translation)
void rigidBodyDynamics::setState(VectorXd x) {
  _r = x.segment<3>(0);
  _v = x.segment<3>(3);
  _a = x.segment<3>(6);
  _w = x.segment<3>(9);
}
// Conversion function from MRP to quaternion (error state)
Vector4d rigidBodyDynamics::mrp2quaternion(Vector3d mrp) const {
  Vector4d dq;
  dq << 8 * mrp / (16 + mrp.transpose() * mrp),
      (16 - mrp.transpose() * mrp) / (16 + mrp.transpose() * mrp);
  dq /= dq.norm();
  return dq;
}
// Conversion function from quaternion to MRP
Vector3d rigidBodyDynamics::quaternion2mrp(Vector4d q) const {
  Vector3d mrp;
  if (q(3) < 0) {
    q = -q;
  }
  mrp << 4 * q(0) / (1 + q(3)), 4 * q(1) / (1 + q(3)), 4 * q(2) / (1 + q(3));
  return mrp;
}
// Function to merge quaternion error state (from MRP representation) to global quaternion
Vector4d rigidBodyDynamics::addQuaternionError(Vector3d& mrp, Vector4d& qref) const {
  Vector4d qnew, dq;
  dq = mrp2quaternion(mrp);
  Vector4d qnew1 = quaternionMultiplication(dq, qref);
  // Return updated quaternion
  if (qnew1.dot(qref) >= 0) {
    return qnew1;

  } else {
    Vector4d qnew2 = -1 * qnew1;
    return qnew2;
  }
}
// Function for quaternion multiplications
Vector4d rigidBodyDynamics::quaternionMultiplication(Vector4d& q1, Vector4d& q2) const {
  // q1 \mult q2
  Matrix4d qm;
  Vector4d result;
  qm << q1(3), q1(2), -q1(1), q1(0), -q1(2), q1(3), q1(0), q1(1), q1(1), -q1(0), q1(3), q1(2),
      -q1(0), -q1(1), -q1(2), q1(3);
  result = qm * q2;
  result /= result.norm();
  return result;
}
// Function for quaternion operation q1 x q2^-1
Vector4d rigidBodyDynamics::quaternionDivision(Vector4d& q1, Vector4d& q2) const {
  Vector4d q2inv;
  q2inv << -q2(0), -q2(1), -q2(2), q2(3);
  Vector4d result = quaternionMultiplication(q1, q2inv);
  return result;
}

// Function to compute 1st order finite difference approximation of angular velocity
Vector3d rigidBodyDynamics::diffQuaternion(Vector4d& q, Vector4d& qprev, double dt) const {
  Vector4d dq = (q - qprev) / dt;
  Matrix4d M;
  M << qprev(3), qprev(2), -qprev(1), -qprev(0), -qprev(2), qprev(3), qprev(0), -qprev(1), qprev(1),
      -qprev(0), qprev(3), -qprev(2), qprev(0), qprev(1), qprev(2), qprev(3);
  Vector4d wp = 2 * M * dq;
  Vector3d w = wp.head(3);
  return w;
}
// Function to compute DCM from quaternion
Matrix3d rigidBodyDynamics::rotationMatrix(Vector4d& q) const {
  Matrix3d rot;
  rot(0, 0) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  rot(0, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
  rot(0, 2) = 2 * (q(0) * q(2) - q(1) * q(3));
  rot(1, 0) = 2 * (q(0) * q(1) - q(2) * q(3));
  rot(1, 1) = -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  rot(1, 2) = 2 * (q(2) * q(1) + q(0) * q(3));
  rot(2, 0) = 2 * (q(0) * q(2) + q(1) * q(3));

  rot(2, 1) = 2 * (q(2) * q(1) - q(0) * q(3));
  rot(2, 2) = -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
  return rot;
}
// Function to compute quaternion from DCM
Vector4d rigidBodyDynamics::quaternionFromRot(Matrix3d& R) const {
  Vector4d q;
  double div1, div2, div3, div4;
  double numerical_limit = 1.0e-4;
  if (abs(R.determinant() - 1) > numerical_limit) {
    std::cerr << "R does not have a determinant of +1" << std::endl;

  } else {
    div1 = 0.5 * sqrt(1 + R(0, 0) + R(1, 1) + R(2, 2));
    div2 = 0.5 * sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2));
    div3 = 0.5 * sqrt(1 - R(0, 0) - R(1, 1) + R(2, 2));
    div4 = 0.5 * sqrt(1 - R(0, 0) + R(1, 1) - R(2, 2));
    // if (div1 > div2 && div1 > div3 && div1 > div4) {
    if (fabs(div1) > numerical_limit) {
      q(3) = div1;
      q(0) = 0.25 * (R(1, 2) - R(2, 1)) / q(3);
      q(1) = 0.25 * (R(2, 0) - R(0, 2)) / q(3);
      q(2) = 0.25 * (R(0, 1) - R(1, 0)) / q(3);
    } else if (fabs(div2) > numerical_limit) {
      // } else if (div2 > div1 && div2 > div3 && div2 > div4)
      q(0) = div2;
      q(1) = 0.25 * (R(0, 1) + R(1, 0)) / q(0);
      q(2) = 0.25 * (R(0, 2) + R(2, 0)) / q(0);
      q(3) = 0.25 * (R(1, 2) + R(2, 1)) / q(0);

    } else if (fabs(div3) > numerical_limit) {  //} else if (div3 > div1 && div3 > div2 && div3
                                                // > div4) {
      q(2) = div3;
      q(0) = 0.25 * (R(0, 2) + R(2, 0)) / q(2);
      q(1) = 0.25 * (R(1, 2) + R(2, 1)) / q(2);
      q(3) = 0.25 * (R(0, 1) - R(1, 0)) / q(2);
    } else if (fabs(div4) > numerical_limit) {
      q(1) = div4;
      q(0) = 0.25 * (R(0, 1) + R(1, 0)) / q(1);
      q(2) = 0.25 * (R(1, 2) + R(2, 1)) / q(1);
      q(3) = 0.25 * (R(2, 0) - R(0, 2)) / q(1);
    } else {
      std::cerr << "quaternionFromRot didn't convert: [" << div1 << ", " << div2 << ", " << div3
                << ", " << div4 << std::endl;
      std::cerr << "Rotation Matrix: " << R << std::endl;
    }
  }
  q /= q.norm();
  return q;
}

// Function to get map of process noise to state space
MatrixXd rigidBodyDynamics::getBw() const {
  Matrix<double, 12, 6> Bw;
  Bw = Matrix<double, 12, 6>::Zero();
  Bw.block<3, 3>(3, 0) = Matrix3d::Identity();
  Bw.block<3, 3>(9, 3) = Matrix3d::Identity();
  return Bw;
}

// Setters of Inerta matrix, ration and uncertainty covariances
Matrix3d rigidBodyDynamics::getJ() const { return _ir.getJ(); }
isam::inertiaRatios rigidBodyDynamics::getIR() const { return _ir; }
void rigidBodyDynamics::setIR(isam::inertiaRatios ir) { _ir = ir; }
double rigidBodyDynamics::getSigmaV() const { return _sigma_v; }
double rigidBodyDynamics::getSigmaW() const { return _sigma_w; }