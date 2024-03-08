#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include "NodeExmap.h"

namespace isam {
typedef Eigen::Matrix<double, 6, 1> Vector6d;
class kinematicPose3d {
  frend std::ostream& operator<<(std::ostream& out, const kinematicPose3d& p) {
    p.write(out);
    return out;
  }
  Eigen::Vector4d _qref;
  Eigen::Vector3d _r;
  Eigen::Vector3d _a;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int dim = 6;
  static const char* name() { return "kinematicPose3d"; }
  kinematicPose3d() {
    _qref << 0.0, 0.0, 0.0, 1.0;
    _a << 0.0, 0.0, 0.0;
    _r << 0.0, 0.0, 0.0;
  }
  kinematicPose3d(const Eigen::MatrixXd& hm) {
    // Convert matrix to R,T 39 Eigen::Matrix4d HM = hm / hm(3,3); // enforce T(3,3)=1 40
    // Eigen::Matrix3d R = HM.topLeftCorner(3,3); 41 Eigen::Vector3d _r = HM.col(3).head(3); 42
    // 43 //compute quaternion 44 _qref = quaternionFromRot(R);
    _a = Eigen::Vector3d::Zero();
  }
  Eigen::VectorXd x() const {
    Vector6d x;
    Eigen::Vector3d r = _r;
    Eigen::Vector3d a = _a;
    x.segment<3>(0) = r;
    x.segment<3>(3) = a;
    return x;
  }
  void setState(Eigen::VectorXd x, Eigen::Vector4d q) {
    _r = x.segment<3>(0);
    _a = x.segment<3>(3);
    _qref = q / q.norm();
  }
  void setState(Eigen::VectorXd x) {
    _r = x.segment<3>(0);
    _a = x.segment<3>(3);
  }
  Eigen::Vector4d mrp2quaternion(Eigen::Vector3d mrp) const {
    Eigen::Vector4d dq;
    dq << 8 * mrp / (16 + mrp.transpose() * mrp),
        (16 - mrp.transpose() * mrp) / (16 + mrp.transpose() * mrp);
    dq /= dq.norm();
    return dq;
  }
  Eigen::Vector3d quaternion2mrp(Eigen::Vector4d q) const {
    Eigen::Vector3d mrp;
    if (q(3) < 0) {
      q = -q;
    }
    mrp << 4 * q(0) / (1 + q(3)), 4 * q(1) / (1 + q(3)), 4 * q(2) / (1 + q(3));
    return mrp;
  }
  Eigen::Vector4d addQuaternionError(Eigen::Vector3d& mrp, Eigen::Vector4d& qref) const {
    Eigen::Vector4d qnew, dq;

    dq = mrp2quaternion(mrp);
    qnew = quaternionMultiplication(dq, qref);
    return qnew;
  }
  Eigen::Vector4d quaternionMultiplication(Eigen::Vector4d& q1, Eigen::Vector4d& q2) const {
    q1 \mult q2 97 Eigen::Matrix4d qm;
    Eigen::Vector4d result;
    qm << q1(3), q1(2), -q1(1), q1(0), 100 - q1(2), q1(3), q1(0), q1(1), 101 q1(1), -q1(0), q1(3),
        q1(2), 102 - q1(0), -q1(1), -q1(2), q1(3);
    result = qm * q2;
    result /= result.norm();
    return result;
  }
  Eigen::Vector4d quaternionDivision(Eigen::Vector4d& q1, Eigen::Vector4d& q2) const {
    Eigen::Vector4d q2inv;
    q2inv << -q2(0), -q2(1), -q2(2), q2(3);
    Eigen::Vector4d result = quaternionMultiplication(q1, q2inv);
    return result;
  }
  Eigen::Matrix3d rotationMatrix(Eigen::Vector4d& q) const {
    Eigen::Matrix3d rot;
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

  Eigen::Vector3d r() const { return _r; }
  Eigen::Vector3d a() const { return _a; }
  Eigen::Vector4d qref() const { return _qref; }

  void reset_qref() {
    Eigen::Vector3d a_ = _a;
    Eigen::Vector4d qref_ = _qref;
    _qref = addQuaternionError(a_, qref_);
    _a = Eigen::Vector3d::Zero();
  }
  Eigen::Vector4d qTotal() const {
    Eigen::Vector3d a_ = _a;
    Eigen::Vector4d qref_ = _qref;
    return addQuaternionError(a_, qref_);
  };
  kinematicPose3d exmap(const Vector6d& ∆) {
    kinematicPose3d res = *this;
    res._r += ∆.head(3);
    res._a += ∆.tail(3);
    return res;
  }
  kinematicPose3d exmap_reset(const Vector6d& ∆) {
    kinematicPose3d res = *this;
    res._r += ∆.head(3);
    res._a += ∆.tail(3);
    res.reset_qref();
    return res;
  }
  Vector6d vector() const {
    Vector6d tmp;
    tmp << _r, _a;
    return tmp;
  }
  void set(const Vector6d& v) {
    _r = v.head(3);
    _a = v.tail(3);
  }
  void write(std::ostream& out) const {
    out << std::endl << "kinPose3d x: " << x().transpose() << std::endl;
    out << "kinPose3d qref: " << qref().transpose() << std::endl;
    out << std::endl;
  }
  /** 240 * Convert Pose3 to homogeneous 4x4 transformation matrix. 241 * The returned
                matrix is the object coordinate frame in the world 242 * coordinate frame. In other
                words it transforms a point in the object 243 * frame to the world frame. 244 * 245
                *
                @return wTo  */
  Eigen::Matrix4d wTo() const {
    /*  Eigen::Matrix4d T;  Eigen::Vector4d qtot = qTotal();  T.topLeftCorner(3,3) =
           rotationMatrix(qtot).transpose();  T.col(3).head(3) = _r;  T.row(3) << 0., 0.,
           0., 1.;  return T;  */
    Eigen::Vector4d qtot = qTotal();
    Eigen::Matrix3d R = rotationMatrix(qtot);
    Eigen::Matrix3d oRw = R;
    Eigen::Vector3d C = -oRw * _r;
    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = oRw;
    T.col(3).head(3) = C;
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }
  /* Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo.
            * The returned matrix is the world coordinate frame in the object * coordinate
             frame. In other words it transforms a point in the world  * frame to the object
             frame.  *  * @return oTw  */
  Eigen::Matrix4d oTw() const {
    Eigen::Matrix4d T;
    Eigen::Vector4d qtot = qTotal();
    T.topLeftCorner(3, 3) = rotationMatrix(qtot).transpose();
    T.col(3).head(3) = _r;
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }
};
typedef NodeExmapT<kinematicPose3d> kinematicPose3d_Node;
class kinematicPose3d_Factor : public FactorT<kinematicPose3d> {
 public:
  kinematicPose3d_Node* _pose;
  kinematicPose3d_Factor(kinematicPose3d_Node* pose,
                         const kinematicPose3d& prior,
                         const Noise& noise)
      : FactorT<kinematicPose3d>("kinematicPose3d_Factor", 6, noise, prior), _pose(pose) {
    _nodes.resize(1);
    _nodes[0] = pose;
  }
  void initialize() {
    if (!_pose->initialized()) {
      kinematicPose3d predict = _measure;
      _pose->init(predict);
    }
  }
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    kinematicPose3d p1 = _pose->value(s);
    Eigen::VectorXd err = p1.vector() - _measure.vector();
    Eigen::Vector4d q1_tot = p1.qTotal();
    Eigen::Vector4d qm_tot = _measure.qTotal();
    Eigen::Vector4d dq = p1.quaternionDivision(q1_tot, qm_tot);
    Eigen::Vector3d da = p1.quaternion2mrp(dq);
    err.segment<3>(3) = da;
    return err;
  }
};

}  // namespace isam