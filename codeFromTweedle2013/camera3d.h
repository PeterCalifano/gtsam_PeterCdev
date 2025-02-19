#pragma once
#include <isam/Node.h>
#include <isam/Point3d.h>
#include <isam/Pose3d.h>
#include <isam/slam_stereo.h>
#include < Eigen/Dense>
#include <isam/Factor.h >
#include "dynamicPose3d_NL.h"

namespace isam {
class cameraPose3d {
  frend std::ostream& operator<<(std::ostream& out, const cameraPose3d& p) {
    p.write(out);
    return out;
  }
  Point3d _t;
  Rot3d _rot;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int dim = 3;
  static const char* name() { return "cameraPose3d"; }
  cameraPose3d() : _t(0, 0, 0), _rot(0, 0, 0) {}
  cameraPose3d(double x, double y, double z, double yaw, double pitch, double roll)
      : _t(x, y, z), _rot(yaw, pitch, roll) {}
  cameraPose3d(const Eigen::MatrixXd& m) {
    if (m.rows() == 6 && m.cols() == 1) {
      _t = Point3d(m(0), m(1), m(2));
      _rot = Rot3d(m(3), m(4), m(5));
    } else if (m.rows() == 4 && m.cols() == 4) {
      // Convert a homogeneous 4x4 transformation matrix to a Pose3.
      Eigen::Matrix4d wTo = m / m(3, 3);  // enforce T(3,3)=1
      Eigen::Vector3d t = wTo.col(3).head(3);
      Eigen::Matrix3d wRo = wTo.topLeftCorner(3, 3);
      _t = Point3d(t(0), t(1), t(2));
      _rot = Rot3d(wRo);

    } else {
      require(false, "Pose3d constructor called with matrix of wrong dimension");
    }
  }
  explicit cameraPose3d(const Eigen::Isometry3d& T) {
    Eigen::Vector3d t(T.translation());
    _t = Point3d(t(0), t(1), t(2));
    _rot = Rot3d(T.rotation());
  }
  cameraPose3d(const Point3d& t, const Rot3d& rot) : _t(t), _rot(rot) {}
  double x() const { return _t.x(); }
  double y() const { return _t.y(); }
  double z() const { return _t.z(); }
  double yaw() const { return _rot.yaw(); }
  double pitch() const { return _rot.pitch(); }
  double roll() const { return _rot.roll(); }
  Point3d trans() const { return _t; }
  Rot3d rot() const { return _rot; }
  void set_x(double x) { _t.set_x(x); }
  void set_y(double y) { _t.set_y(y); }
  void set_z(double z) { _t.set_z(z); }
  void set_yaw(double yaw) { _rot.set_yaw(yaw); }
  void set_pitch(double pitch) { _rot.set_pitch(pitch); }
  void set_roll(double roll) { _rot.set_roll(roll); }
  cameraPose3d exmap(const Eigen::Vector3d& ∆) {
    cameraPose3d res = *this;
    res._t = res._t.exmap(∆.head(3));
    res._rot = res._rot.exmap(∆.tail(3));
    return res;
  }
  Eigen::Vector3d vector() const {
    double Y, P, R;  // cheaper to recover ypr at once

    _rot.ypr(Y, P, R);
    Eigen::Vector3d tmp;
    tmp << x(), y(), z();  //, Y, P, R;
    return tmp;
  }
  void set(double x, double y, double z, double yaw, double pitch, double roll) {
    _t = Point3d(x, y, z);
    _rot = Rot3d(yaw, pitch, roll);
  }
  void set(const Eigen::Vector3d& v) {
    _t = Point3d(v(0), v(1), v(2));
    _rot = Rot3d(standardRad(v(3)), standardRad(v(4)), standardRad(v(5)));
  }
  void of_pose2d(const Pose2d& p) { set(p.x(), p.y(), 0., p.t(), 0., 0.); }
  void of_point2d(const Point2d& p) { set(p.x(), p.y(), 0., 0., 0., 0.); }
  void of_point3d(const Point3d& p) { set(p.x(), p.y(), p.z(), 0., 0., 0.); }
  void write(std::ostream& out) const {
    out << x() << ", " << y() << ", " << z() << "; " << yaw() << ", " << pitch() << ", " << roll();
  }
  /**  * Convert Pose3 to
  homogeneous 4x4 transformation matrix.  * The returned matrix is the object
  coordinate frame in the world  * coordinate frame. In other words it transforms a
  point in the object  * frame to the world frame.  *  * @return wTo  */
  Eigen::Matrix4d wTo() const {
    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = _rot.wRo();
    T.col(3).head(3) << x(), y(), z();
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }
  /* Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo.
    The returned matrix is the world coordinate frame in the object coordinate
    frame. In other words it transforms a point in the world  frame to the object
    frame. @return oTw */
  Eigen::Matrix4d oTw() const {
    Eigen::Matrix3d oRw = _rot.wRo().transpose();
    Eigen::Vector3d t(x(), y(), z());
    Eigen::Vector3d C = -oRw * t;
    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = oRw;
    T.col(3).head(3) = C;
    T.row(3) << 0., 0., 0., 1.;
    return T;
  }
  /* Calculate new pose b composed from this pose (a) and the odometry d.
   Follows notation of Lu&Milios 1997. \f$ b = a \oplus d \f$  @param d
   Pose difference to add.
   @return d transformed from being local in this frame (a) to the global frame.  */
  Pose3d oplus(const Pose3d& d) const { return Pose3d(wTo() * d.wTo()); }
  /* Odometry d from b to this pose (a). Follows notation of  Lu&Milios
    1997. 1\f$ d = a \ominus b \f$  @param b Base frame. @return Global
    this (a) expressed in base frame b. */
  Pose3d ominus(const Pose3d& b) const { return Pose3d(b.oTw() * wTo()); }
  /* Project point into this coordinate frame. 177 * @param p Point to project
    @return Point p locally expressed in this frame.
  */
  Point3dh transform_to(const Point3dh& p) const { return Point3dh(oTw() * p.vector()); }
  /* Project point into this coordinate frame.
   * @param p Point to project
                  @return Point p locally expressed in this frame.  */
  Point3d transform_to(const Point3d& p) const { return transform_to(Point3dh(p)).to_point3d(); }
  /**  * Project point from this coordinate frame.  * @param p Point to project
              *
             @return Point p is expressed in the global frame.  */
  Point3dh transform_from(const Point3dh& p) const { return Point3dh(wTo() * p.vector()); }
  /**  * Project point from this coordinate frame.  * @param p Point to project
             *
            @return Point p is expressed in the global frame.  */
  Point3d transform_from(const Point3d& p) const {
    return transform_from(Point3dh(p)).to_point3d();
  }
  Pose3d getPose3d() {
    Pose3d val(this->x(), this->y(), this->z(), this->yaw(), this->pitch(), this->roll());
    return val;
  }
};
typedef NodeT<cameraPose3d> cameraPose3d_Node;
class cameraPose_Factor : public FactorT<cameraPose3d> {
  cameraPose3d_Node* _pose;

 public:
  cameraPose_Factor(cameraPose3d_Node* pose, const cameraPose3d& prior, const Noise& noise)
      : FactorT<cameraPose3d>("CameraPose3d_Factor", 3, noise, prior), _pose(pose) {
    _nodes.resize(1);
    _nodes[0] = pose;
  }
  void initialize() {
    if (!_pose->initialized()) {
      cameraPose3d predict = _measure;
      _pose->init(predict);
    }
  }
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Eigen::VectorXd err = _nodes[0]->vector(s).head(3) - _measure.vector().head(3);
    return err;
  }
};

}  // namespace isam