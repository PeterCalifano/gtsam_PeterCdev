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
    193 Eigen::Vector3d a_ = _a;
    194 Eigen::Vector4d qref_ = _qref;
    195 _qref = addQuaternionError(a_, qref_);
    196 _a = Eigen::Vector3d::Zero();
    197
  }
  198 199 Eigen::Vector4d qTotal() const {
    200 Eigen::Vector3d a_ = _a;
    201 Eigen::Vector4d qref_ = _qref;
    202 return addQuaternionError(a_, qref_);
    203
  };
  204 205 206 207 kinematicPose3d exmap(const Vector6d& ∆) {
    208 kinematicPose3d res = *this;
    209 res._r += ∆.head(3);
    210 res._a += ∆.tail(3);
    211 return res;
    212
  }
  213 214 kinematicPose3d exmap_reset(const Vector6d& ∆) {
    215 kinematicPose3d res = *this;
    216 res._r += ∆.head(3);
    217 res._a += ∆.tail(3);
    218 res.reset_qref();
    219 return res;
  }
  221 Vector6d vector() const {
    222 Vector6d tmp;
    223 tmp << _r, _a;
    224 return tmp;
    225
  }
  226 227 void set(const Vector6d& v) {
    228 _r = v.head(3);
    229 _a = v.tail(3);
    230
  }
  231 232 void write(std::ostream& out) const {
    233 out << std::endl << "kinPose3d x: " << x().transpose() << std::endl;
    234 out << "kinPose3d qref: " << qref().transpose() << std::endl;
    235 out << std::endl;
    236
  }
  237 238 239 /** 240 * Convert Pose3 to homogeneous 4x4 transformation matrix. 241 * The returned
                 matrix is the object coordinate frame in the world 242 * coordinate frame. In other
                 words it transforms a point in the object 243 * frame to the world frame. 244 * 245
                 *
                 @return wTo 246 */
      247 Eigen::Matrix4d
      wTo() const {
    248 /* 249 Eigen::Matrix4d T; 250 Eigen::Vector4d qtot = qTotal(); 251 T.topLeftCorner(3,3) =
           rotationMatrix(qtot).transpose(); 252 T.col(3).head(3) = _r; 253 T.row(3) << 0., 0.,
           0., 1.; 254 return T; 255 */
        256 Eigen::Vector4d qtot = qTotal();
    257 Eigen::Matrix3d R = rotationMatrix(qtot);
    258 Eigen::Matrix3d oRw = R;
    259 Eigen::Vector3d C = -oRw * _r;
    260 Eigen::Matrix4d T;
    261 T.topLeftCorner(3, 3) = oRw;
    262 T.col(3).head(3) = C;
    263 T.row(3) << 0., 0., 0., 1.;
    264 return T;
  }
  267 268 /** 269 * Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo.
             270 * The returned matrix is the world coordinate frame in the object 271 * coordinate
             frame. In other words it transforms a point in the world 272 * frame to the object
             frame. 273 * 274 * @return oTw 275 */
      276 Eigen::Matrix4d
      oTw() const {
    277 Eigen::Matrix4d T;
    278 Eigen::Vector4d qtot = qTotal();
    279 T.topLeftCorner(3, 3) = rotationMatrix(qtot).transpose();
    280 T.col(3).head(3) = _r;
    281 T.row(3) << 0., 0., 0., 1.;
    282 return T;
    283 284
  }
  285 286 287
};
288 289 typedef NodeExmapT<kinematicPose3d> kinematicPose3d_Node;
290 291 class kinematicPose3d_Factor : public FactorT<kinematicPose3d> {
  292 public : 293 kinematicPose3d_Node* _pose;
  294 295 kinematicPose3d_Factor(kinematicPose3d_Node* pose,
                                 const kinematicPose3d& prior,
                                 const Noise& noise) 296
      : FactorT<kinematicPose3d>("kinematicPose3d_Factor", 6, noise, prior),
        _pose(pose) {
    297 _nodes.resize(1);
    298 _nodes[0] = pose;
    299
  }
  300 301 void initialize() {
    302 if (!_pose->initialized()) {
      303 kinematicPose3d predict = _measure;
      304 _pose->init(predict);
      305
    }
    306
  }
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    309 310 kinematicPose3d p1 = _pose->value(s);
    311 Eigen::VectorXd err = p1.vector() - _measure.vector();
    312 Eigen::Vector4d q1_tot = p1.qTotal();
    313 Eigen::Vector4d qm_tot = _measure.qTotal();
    314 Eigen::Vector4d dq = p1.quaternionDivision(q1_tot, qm_tot);
    315 Eigen::Vector3d da = p1.quaternion2mrp(dq);
    316 317 err.segment<3>(3) = da;
    318 319 return err;
    320
  }
  321
};
322
}  // namespace isam