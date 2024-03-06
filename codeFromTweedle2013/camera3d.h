#pragma once 2 3 #include < Eigen / Dense> 4 5 #include < isam / Node.h> 6 #include <            \
    isam / Factor.h> 7 #include < isam / Pose3d.h> 8 #include <                                  \
    isam / Point3d.h> 9 #include "dynamicPose3d_NL.h" 10 #include <                              \
    isam / slam_stereo.h> 11 12 namespace isam{13 14 class cameraPose3d{                         \
               15 frend std::ostream& operator<< (std::ostream & out,                            \
                                                  const cameraPose3d& p){16 p.write(out);        \
                                                                         17 return out;          \
                                                                         18 } 19 20 Point3d _t;  \
               21 Rot3d _rot;                                                                    \
               22 public : 23 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 24 25 static const int dim = 3;    \
               26 static const char *                                                            \
               name(){                                                                           \
                   27 return "cameraPose3d"; 28 } 29 30 cameraPose3d() : _t(0, 0, 0),            \
                   _rot(0,                                                                       \
                        0,                                                                       \
                        0){} 31 32 cameraPose3d(double x,                                        \
                                                double y,                                        \
                                                double z,                                        \
                                                double yaw,                                      \
                                                double pitch,                                    \
                                                double roll) : _t(x, y, z),                      \
                           _rot(yaw, pitch, roll){} 33 34 cameraPose3d(                          \
                               const Eigen::MatrixXd& m){35 if (m.rows() == 6 && m.cols() == 1){ \
                               36 _t = Point3d(m(0), m(1), m(2));                                \
                               37 _rot = Rot3d(m(3), m(4), m(5));                                \
                               38 } else if (m.rows() == 4 && m.cols() == 4){                    \
                               39  // Convert a homogeneous 4x4 transformation matrix to a Pose3. 40
                                   // Eigen::Matrix4d wTo = m / m(3,3); // enforce T(3,3)=1 41
                                   // Eigen::Vector3d t = wTo.col(3).head(3); 42 Eigen::Matrix3d wRo
                                   // = wTo.topLeftCorner(3,3); 43 _t = Point3d(t(0), t(1), t(2));
_rot = Rot3d(wRo);
45
}
else {
  46 require(false, "Pose3d constructor called with matrix of wrong dimension");
  47
}
48
}
49 50 explicit cameraPose3d(const Eigen::Isometry3d& T) {
  51 Eigen::Vector3d t(T.translation());
  52 _t = Point3d(t(0), t(1), t(2));
  53 _rot = Rot3d(T.rotation());
  54
}
55 56 cameraPose3d(const Point3d& t, const Rot3d& rot) : _t(t), _rot(rot) {}
57 58 double x() const { return _t.x(); }
59 double y() const { return _t.y(); }
60 double z() const { return _t.z(); }
61 double yaw() const { return _rot.yaw(); }
62 double pitch() const { return _rot.pitch(); }
63 double roll() const { return _rot.roll(); }
64 65 Point3d trans() const { return _t; }
66 Rot3d rot() const { return _rot; }
67 68 void set_x(double x) { _t.set_x(x); }
69 void set_y(double y) { _t.set_y(y); }
70 void set_z(double z) { _t.set_z(z); }
71 void set_yaw(double yaw) { _rot.set_yaw(yaw); }
72 void set_pitch(double pitch) { _rot.set_pitch(pitch); }
73 void set_roll(double roll) { _rot.set_roll(roll); }
74 75 cameraPose3d exmap(const Eigen::Vector3d& ∆) {
  76 cameraPose3d res = *this;
  77 res._t = res._t.exmap(∆.head(3));
  78  // res._rot = res._rot.exmap(∆.tail(3)); 79 return res; 80 } 81 82 Eigen::Vector3d vector()
      // const { 83 // double Y, P, R; 84 // cheaper to recover ypr at once 85 //_rot.ypr(Y, P, R);
      // 86 Eigen::Vector3d tmp; 87 tmp << x(), y(), z();//, Y, P, R; 88 return tmp;
}
90 91 void set(double x, double y, double z, double yaw, double pitch, double roll) {
  92 _t = Point3d(x, y, z);
  93 _rot = Rot3d(yaw, pitch, roll);
  94
}
95 96 void set(const Eigen::Vector3d& v) {
  97 _t = Point3d(v(0), v(1), v(2));
  98  //_rot = Rot3d(standardRad(v(3)), standardRad(v(4)), standardRad(v(5))); 99 } 100 101 void
      // of_pose2d(const Pose2d& p) { 102 set(p.x(), p.y(), 0., p.t(), 0., 0.); 103 } 104 105 void
      // of_point2d(const Point2d& p) { 106 set(p.x(), p.y(), 0., 0., 0., 0.); 107 } 108 109 void
      // of_point3d(const Point3d& p) { 110 set(p.x(), p.y(), p.z(), 0., 0., 0.); 111 } 112 113 void
      // write(std::ostream &out) const { 114 out << x() << ", " << y() << ", " << z() << "; " 115
      // << yaw() << ", " << pitch() << ", " << roll(); 116 } 117 118 /** 119 * Convert Pose3 to
      // homogeneous 4x4 transformation matrix. 120 * The returned matrix is the object coordinate
      // frame in the world 121 * coordinate frame. In other words it transforms a point in the
      // object 122 * frame to the world frame. 123 * 124 * @return wTo 125 */ 126 Eigen::Matrix4d
      // wTo() const { 127 Eigen::Matrix4d T; 128 T.topLeftCorner(3,3) = _rot.wRo(); 129
      // T.col(3).head(3) << x(), y(), z(); 130 T.row(3) << 0., 0., 0., 1.; 131 return T; 132 }
      /** 135 * Convert Pose3 to homogeneous 4x4 transformation matrix. Avoids inverting wTo. 136 *
         The returned matrix is the world coordinate frame in the object 137 * coordinate frame. In
         other words it transforms a point in the world 138 * frame to the object frame. 139 * 140 *
         @return oTw 141 */
      142 Eigen::Matrix4d
      oTw() const {
    143 Eigen::Matrix3d oRw = _rot.wRo().transpose();
    144 Eigen::Vector3d t(x(), y(), z());
    145 Eigen::Vector3d C = -oRw * t;
    146 Eigen::Matrix4d T;
    147 T.topLeftCorner(3, 3) = oRw;
    148 T.col(3).head(3) = C;
    149 T.row(3) << 0., 0., 0., 1.;
    150 return T;
    151
  }
  152 153 /** 154 * Calculate new pose b composed from this pose (a) and the odometry d. 155 *
             Follows notation of Lu&Milios 1997. 156 * \f$ b = a \oplus d \f$ 157 * @param d Pose
             difference to add. 158 * @return d transformed from being local in this frame (a) to
             the global frame. 159 */
      160 Pose3d
      oplus(const Pose3d& d) const {
    161 return Pose3d(wTo() * d.wTo());
    162
  }
  163 164 /** 165 * Odometry d from b to this pose (a). Follows notation of 166 * Lu&Milios 1997.
             167 * \f$ d = a \ominus b \f$ 168 * @param b Base frame. 169 * @return Global this (a)
             expressed in base frame b. 170 */
      171 Pose3d
      ominus(const Pose3d& b) const {
    172 return Pose3d(b.oTw() * wTo());
    173
  }
  174 175 /** 176 * Project point into this coordinate frame. 177 * @param p Point to project 178 *
           * @return Point p locally expressed in this frame.
           */
      180 Point3dh
      transform_to(const Point3dh& p) const {
    181 return Point3dh(oTw() * p.vector());
    182
  }
  183 184 185 /** 186 * Project point into this coordinate frame. 187 * @param p Point to project
                 188 * @return Point p locally expressed in this frame. 189 */
      190 Point3d
      transform_to(const Point3d& p) const {
    191 return transform_to(Point3dh(p)).to_point3d();
    192
  }
  193 194 /** 195 * Project point from this coordinate frame. 196 * @param p Point to project 197 *
             @return Point p is expressed in the global frame. 198 */
      199 Point3dh
      transform_from(const Point3dh& p) const {
    200 return Point3dh(wTo() * p.vector());
    201
  }
  202 203 /** 204 * Project point from this coordinate frame. 205 * @param p Point to project 206 *
             @return Point p is expressed in the global frame. 207 */
      208 Point3d
      transform_from(const Point3d& p) const {
    209 return transform_from(Point3dh(p)).to_point3d();
    210
  }
  211 212 Pose3d getPose3d() {
    213 Pose3d val(this->x(), this->y(), this->z(), this->yaw(), this->pitch(), this->roll());
    214 return val;
    215
  }
  216 217
};
218 219 typedef NodeT<cameraPose3d> cameraPose3d_Node;
220 221 class cameraPose_Factor : public FactorT<cameraPose3d> {
  222 cameraPose3d_Node* _pose;

 public:
  225 226 cameraPose_Factor(cameraPose3d_Node* pose,
                            const cameraPose3d& prior,
                            const Noise& noise) 227
      : FactorT<cameraPose3d>("CameraPose3d_Factor", 3, noise, prior),
        _pose(pose) {
    228 _nodes.resize(1);
    229 _nodes[0] = pose;
    230
  }
  231 232 void initialize() {
    233 if (!_pose->initialized()) {
      234 cameraPose3d predict = _measure;
      235 _pose->init(predict);
      236
    }
    237
  }
  238 239 Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    240 Eigen::VectorXd err = _nodes[0]->vector(s).head(3) - _measure.vector().head(3);
    241 return err;
    242
  }
  243 244
};
245 246
}