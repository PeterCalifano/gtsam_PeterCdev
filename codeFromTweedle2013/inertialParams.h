#pragma once 2 3 #include < cmath> 4 #include < ostream> 5 #include < iostream> 6 7 #include < \
    isam / util.h> 8 #include "math.h" 9 #include < Eigen / Dense> 10 11 using namespace isam; \
    12 using namespace Eigen; 13 14 class principalAxesFrame{                                  \
        15 frend std::ostream &                                                                \
        operator<< (std::ostream & out, const principalAxesFrame& p){16 p.write(out);          \
                                                                     17 return out;            \
                                                                     18 } 19 20 Vector3d       \
                _r;  // position - from the target frame to the principal frame 21 Vector4d _q;
                     // //quaternion - from the target frame to the principal frame 22 23 //3
                     // parameter attitude error 24 Vector3d _a; //Modified Rodrigues Parameter 25
                     // 26 public: 27 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 28 static const int dim = 6;
                     // 29 static const char* name() { 30 return "principalAxesFrame"; 31 } 32
                     // Matrix<double, 6, 6> _sqrtinf; 33 34 principalAxesFrame() { 35 _r << 0.0,
                     // 0.0, 0.0; 36 _a << 0.0, 0.0, 0.0; 37 _q << 0.0, 0.0, 0.0, 1.0; 38 } 39 40
                     // principalAxesFrame(Matrix<double,3,1> r) { 41 _r = r; 42 } 43 44
                     // principalAxesFrame(Matrix<double,3,1> r, Vector4d q) {
_r = r;
46 _q = q;
47
}
48 49 VectorXd vector() const {
  50 Matrix<double, 6, 1> x;
  51 x << _r, _a;
  52 return x;
  53
}
54 55 void set(const VectorXd& v) {
  56 _r = v.block<3, 1>(0, 0);
  57 _a = v.block<3, 1>(3, 0);
  58
}
59 60 61 Matrix<double, 6, 1> x() {
  62 Matrix<double, 6, 1> x;
  63 x << _r, _a;
  64 return x;
  65
}
66 67 Vector4d q() {
  68 return _q;
  69
}
70 71 Vector4d mrp2quaternion(Vector3d mrp) const {
  72 Vector4d dq;
  73 dq << 8 * mrp / (16 + mrp.squaredNorm()), (16 - mrp.squaredNorm()) / (16 + mrp.squaredNorm());
  74 return dq;
  75
}
76 77 Vector4d addQuaternionError(Vector3d mrp, Vector4d qref) const {
  78 Vector4d qnew, dq;
  79 dq = mrp2quaternion(mrp);
  80 81 qnew = quaternionMultiplication(dq, qref);
  82 return qnew;
  83
}
84 85 86 principalAxesFrame exmap(const Matrix<double, 6, 1>& ∆) const {
  87 principalAxesFrame res = *this;
  88 res._r += ∆.block<3, 1>(0, 0);
  res._a += ∆.block<3, 1>(3, 0);
  90 return res;
  91
}
92 93 principalAxesFrame exmap_reset(const Matrix<double, 6, 1>& ∆) {
  94 principalAxesFrame res = *this;
  95 96 res._r += ∆.block<3, 1>(0, 0);
  97 res._a += ∆.block<3, 1>(3, 0);
  98 99 res.write();
  100 101  // reset step 102 res._q = addQuaternionError(res._a, res._q); 103 res._a =
           // Vector3d::Zero(); 104 105 printf("inertial reset\n"); 106 107 return res; 108 } 109
           // 110 111 void write(std::ostream &out = std::cout) const { 112 out << " " <<
           // _r.transpose(); 113 out << " " << _q(0) << " " << _q(1) << " " << _q(2) << " " <<
           // _q(3); 114 out << " " << _a.transpose(); 115 out << std::endl; 116 } 117 118 Vector4d
           // quaternionMultiplication(Vector4d q1, Vector4d q2) const { 119 //q1 \mult q2 120
           // Matrix4d qm; 121 Vector4d result; 122 qm << q1(3), q1(2), -q1(1), q1(0), 123 -q1(2),
           // q1(3), q1(0), q1(1), 124 q1(1), -q1(0), q1(3), q1(2), 125 -q1(0), -q1(1), -q1(2),
           // q1(3); 126 127 result = qm*q2; 128 result /= result.norm(); 129 130 return result; 131
           // } 132 133 Matrix3d rotationMatrix(Vector4d q) const {
      Matrix3d rot;
  135 136 rot(0, 0) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  137 rot(0, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
  138 rot(0, 2) = 2 * (q(0) * q(2) - q(1) * q(3));
  139 140 rot(1, 0) = 2 * (q(0) * q(1) - q(2) * q(3));
  141 rot(1, 1) = -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  142 rot(1, 2) = 2 * (q(2) * q(1) + q(0) * q(3));
  143 144 rot(2, 0) = 2 * (q(0) * q(2) + q(1) * q(3));
  145 rot(2, 1) = 2 * (q(2) * q(1) - q(0) * q(3));
  146 rot(2, 2) = -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
  147 148  // std::cout << "q2rot: " << q << rot << std::endl; 149 return rot; 150 } 151 152 Point3d
           // toPrincipalFrame(const Point3d& p_m) const { 153 Matrix3d R =
           // rotationMatrix(addQuaternionError(_a,_q)); 154 Vector3d vecBody = R * (p_m.vector() -
           // _r); 155 Point3d p_c(vecBody); 156 157 return p_c; 158 } 159 160 Point3d
           // fromPrincipalFrame(const Point3d& p_m) const { 161 Matrix3d R =
           // rotationMatrix(addQuaternionError(_a,_q)); 162 Vector3d vecBody = R.transpose() *
           // p_m.vector() + _r; 163 Point3d p_c(vecBody); 164 165 return p_c; 166 } 167 168 };