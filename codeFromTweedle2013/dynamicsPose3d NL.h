#pragma once 3 4 #include < ostream> 5 #include <
Eigen / Dense > 6 #include "isam/isam.h" 7 #include "rigidBodyDynamics.h" 8 #include
    "FactorVariableNoise.h" 9 10 11 using namespace Eigen;
12 namespace isam {
  13 14 class dynamicPose3d_NL {
    15 frend std::ostream& operator<<(std::ostream& out, const dynamicPose3d_NL& p) 16 {
      17 p.write(out);
      18 return out;
      19
    }
    20 21 rigidBodyDynamics rbd;
    22 public : 23 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // assignment operator and copy
        // constructor implicitly
        // created, which is ok
        static const int dim = 12;
    26 static const char* name() {
      27 return "dynamicPose3d_NL";
      28
    }
    29 30 Noise* factor_noise;  // check
    if
      this is ever used 31 32 dynamicPose3d_NL(inertiaRatios ir, double sigma_v, double sigma_w)
          : rbd(ir, sigma_v, sigma_w) {
        33
      }
    34 35  // copy
        constructor 36 dynamicPose3d_NL(const dynamicPose3d_NL& cSource) : 37 rbd(
            cSource.rbd.getIR(), cSource.rbd.getSigmaV(), cSource.rbd.getSigmaW()) 38 {
      39 rbd.setState(cSource.rbd.x(), cSource.rbd.qref());
      40
    }
    41 42 dynamicPose3d_NL& operator=(const dynamicPose3d_NL& cSource) {
      43 rbd =
          rigidBodyDynamics(cSource.rbd.getIR(), cSource.rbd.getSigmaV(), cSource.rbd.getSigmaW());
      44 rbd.setState(cSource.rbd.x(), cSource.rbd.qref());
      45 return *this;
      46
    }
    47 48 dynamicPose3d_NL(VectorXd x, inertiaRatios ir, double sigma_v, double sigma_w) 49
        : rbd(ir, sigma_v, sigma_w) 50 {
      51 Vector4d qref;
      52 qref << 0, 0, 0, 1;
      53 if (x.size() == 12) {
        54 rbd.setState(x, qref);
        55
      }
      56
    }
    57 58 dynamicPose3d_NL(VectorXd x,
                           Vector4d qref,
                           inertiaRatios ir,
                           double sigma_v,
                           double sigma_w) 59 : rbd(ir, sigma_v, sigma_w) 60 {
      61 if (x.size() == 12) {
        62 rbd.setState(x, qref);
        63
      }
      64
    }
    65 66 dynamicPose3d_NL(const Matrix4d& hm,
                           bool initVelocities,
                           double dt,
                           inertiaRatios ir,
                           double sigma_v,
                           double sigma_w) 67 : rbd(ir, sigma_v, sigma_w) 68 {
      69 Matrix<double, 12, 1> x;
      70 Vector3d r;
      71 Vector3d v;
      72 Vector3d a;
      73 Vector4d q;
      74 Vector3d w;
      // Convert matrix to R,T
      77 Matrix4d HM = hm / hm(3, 3);  // enforce
      T(3, 3) = 1 78 Matrix3d R = HM.topLeftCorner(3, 3);
      79 Vector3d T = HM.col(3).head(3);
      80 81  // compute
          quaternion 82 q = rbd.quaternionFromRot(R);
      83 a = Vector3d::Zero();
      84 85 if (initVelocities) {  // differentiate linear velocity
        87 v = T / dt;
        88 89 /*
                      // Differentiate quaternion: 90 * dq/dt = (q[k] - q[k-1])/dt =
                       0.5 O(w[k-1]) q[k-1] 91 * where O(w[k-1]) is an orthonormal
                       quaternion mult matrix for [w1; w2; w3; 0] (i.e.
                       quaternionMultiplication()) 92 * set q[k-1] = [0;0;0;1] by
                       definition (from a refererence frame) and solve for w[k-1]
                       gives 93 * w[k-1] = 2 [q1[k]; q2[k]; q3[k]] / dt 94 */
            95 w = 2 * q.head(3) / dt;
        96
      }
      else {
        97 v = Vector3d::Zero();
        98 w = Vector3d::Zero();
        99
      }
      100 101 x.block<3, 1>(0, 0) = T;
      102 x.block<3, 1>(3, 0) = v;
      103 x.block<3, 1>(6, 0) = a;
      104 x.block<3, 1>(9, 0) = w;
      105 rbd.setState(x, q);
      106
    }
    107 108 VectorXd x() { return rbd.x(); };
    109 Vector4d q() { return rbd.qref(); };
    110 Vector4d qTotal() const { return rbd.qTotal(); };
    111 112 dynamicPose3d_NL exmap(const Matrix<double, 12, 1>& ∆) const {
      113 dynamicPose3d_NL res = *this;
      114 res.rbd.setState(res.rbd.x() + ∆);
      115 return res;
      116
    }
    117 118 dynamicPose3d_NL exmap_reset(const Matrix<double, 12, 1>& ∆) {
      119 dynamicPose3d_NL res = *this;
      120 res.rbd.setState(res.rbd.x() + ∆);
      121 res.rbd.reset_qref();
      122 123
          /* We should REALLY, REALLY
    update Factor::_sqrtinf at this location * in the code.
    However it is a const variable, and there is no way  * to
    do callbacks to the Factor class. So I will leave this for
    future  * work. Now, the value that is created on
    initialization is the final * version, even after many
    relinearizations.  */

          // NOTE - THIS IS NOW DONE in NodeExmapT->apply_reset(); 131 132 return res; 133 } 134 135
          // void set(const VectorXd& v) { 136 rbd.setState(v); 137 } 138 139 void set_qref(const
          // Vector4d& qset) { 140 rbd.setState(rbd.x(), qset); 141 } 142 143 void rezero() { 144
          // VectorXd x = VectorXd::Zero(12); 145 Vector4d q; 146 q << 0 , 0, 0, 1; 147
          // rbd.setState(x,q); 148 } 149 150 151 dynamicPose3d_NL propagate(double dt,
          // inertiaRatios& ir) { 152 VectorXd x0 = VectorXd::Zero(90); 153 x0.head(12) = rbd.x();
          // 154 rbd.setIR(ir); 155 // std::cout << "x0: " << x0.head(12).transpose() << std::endl;
          // 156 VectorXd newX = rbd.propagateRK4_adaptive(dt, x0).head(12); 157 158 // std::cout <<
          // "dt: " << dt << std::endl; 159 // std::cout << "newX: " << newX.transpose() <<
          // std::endl; 160 161 dynamicPose3d_NL xNew(newX, this->rbd.qref(), this->rbd.getIR(),
          // this->rbd. getSigmaV(), this->rbd.getSigmaW()); 162
          // xNew.exmap(Matrix<double,12,1>::Zero()); 163 return xNew; 164 } 165 166 Vector3d
          // getMRPdifference(Vector4d qtot1, Vector4d qtot2) { 167 Vector4d dq =
          // rbd.quaternionDivision(qtot1,qtot2); 168 Vector3d da = rbd.quaternion2mrp(dq); 169
          // return da; 170 } 171 172 //compute the control input using w_t = x_{t+1} -
          // \int_tˆ{t+1}f(x_t)

          VectorXd
          computeStateChange(dynamicPose3d_NL & prev, double dt, inertiaRatios& ir) {
        VectorXd w;
        dynamicPose3d_NL predicted = prev.propagate(dt, ir);
        Vector4d qTot = this->qTotal();
        Vector4d qTotpred = predicted.qTotal();
        Vector3d da = getMRPdifference(qTot, qTotpred);
        w = this->x() - predicted.x();
        w.segment<3>(6) = da;
        return w;
      }
      Vector6d getOdometry() {
        Vector6d odo;
        VectorXd x = rbd.x();
        Vector4d qref = rbd.qref();
        Vector3d a = x.segment<3>(6);
        odo.head(3) = x.segment<3>(0);
        Vector4d qnew = rbd.addQuaternionError(a, qref);
        odo.tail(3) = rbd.quaternion2mrp(qnew);
        return odo;
      }
      Vector6d getOdometry(dynamicPose3d_NL & prev) {
        Vector6d odo;
        VectorXd x, xprev;
        Vector4d q, qprev;
        Vector3d a, aprev;
        x = rbd.x();
        xprev = prev.x();
        a = x.segment<3>(6);
        q = rbd.qref();
        qprev = prev.q();
        aprev = xprev.segment<3>(6);
        Vector3d dr = x.segment<3>(0) - xprev.segment<3>(0);
        Vector4d qtot_this = rbd.addQuaternionError(a, q);
        Vector4d qtot_prev = rbd.addQuaternionError(aprev, qprev);
        Vector4d qprev_inv;
        220 qprev_inv << -qtot_prev(0), -qtot_prev(1), -qtot_prev(2), qtot_prev(3);
        221 Vector4d qDiff = rbd.quaternionMultiplication(qtot_this, qprev_inv);
        222 Vector3d mrp = rbd.quaternion2mrp(qDiff);
        223 odo.tail(3) = mrp;
        224 225 Matrix3d Rprev = rbd.rotationMatrix(qtot_prev);
        226 odo.head(3) = Rprev.transpose() * dr;
        227 228 return odo;
        229
      }
      dynamicPose3d_NL getOdometryPose(dynamicPose3d_NL & prev, bool initVelocities, double dt) {
        232 dynamicPose3d_NL newPose(prev.rbd.getIR(), prev.rbd.getSigmaV(), prev.rbd.getSigmaW());
        233 VectorXd new_x(12);
        234 Vector3d new_r;
        235 Vector3d new_v;
        236 Vector3d new_a;
        237 Vector4d new_q;
        238 Vector3d new_w;
        239 240 VectorXd x, xprev;
        241 Vector4d q, qprev;
        242 Vector3d a, aprev;
        243 244  // get x's 245 x = rbd.x(); 246 xprev = prev.x(); 247 248 //attitude gets 249 a =
                 // x.segment<3>(6); 250 aprev = xprev.segment<3>(6); 251 q = rbd.qref(); 252 qprev
                 // = prev.q(); 253 //total attitude 254 Vector4d qtot_this =
                 // rbd.addQuaternionError(a, q); 255 Vector4d qtot_prev =
                 // rbd.addQuaternionError(aprev, qprev); 256 Vector4d qprev_inv; 257 qprev_inv <<
                 // -qtot_prev(0), -qtot_prev(1), -qtot_prev(2), qtot_prev(3); 258 Vector4d qDiff =
                 // rbd.quaternionMultiplication(qtot_this, qprev_inv); 259 //previous rotation mat
                 // 260 Matrix3d Rprev = rbd.rotationMatrix(qtot_prev);

            new_r = Rprev.transpose() * (x.segment<3>(0) - xprev.segment<3>(0));
        263 Matrix3d Rdiff = rbd.rotationMatrix(qDiff);
        264 new_q = rbd.quaternionFromRot(Rdiff);
        265 266 if (initVelocities) {
          267  // differentiate linear velocity 268 new_v = new_r / dt; 269 270 /* Differentiate
               // quaternion: 271 * dq/dt = (q[k] - q[k-1])/dt = 0.5 O(w[k-1]) q[k-1] 272 * where
               // O(w[k-1]) is an orthonormal quaternion mult matrix for [w1; w2; w3; 0] (i.e.
               // quaternionMultiplication()) 273 * set q[k-1] = [0;0;0;1] by definition (from a
               // refererence frame) and solve for w[k-1] gives 274 * w[k-1] = 2 [q1[k]; q2[k];
               // q3[k]] / dt 275 */ 276 new_w = 2*new_q.head(3) / dt; 277 } else { 278 new_v =
               // Vector3d::Zero(); 279 new_w = Vector3d::Zero(); 280 } 281 new_a =
               // Vector3d::Zero(); 282 283 new_x.block<3,1>(0,0) = new_r; 284 new_x.block<3,1>(3,0)
               // = new_v; 285 new_x.block<3,1>(6,0) = new_a; 286 new_x.block<3,1>(9,0) = new_w; 287
               // newPose.rbd.setState(new_x, new_q); 288 return newPose; 289 290 }

              dynamicPose3d_NL
              adjustAttitude(dynamicPose3d_NL & prev) {
            293 Vector4d q, qprev;
            294 dynamicPose3d_NL newPose(
                prev.rbd.getIR(), prev.rbd.getSigmaV(), prev.rbd.getSigmaW());
            295 296 VectorXd x = rbd.x();
            297 q = rbd.qTotal();
            298 qprev = prev.qTotal();
            299 300 std::cout << "q: " << q.transpose() << std::endl;
            301 std::cout << "qprev: " << qprev.transpose() << std::endl;

            Matrix3d R = rbd.rotationMatrix(q);
            304 Matrix3d Rprev = rbd.rotationMatrix(qprev);
            305 Matrix3d Rdiff = R * Rprev.transpose();
            306 Vector4d new_qdiff = rbd.quaternionFromRot(Rdiff);
            307 308 std::cout << "R: " << R << std::endl;
            309 std::cout << "Rprev: " << Rprev << std::endl;
            310 std::cout << "Rdiff: " << Rdiff << std::endl;
            311 std::cout << "new_qdiff: " << new_qdiff.transpose() << std::endl;
            312 313 Vector4d qnew = rbd.quaternionMultiplication(new_qdiff, qprev);
            314 315 std::cout << "qnew aa: " << qnew.transpose() << std::endl << std::endl;
            316 if (isnan(qnew(1))) {
              317 std::cout << "qnew aa nan\n";
              318 new_qdiff = rbd.quaternionFromRot(Rdiff);
              319
            }
            320 321 x.segment<3>(6) = Vector3d::Zero();
            322 rbd.setState(x, qnew);
            323 newPose.rbd.setState(x, qnew);
            324 return newPose;
            325 326
          }
          327 328 void shortenQuaternion(dynamicPose3d_NL & prev) {
            329 Vector4d q, qprev, qnew;
            330 331 VectorXd x = rbd.x();
            332 q = rbd.qTotal();
            333 qprev = prev.qTotal();
            334 if (q.dot(qprev) < 0) {
              335 qnew = -q;
              336 x.segment<3>(6) = Vector3d::Zero();
              337 rbd.setState(x, qnew);
              338
            }
            339
          }
          340 341 342 dynamicPose3d_NL applyOdometry(dynamicPose3d_NL & prev) {
            343 dynamicPose3d_NL newPose(
                prev.rbd.getIR(), prev.rbd.getSigmaV(), prev.rbd.getSigmaW());
            344 VectorXd new_x(12);
            345 Vector3d new_r;
            346 Vector3d new_v;

            Vector3d new_a;
            348 Vector4d new_q;
            349 Vector3d new_w;
            350 351 VectorXd x, xprev;
            352 Vector4d q, qprev;
            353 Vector3d a, aprev;
            354 355  // get x's 356 x = rbd.x(); 357 xprev = prev.x(); 358 359 //attitude gets 360 q
                     // = rbd.qTotal(); 361 qprev = prev.qTotal(); 362 363 new_q =
                     // rbd.quaternionMultiplication(q,qprev); 364 365 Matrix3d Rprev =
                     // rbd.rotationMatrix(qprev); 366 new_r = Rprev * x.head(3) + xprev.head(3);
                     // 367 368 new_v = Vector3d::Zero(); 369 new_a = Vector3d::Zero(); 370 new_w =
                     // Vector3d::Zero(); 371 372 new_x.block<3,1>(0,0) = new_r; 373
                     // new_x.block<3,1>(3,0) = new_v; 374 new_x.block<3,1>(6,0) = new_a; 375
                     // new_x.block<3,1>(9,0) = new_w; 376 377 newPose.rbd.setState(new_x, new_q);
                     // 378 return newPose; 379 } 380 381 382 Matrix4d wTo() const { 383 Matrix4d T;
                     // 384 385 //error quaternion is applied 386 Vector4d qtot = rbd.qTotal(); 387
                     // VectorXd x = rbd.x(); 388 T.topLeftCorner(3,3) =
                     // rbd.rotationMatrix(qtot).transpose(); 389 T.col(3).head(3) <<
                     // x.segment<3>(0); 390 T.row(3) << 0., 0., 0., 1.; 391 return T;
          }
          393 394 Matrix4d oTw() const {
            395 Matrix4d T;
            396 Matrix3d R;
            397 398  // error quaternion is applied 399 Vector4d qtot = rbd.qTotal(); 400 VectorXd x
                     // = rbd.x(); 401 R = rbd.rotationMatrix(qtot); 402 403 T.topLeftCorner(3,3) =
                     // R; 404 T.col(3).head(3) << - R * x.segment<3>(0); 405 T.row(3) << 0., 0.,
                     // 0., 1.; 406 return T; 407 } 408 409 410 Pose3d getPose3d() { 411 return
                     // Pose3d(this->wTo()); //may be wrong: Mar 25, 2013, B.E.T. 412 //return
                     // Pose3d(this->oTw()); 413 } 414 415 Point3dh transform_to_inertial(const
                     // Point3dh& pBody) const{ 416 Vector3d p; 417 p << pBody.x(), pBody.y(),
                     // pBody.z(); 418 Vector4d qtot = rbd.qTotal(); 419 VectorXd x = rbd.x(); 420
                     // Vector3d T = x.head(3); 421 Matrix3d Rt =
                     // rbd.rotationMatrix(qtot).transpose(); 422 423 Vector3d pInertial = Rt*p + T;
                     // 424 425 return Point3dh(pInertial(0), pInertial(1), pInertial(2), 1.0); 426
                     // } 427 428 Point3dh transform_to_body(const Point3dh& pInertial) const{ 429
                     // Vector3d p; 430 p << pInertial.x(), pInertial.y(), pInertial.z(); 431
                     // Vector4d qtot = rbd.qTotal(); 432 VectorXd x = rbd.x(); 433 Vector3d T =
                     // x.head(3); 434 Matrix3d R = rbd.rotationMatrix(qtot); 435 436 Vector3d pBody
                     // = R*(p - T);

                return Point3dh(pBody(0), pBody(1), pBody(2), 1.0);
            439
          }
          440 441 442 Noise getProcessNoise(double dt, inertiaRatios ir) {
            443 VectorXd x0 = VectorXd::Zero(90);
            444 x0.head(12) = rbd.x();
            445 rbd.setIR(ir);
            446 VectorXd newLambda = rbd.propagateRK4_adaptive(dt, x0).tail(78);
            447 448 Matrix<double, 12, 12> lambda = rbd.vec2symmMat(newLambda);
            449 Noise n = isam::Covariance(lambda);
            450 return n;
            451
          }
          452 453 VectorXd vectorFull() const {
            454 VectorXd x = rbd.x();
            455 Vector4d q = rbd.qref();
            456 Vector3d mrp = rbd.quaternion2mrp(q);
            457 x(6) += mrp(0);
            458 x(7) += mrp(1);
            459 x(8) += mrp(2);
            460 return x;
            461
          }
          462 463 VectorXd vector() const {
            464 return rbd.x();
            465
          }
          466 467 void write(std::ostream & out) const {
            468 469 out << std::endl << "dP3d_NL x: " << rbd.x().transpose() << std::endl;
            470 out << "dP3d_NL qref: " << rbd.qref().transpose() << std::endl;
            471 out << std::endl;
            472
          }
          473 474 475
    };
    476
  }