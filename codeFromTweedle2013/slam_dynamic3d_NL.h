#ifndef SLAMDYNAMICS_H_
#define SLAMDYNAMICS_H_
#include <
Eigen / Dense >
#include < isam / Factor.h >
#include < isam / Node.h >
#include < isam / Point3d.h >
#include < isam / Pose3d.h >
#include < isam / slam_stereo.h >
#include "FactorVariableNoise.h"
#include "NodeExmap.h"
#include "camera3d.h"
#include "dynamicPose3d_NL.h"
#include "inertiaRatios.h"
#include "kinematicPose3d.h"

    namespace isam {
  21 22 typedef NodeExmapT<dynamicPose3d_NL> dynamicPose3d_NL_Node;
  23 typedef NodeT<Point3d> Point3d_Node;
  24 25 /** 26 * Prior on dynamicPose3d. 27 */
      28 class dynamicPose3d_NL_Factor : public FactorT<dynamicPose3d_NL> {
    29 public : 30 dynamicPose3d_NL_Node* _pose;
    31 32 dynamicPose3d_NL_Factor(dynamicPose3d_NL_Node* pose,
                                  const dynamicPose3d_NL& prior,
                                  const Noise& noise) 33
        : FactorT<dynamicPose3d_NL>("dynamicPose3d_NL_Factor", 12, noise, prior),
          _pose(pose) {
      34 _nodes.resize(1);
      35 _nodes[0] = pose;
      36
    }
    37 38 void initialize() {
      39 if (!_pose->initialized()) {
        40 dynamicPose3d_NL predict = _measure;
        41 _pose->init(predict);
      }
      44 45 Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
        46 47 dynamicPose3d_NL p1 = _pose->value(s);
        48 Eigen::VectorXd err = p1.vectorFull() - _measure.vector();
        49 50 Eigen::Vector4d p1qTot = p1.qTotal();
        51 Eigen::Vector4d mqTot = _measure.qTotal();
        52 Vector3d da = p1.getMRPdifference(p1qTot, mqTot);
        53 err.segment<3>(6) = da;
        54 55 return err;
        56
      }
      57
    };
    58 59 60  // Process Model Factor - one of the main contributions of this thesis
        class dynamicPose3d_NL_dynamicPose3d_NL_Factor : public FactorVarNoiseT<dynamicPose3d_NL> {
      62 dynamicPose3d_NL_Node* _pose1;
      63 dynamicPose3d_NL_Node* _pose2;
      64 inertiaRatios_Node* _ir_node;
      65 double dt;
      66 67 public : 68 69 /** 70 *
Constructor. 71 * @param pose1 The pose from which the measurement starts. 72 *
// @param pose2 The pose to which the measurement extends. 73 * @param measure DOES
// NOTHING - DON'T USE IT!!!! (could be extended in future release to add
// forces/torques 74 * @param noise The 12x12 square root information matrix (upper
// triangular). 75 */
          76 dynamicPose3d_NL_dynamicPose3d_NL_Factor(dynamicPose3d_NL_Node* pose1,
                                                      dynamicPose3d_NL_Node* pose2,
                                                      inertiaRatios_Node* ir_node,
                                                      77 const dynamicPose3d_NL& measure,
                                                      const Noise& noise,
                                                      double timestep) 78
          : FactorVarNoiseT<dynamicPose3d_NL>("dp3dNL_dp3dNL_IR_Factor", 12, noise, measure),
            79 _pose1(pose1),
            _pose2(pose2),
            _ir_node(ir_node),
            dt(timestep) {
        80 _nodes.resize(3);
        81 _nodes[0] = pose1;
        82 _nodes[1] = pose2;
        83 _nodes[2] = ir_node;
      }
      85 86 void initialize() {
        87 dynamicPose3d_NL_Node* pose1 = _pose1;
        88 dynamicPose3d_NL_Node* pose2 = _pose2;
        89 inertiaRatios_Node* ir_node = _ir_node;
        90 require(pose1->initialized() || pose2->initialized(), 91 "dynamicSLAM: dynamicPose3d_NL_dynamicPose3d_NL_Factor requires pose1 or pose2 to be initialized");
        92 93 if (!_ir_node->initialized()) {
          94 inertiaRatios init_ir;
          95 _ir_node->init(init_ir);
          96
        }
        97 98 if (!pose1->initialized() && pose2->initialized()) {
          99 std::cout << "No BACKWARDS PROPAGATE" << std::endl;
          100
        }
        else if (pose1->initialized() && !pose2->initialized()) {
          101 inertiaRatios ir = _ir_node->value();
          102 dynamicPose3d_NL a = pose1->value();
          103 dynamicPose3d_NL predict = a.propagate(dt, ir);
          104 pose2->init(predict);
          105
        }
        106
      }
      107 108 Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
        109 110 dynamicPose3d_NL p1 = _pose1->value(s);
        111 dynamicPose3d_NL p2 = _pose2->value(s);
        112 inertiaRatios ir = _ir_node->value(s);
        113 114 Eigen::VectorXd err = p2.computeStateChange(p1, dt, ir);
        115 116 return err;
        117
      }
      118 119 Eigen::MatrixXd get_sqrtinf() const {
        120 inertiaRatios ir = _ir_node->value();
        121 Eigen::MatrixXd new_sqrtinf = _pose1->value().getProcessNoise(dt, ir)._sqrtinf;
        122 return new_sqrtinf;
        123
      }
      124 125 bool checkPose1(dynamicPose3d_NL_Node* poseRef) {
        126 if (_pose1 == poseRef) { 127 return true; }
        else {
          129 return false;
          130
        }
        131
      }
      132 133 bool checkPose1(inertiaRatios_Node* ir_node) {
        134 if (_ir_node == ir_node) {
          135 return true;
          136
        }
        else {
          137 return false;
          138
        }
        139
      }
      140 141 bool checkPose1(kinematicPose3d_Node* poseRef) {
        142 return false;
        143
      }
      144 145 double get_dt() { return dt; }
      146 147 void write(std::ostream& out) const {
        148 FactorVarNoiseT<dynamicPose3d_NL>::write(out);
        149
      }
      150
    };
    151 152 153 typedef NodeT<Point3dh> Point3dh_Node;
    154 155  // stereo camera class 156 class StereoCameraDebug { // for now, camera and robot are
             // identical 157 double _f; 158 Eigen::Vector2d _pp; 159 double _b; 160 161 public: 162
             // EIGEN_MAKE_ALIGNED_OPERATOR_NEW 163 164 StereoCameraDebug() : _f(1),
             // _pp(Eigen::Vector2d(0.5,0.5)), _b(0.1) {} 165 StereoCameraDebug(double f, const
             // Eigen::Vector2d& pp, double b) : _f(f), _pp(pp), _b(b) {} 166 167 inline double
             // focalLength() const {return _f;} 168 169 inline Eigen::Vector2d principalPoint()
             // const {return _pp;} 170 171 inline double baseline() const {return _b;}
        StereoMeasurement
        project(const Pose3d& pose, const Point3dh& Xw) const {
      174 Point3dh X = pose.transform_to(Xw);
      175  // camera system has z pointing forward, instead of x 176 double x = -X.y(); 177 double y
           // = -X.z(); 178 double z = X.x(); 179 180 // left camera 181 double fz = _f / z; 182
           // double u = x * fz + _pp(0); 183 double v = y * fz + _pp(1); 184 // right camera 185
           // double u2 = u -_b*fz; 186 bool valid = ( z > 0.0); // infront of camera? 187 188 if
           // (valid == false) { 189 std::cout << "invalid." << std::endl; 190 } 191 192 return
           // StereoMeasurement(u, v, u2, valid); 193 } 194 195 StereoMeasurement project(const
           // cameraPose3d& pose, const Point3dh& Xw) const { 196 Point3dh X =
           // pose.transform_to(Xw); 197 // camera system has z pointing forward, instead of x 198
           // double x = -X.y(); 199 double y = -X.z(); 200 double z = X.x(); 201 202 // left camera
           // 203 double fz = _f / z; 204 double u = x * fz + _pp(0); 205 double v = y * fz +
           // _pp(1); 206 // right camera 207 double u2 = u -_b*fz; 208 bool valid = ( z > 0.0); //
           // infront of camera? 209 if (valid == false) { 210 std::cout << "invalid." << std::endl;
           // 211 } 212 213 return StereoMeasurement(u, v, u2, valid); 214 }
          Point3dh
          backproject(const Pose3d& pose, const StereoMeasurement& measure) const {
        218 double disparity = measure.u - measure.u2;
        219 double lz = _f * _b / disparity;
        220 double lx = (measure.u - _pp(0)) * lz / _f;
        221 double ly = (measure.v - _pp(1)) * lz / _f;
        222 if (disparity < 0.) {
          223 std::cout
              << "Warning: StereoCameraDebug.backproject called with negative disparity\n";
          224
        }
        225 226 Point3dh X(lz, -lx, -ly, 1.0);
        227 228 return pose.transform_from(X);
        229
      }
      230 231 Point3dh backproject(const cameraPose3d& pose, const StereoMeasurement& measure)
          const {
        232 233 double disparity = measure.u - measure.u2;
        234 double lz = _f * _b / disparity;
        235 double lx = (measure.u - _pp(0)) * lz / _f;
        236 double ly = (measure.v - _pp(1)) * lz / _f;
        237 if (disparity < 0.) {
          238 std::cout
              << "Warning: StereoCameraDebug.backproject called with negative disparity\n";
          239
        }
        240 Point3dh X(lz, -lx, -ly, 1.0);
        241 return pose.transform_from(X);
        242
      }
      243 244
    };
    // stereo measurement factor with geometric frame reference 248 class
    // dStereo_MovingMap_CoM_Factor : public FactorT<StereoMeasurement> { 249
    // dynamicPose3d_NL_Node* _pose; 250 Point3d_Node* _point; 251 Point3dh_Node*
    // _point_h; 252 StereoCameraDebug* _camera; 253 cameraPose3d_Node* _camera_pose3d;
    // 254 kinematicPose3d_Node* _centerOfMass_princAxes; 255 256 Point3dh
    // predict_inertial_stored;
   public:
     constructor for projective geometry 261 dStereo_MovingMap_CoM_Factor(dynamicPose3d_NL_Node*
     pose, Point3dh_Node* point, StereoCameraDebug* camera, cameraPose3d_Node* camera_pose3d,
     kinematicPose3d_Node* centerOfMass_princAxes, 262 const StereoMeasurement& measure, const
     Noise& noise) 263 : FactorT<StereoMeasurement>("Stereo_Factor COM", 3, noise, measure), 264
     _pose(pose), _point(NULL), _point_h(point), _camera(camera), _camera_pose3d( camera_pose3d),
     _centerOfMass_princAxes(centerOfMass_princAxes) { 265 // StereoCameraDebug could also be a
     node later (either with 0 variables, 266 // or with calibration as variables) 267
     _nodes.resize(3); 268 _nodes[0] = pose; 269 _nodes[1] = _centerOfMass_princAxes; 270
     _nodes[2] = point; 271 272
     } 273 274 // constructor for Euclidean geometry 275 // WARNING:
     only use for points at short range 276 dStereo_MovingMap_CoM_Factor(dynamicPose3d_NL_Node*
     pose, Point3d_Node* point, StereoCameraDebug* camera, cameraPose3d_Node* camera_pose3d,
     kinematicPose3d_Node* centerOfMass_princAxes, 277 const StereoMeasurement& measure, const
     Noise& noise) 278 : FactorT<StereoMeasurement>("Stereo_Factor COM", 3, noise, measure), 279
     _pose(pose), _point(point), _point_h(NULL), _camera(camera), _camera_pose3d( camera_pose3d),
     _centerOfMass_princAxes(centerOfMass_princAxes) {
       280 _nodes.resize(3);
       281 _nodes[0] = pose;
       282 _nodes[1] = _centerOfMass_princAxes;
       283 _nodes[2] = point;
       284
     }
     285 286 void initialize() { 287 require(_pose->initialized(), "dynamic Stereo_Factor requires pose to be
     initialized"); 288 if(!_centerOfMass_princAxes->initialized()) { 289 kinematicPose3d
     com_pa_init; 290 _centerOfMass_princAxes->init(com_pa_init); 291
     }
     292 bool initialized = (_point_h != NULL) ? _point_h->initialized() : _point->initialized();
     293 if (!initialized) {
       294 Point3dh predict_inertial = _camera->backproject(_camera_pose3d->value(), _measure);
       predict_inertial_stored = predict_inertial;
       296 297 Point3dh predict_body = _pose->value().transform_to_body(predict_inertial);
       298 299 Point3dh predict_feature(_centerOfMass_princAxes->value().oTw() *
                                        predict_body.vector());
       300 301  // subtract Center of mass offset
           Vector3d vec_point_feat_frame =
               predict_feature.vector().head(3);  // _com_offset->value().vector(); 303 Point3dh
       point_com = Point3dh(Point3d(vec_point_feat_frame));
       304 305 if (_point_h != NULL) {
         306 _point_h->init(point_com);
         307
       }
       else {
         308 _point->init(point_com.to_point3d());
         309
       }
       310
     }
     311
  } 312 313 Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    314  // point in body feature frame 315
        Point3dh point = (_point_h != NULL) ? _point_h->value(s) : _point->value(s);
    316 317
        // add center of mass offset 318 Vector4d vec_point_feat_frame; 319
        vec_point_feat_frame
        << point.vector().head(3),
        1.0;
    320 Vector3d vec_point_com_frame =
        (_centerOfMass_princAxes->value(s).wTo() * vec_point_feat_frame).head(3);
    321 Point3dh point_com = Point3dh(Point3d(vec_point_com_frame));
    322 323  // transform from body frame to
        inertial frame 324 Point3dh inertialPoint =
            _pose->value(s).transform_to_inertial(point_com);
    325 326                              // project into camera
        327 StereoMeasurement predicted = _camera->project(_camera_pose3d->value(s), inertialPoint); 328 329 //create error measurement 330 if (_point_h!=NULL ||
                 predicted.valid == true) {
      331 return (predicted.vector() - _measure.vector());
      332
    }
    else {
      333 std::cout << "Warning -
                       dynamicStereo_MovingMap_Factor.basic_error : point behind camera,
          dropping term.\n "; 334 std::cout << "_camera_pose3d
                  ->value(s)
          : " <<
            _camera_pose3d->value(s)
              << std::endl;

      std::cout << "_pose->value(s): " << _pose->value(s) << std::endl;
      std::cout << "inertialPoint: " << inertialPoint << std::endl << std::endl;
      return Eigen::Vector3d::Zero();
    }
  }
};
point3d prior factor 344 class Point3d_Factor : public FactorT<Point3d> {
  345 Point3d_Node* _point;
  346 347 public : 348 349 Point3d_Factor(Point3d_Node* point,
                                          const Point3d& prior,
                                          const Noise& noise) 350
      : FactorT<Point3d>("Point3d_Factor", 3, noise, prior),
        _point(point) {
    351 _nodes.resize(1);
    352 _nodes[0] = point;
    353
  }
  354 355 void initialize() {
    356 if (!_point->initialized()) {
      357 Point3d predict = _measure;
      358 _point->init(predict);
      359
    }
    360
  }
  361 362 Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    363 return (_point->vector(s) - _measure.vector());
    364
  }
  365 366
};
367 368 369
}
370 371 #endif