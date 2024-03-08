#ifndef SLAMDYNAMICS_H_
#define SLAMDYNAMICS_H_
#include <Eigen/Dense >
#include <isam/Factor.h >
#include <isam/Node.h >
#include <isam/Point3d.h >
#include <isam/Pose3d.h >
#include <isam/slam_stereo.h >
#include "FactorVariableNoise.h"
#include "NodeExmap.h"
#include "camera3d.h"
#include "dynamicPose3d_NL.h"
#include "inertiaRatios.h"
#include "kinematicPose3d.h"

namespace isam {
typedef NodeExmapT<dynamicPose3d_NL> dynamicPose3d_NL_Node;
typedef NodeT<Point3d> Point3d_Node;
/* Prior on dynamicPose3d. */
class dynamicPose3d_NL_Factor : public FactorT<dynamicPose3d_NL> {
 public:
  dynamicPose3d_NL_Node* _pose;
  dynamicPose3d_NL_Factor(dynamicPose3d_NL_Node* pose,
                          const dynamicPose3d_NL& prior,
                          const Noise& noise)
      : FactorT<dynamicPose3d_NL>("dynamicPose3d_NL_Factor", 12, noise, prior), _pose(pose) {
    _nodes.resize(1);
    _nodes[0] = pose;
  }
  void initialize() {
    if (!_pose->initialized()) {
      dynamicPose3d_NL predict = _measure;
      _pose->init(predict);
    }
    Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
      dynamicPose3d_NL p1 = _pose->value(s);
      Eigen::VectorXd err = p1.vectorFull() - _measure.vector();
      Eigen::Vector4d p1qTot = p1.qTotal();
      Eigen::Vector4d mqTot = _measure.qTotal();
      Vector3d da = p1.getMRPdifference(p1qTot, mqTot);
      err.segment<3>(6) = da;
      return err;
    }
  };
  // Process Model Factor - one of the main contributions of this thesis
  class dynamicPose3d_NL_dynamicPose3d_NL_Factor : public FactorVarNoiseT<dynamicPose3d_NL> {
    dynamicPose3d_NL_Node* _pose1;
    dynamicPose3d_NL_Node* _pose2;
    inertiaRatios_Node* _ir_node;
    double dt;

   public:
    /*
   Constructor.  * @param pose1 The pose from which the measurement starts.  *
   // @param pose2 The pose to which the measurement extends.  * @param measure DOES
   // NOTHING - DON'T USE IT!!!! (could be extended in future release to add
   // forces/torques  * @param noise The 12x12 square root information matrix (upper
   // triangular).  */
    dynamicPose3d_NL_dynamicPose3d_NL_Factor(dynamicPose3d_NL_Node* pose1,
                                             dynamicPose3d_NL_Node* pose2,
                                             inertiaRatios_Node* ir_node,
                                             const dynamicPose3d_NL& measure,
                                             const Noise& noise,
                                             double timestep)
        : FactorVarNoiseT<dynamicPose3d_NL>("dp3dNL_dp3dNL_IR_Factor", 12, noise, measure),
          _pose1(pose1),
          _pose2(pose2),
          _ir_node(ir_node),
          dt(timestep) {
      _nodes.resize(3);
      _nodes[0] = pose1;
      _nodes[1] = pose2;
      _nodes[2] = ir_node;
    }
    void initialize() {
      dynamicPose3d_NL_Node* pose1 = _pose1;
      dynamicPose3d_NL_Node* pose2 = _pose2;
      inertiaRatios_Node* ir_node = _ir_node;
      require(pose1->initialized() || pose2->initialized(),
              "dynamicSLAM: dynamicPose3d_NL_dynamicPose3d_NL_Factor requires pose1 or pose2 to "
              "be initialized");
      if (!_ir_node->initialized()) {
        inertiaRatios init_ir;
        _ir_node->init(init_ir);
      }
      if (!pose1->initialized() && pose2->initialized()) {
        std::cout << "No BACKWARDS PROPAGATE" << std::endl;

      } else if (pose1->initialized() && !pose2->initialized()) {
        inertiaRatios ir = _ir_node->value();
        dynamicPose3d_NL a = pose1->value();
        dynamicPose3d_NL predict = a.propagate(dt, ir);
        pose2->init(predict);
      }
    }
    Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
      dynamicPose3d_NL p1 = _pose1->value(s);
      dynamicPose3d_NL p2 = _pose2->value(s);
      inertiaRatios ir = _ir_node->value(s);
      Eigen::VectorXd err = p2.computeStateChange(p1, dt, ir);
      return err;
    }
    Eigen::MatrixXd get_sqrtinf() const {
      inertiaRatios ir = _ir_node->value();
      Eigen::MatrixXd new_sqrtinf = _pose1->value().getProcessNoise(dt, ir)._sqrtinf;
      return new_sqrtinf;
    }
    bool checkPose1(dynamicPose3d_NL_Node* poseRef) {
      if (_pose1 == poseRef) {
        return true;
      } else {
        return false;
      }
    }
    bool checkPose1(inertiaRatios_Node* ir_node) {
      if (_ir_node == ir_node) {
        return true;

      } else {
        return false;
      }
    }
    bool checkPose1(kinematicPose3d_Node* poseRef) { return false; }
    double get_dt() { return dt; }
    void write(std::ostream& out) const { FactorVarNoiseT<dynamicPose3d_NL>::write(out); }
  };
  typedef NodeT<Point3dh> Point3dh_Node;
  stereo camera class class StereoCameraDebug {  // for now, camera and robot are
    identical double _f;
    Eigen::Vector2d _pp;
    double _b;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW StereoCameraDebug()
        : _f(1), _pp(Eigen::Vector2d(0.5, 0.5)), _b(0.1) {}
    StereoCameraDebug(double f, const Eigen::Vector2d& pp, double b) : _f(f), _pp(pp), _b(b) {}
    inline double focalLength() const { return _f; }
    inline Eigen::Vector2d principalPoint() const { return _pp; }
    inline double baseline() const { return _b; }
    StereoMeasurement project(const Pose3d& pose, const Point3dh& Xw) const {
      Point3dh X = pose.transform_to(Xw);
      camera system has z pointing forward, instead of x double x = -X.y();
      double y = -X.z();
      double z = X.x();  // left camera  double fz = _f / z;
      double u = x * fz + _pp(0);
      double v = y * fz + _pp(1);  // right camera
      double u2 = u - _b * fz;
      bool valid = (z > 0.0);  // infront of camera?  if
      (valid == false) { std::cout << "invalid." << std::endl; }
      return StereoMeasurement(u, v, u2, valid);
    }
    StereoMeasurement project(const cameraPose3d& pose, const Point3dh& Xw) const {
      Point3dh X = pose.transform_to(Xw);  // camera system has z pointing forward, instead of x
      double x = -X.y();
      double y = -X.z();
      double z = X.x();  // left camera
      double fz = _f / z;
      double u = x * fz + _pp(0);
      double v =
          y * fz + _pp(1);  // right camera  double u2 = u -_b*fz;  bool valid = ( z > 0.0); //
      infront of camera ? if (valid == false) { std::cout << "invalid." << std::endl; }
      return StereoMeasurement(u, v, u2, valid);
    }
    Point3dh backproject(const Pose3d& pose, const StereoMeasurement& measure) const {
      double disparity = measure.u - measure.u2;
      double lz = _f * _b / disparity;
      double lx = (measure.u - _pp(0)) * lz / _f;
      double ly = (measure.v - _pp(1)) * lz / _f;
      if (disparity < 0.) {
        std::cout << "Warning: StereoCameraDebug.backproject called with negative disparity\n";
      }
      Point3dh X(lz, -lx, -ly, 1.0);
      return pose.transform_from(X);
    }
    Point3dh backproject(const cameraPose3d& pose, const StereoMeasurement& measure) const {
      double disparity = measure.u - measure.u2;
      double lz = _f * _b / disparity;
      double lx = (measure.u - _pp(0)) * lz / _f;
      double ly = (measure.v - _pp(1)) * lz / _f;
      if (disparity < 0.) {
        std::cout << "Warning: StereoCameraDebug.backproject called with negative disparity\n";
      }
      Point3dh X(lz, -lx, -ly, 1.0);
      return pose.transform_from(X);
    }
  };
  // stereo measurement factor with geometric frame reference
  class dStereo_MovingMap_CoM_Factor : public FactorT<StereoMeasurement> {
    dynamicPose3d_NL_Node* _pose;
    Point3d_Node* _point;
    Point3dh_Node* _point_h;
    StereoCameraDebug* _camera;
    cameraPose3d_Node* _camera_pose3d;
    kinematicPose3d_Node* _centerOfMass_princAxes;
    Point3dh predict_inertial_stored;

   public:
    // constructor for projective geometry
    dStereo_MovingMap_CoM_Factor(
        dynamicPose3d_NL_Node* pose,
        Point3dh_Node* point,
        StereoCameraDebug* camera,
        cameraPose3d_Node* camera_pose3d,
        kinematicPose3d_Node* centerOfMass_princAxes,
        const StereoMeasurement& measure,
        const Noise& noise)::FactorT<StereoMeasurement>("Stereo_Factor COM", 3, noise, measure),
        _pose(pose), _point(NULL), _point_h(point), _camera(camera), _camera_pose3d(camera_pose3d),
        _centerOfMass_princAxes(centerOfMass_princAxes) {  // StereoCameraDebug could also be a
      // node later (either with 0 variables,  // or with calibration as variables)
      _nodes.resize(3);
      _nodes[0] = pose;
      _nodes[1] = _centerOfMass_princAxes;
      _nodes[2] = point;
    }  // constructor for Euclidean geometry  // WARNING:
       // only use for points at short range
    dStereo_MovingMap_CoM_Factor(dynamicPose3d_NL_Node* pose,
                                 Point3d_Node* point,
                                 StereoCameraDebug* camera,
                                 cameraPose3d_Node* camera_pose3d,
                                 kinematicPose3d_Node* centerOfMass_princAxes,
                                 const StereoMeasurement& measure,
                                 const Noise& noise)
        : FactorT<StereoMeasurement>("Stereo_Factor COM", 3, noise, measure),
          _pose(pose),
          _point(point),
          _point_h(NULL),
          _camera(camera),
          _camera_pose3d(camera_pose3d),
          _centerOfMass_princAxes(centerOfMass_princAxes) {
      _nodes.resize(3);
      _nodes[0] = pose;
      _nodes[1] = _centerOfMass_princAxes;
      _nodes[2] = point;
    }
    void initialize() {  require(_pose->initialized(), "dynamic Stereo_Factor requires pose to be
     initialized");  if(!_centerOfMass_princAxes->initialized()) {  kinematicPose3d
     com_pa_init;  _centerOfMass_princAxes->init(com_pa_init);
    }
    bool initialized = (_point_h != NULL) ? _point_h->initialized() : _point->initialized();
    if (!initialized) {
      Point3dh predict_inertial = _camera->backproject(_camera_pose3d->value(), _measure);
      predict_inertial_stored = predict_inertial;
      Point3dh predict_body = _pose->value().transform_to_body(predict_inertial);
      Point3dh predict_feature(_centerOfMass_princAxes->value().oTw() * predict_body.vector());
      // subtract Center of mass offset
      Vector3d vec_point_feat_frame =
          predict_feature.vector().head(3);  // _com_offset->value().vector();
      Point3dh point_com = Point3dh(Point3d(vec_point_feat_frame));
      if (_point_h != NULL) {
        _point_h->init(point_com);

      } else {
        _point->init(point_com.to_point3d());
      }
    }

  } Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    // point in body feature frame
    Point3dh point = (_point_h != NULL) ? _point_h->value(s) : _point->value(s);

    // add center of mass offset
    Vector4d vec_point_feat_frame;
    vec_point_feat_frame << point.vector().head(3), 1.0;
    Vector3d vec_point_com_frame =
        (_centerOfMass_princAxes->value(s).wTo() * vec_point_feat_frame).head(3);
    Point3dh point_com = Point3dh(Point3d(vec_point_com_frame));
    // transform from body frame to
    inertial frame Point3dh inertialPoint = _pose->value(s).transform_to_inertial(point_com);
    // project into camera
    StereoMeasurement predicted =
        _camera->project(_camera_pose3d->value(s), inertialPoint);  // create error measurement
    if (_point_h != NULL || predicted.valid == true) {
      return (predicted.vector() - _measure.vector());

    } else {
      std::cout << "Warning -
                   dynamicStereo_MovingMap_Factor.basic_error : point behind camera,
          dropping term.\n ";  std::cout << "_camera_pose3d
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
point3d prior factor class Point3d_Factor : public FactorT<Point3d> {
  Point3d_Node* _point;

 public:
  Point3d_Factor(Point3d_Node* point, const Point3d& prior, const Noise& noise)
      : FactorT<Point3d>("Point3d_Factor", 3, noise, prior), _point(point) {
    _nodes.resize(1);
    _nodes[0] = point;
  }
  void initialize() {
    if (!_point->initialized()) {
      Point3d predict = _measure;
      _point->init(predict);
    }
  }
  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    return (_point->vector(s) - _measure.vector());
  }
};
}  // namespace isam
#endif