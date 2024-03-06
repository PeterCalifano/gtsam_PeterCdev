#pragma once 2 3 #include < cmath> 4 #include < Eigen / Dense> 5 #include <
Eigen / Geometry > 6 #include < isam / Node.h > 7 #include < isam / Factor.h > 8 #include
    "NodeExmap.h" 9 10 namespace isam {
  11 12 class inertiaRatios {
    13 frend std::ostream& operator<<(std::ostream& out, const inertiaRatios& p) {
      14 p.write(out);
      15 return out;
      16
    }
    17 18 /* 19 * k1 = ln(J11 / J22) 20
           * k2 = ln(J22 / J33) 21 */
        22 double _k1;
    23 double _k2;
    24 25 public : 26 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 27 28 static const int dim = 2;
    29 static const char* name() {
      30 return "inertiaRatios";
      31
    }
    32 33 inertiaRatios() {
      34 _k1 = 0;
      35 _k2 = 0;
      36
    }
    37 38 39 inertiaRatios(const double& k1, const double& k2) {
      40 _k1 = k1;
      41 _k2 = k2;
      42
    }
    43 44 Eigen::Matrix3d getJ() const {
      Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
      46 double Jscale =
          1.0;  // 0.0116; 47 J(0,0) = exp(_k1); 48 J(1,1) = 1.0; 49 J(2,2) = exp(-_k2); 50 51 J *=
                // Jscale; 52 53 return J; 54 } 55 56 Eigen::VectorXd x() const{ 57 Eigen::Vector2d
                // x; 58 x(0) = _k1; 59 x(1) = _k2; 60 return x; 61 } 62 63 void
                // setState(Eigen::VectorXd x) { 64 _k1 = x(0); 65 _k2 = x(1); 66 } 67 68
                // inertiaRatios exmap(const Eigen::Vector2d& ∆) { 69 inertiaRatios res = *this; 70
                // res._k1 += ∆(0); 71 res._k2 += ∆(1); 72 return res; 73 } 74 75 inertiaRatios
                // exmap_reset(const Eigen::Vector2d& ∆) { 76 inertiaRatios res = *this; 77 res._k1
                // += ∆(0); 78 res._k2 += ∆(1); 79 return res; 80 } 81 82 Eigen::VectorXd vector()
                // const { 83 Eigen::Vector2d tmp; 84 tmp << _k1, _k2; 85 return tmp; 86 } 87 88
                // void set(const Eigen::Vector2d& v) { 89 _k1 = v(0);
      _k2 = v(1);
      91
    }
    92 93 void write(std::ostream& out) const {
      94 Eigen::Matrix3d Jcurr = getJ();
      95 out << std::endl
             << "inertaRatios x: " << x().transpose() << std::endl
             << Jcurr(0, 0) << " , " << Jcurr(1, 1) << " , " << Jcurr(2, 2) << std::endl;
      96
    }
    97 98
  };
  99 100 typedef NodeExmapT<inertiaRatios> inertiaRatios_Node;
  101 102 /** 103 * Prior on inertiaRatios. 104 */ 105 class inertiaRatios_Factor
      : public FactorT<inertiaRatios> {
    106 public : 107 inertiaRatios_Node* _ir_node;
    108 109 inertiaRatios_Factor(inertiaRatios_Node* ir_node,
                                 const inertiaRatios& prior,
                                 const Noise& noise) 110
        : FactorT<inertiaRatios>("inertiaRatios_Factor", 2, noise, prior),
          _ir_node(ir_node) {
      111 _nodes.resize(1);
      112 _nodes[0] = ir_node;
      113
    }
    114 115 void initialize() {
      116 if (!_ir_node->initialized()) {
        117 inertiaRatios predict = _measure;
        118 _ir_node->init(predict);
        119
      }
      120
    }
    121 122 Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
      123 inertiaRatios ir = _ir_node->value(s);
      124 Eigen::VectorXd err = ir.vector() - _measure.vector();
      125 return err;
      126
    }
    127
  };
  128
}