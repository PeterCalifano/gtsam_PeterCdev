#include <iostream>
#include <cmath>
#include <Eigen/Dense>

/*
RHS of the translational (r,v) dynamics of a point mass with respect to a non-inertial G frame fixed to a rotating rigid-body
with offset with respect to its centre of mass A.
NOTE: this may be converted to a class, to allow for use with two options: A!=G and A=G
Coded by PeterC - DD/03/2024
*/
template <class T>
Eigen::Vector<T, 6> relTranslDynRHS(double tNow, Eigen::Vector<T, 6> xTranslState_G)
{
    // Define local alias
    using Vector6T = Eigen::Vector<T, 6>;
    using posIdx = Eigen::seq(0, 2);
    using velIdx = Eigen::seq(3, 5);

    // TEMP to avoid error
    Eigen::Vector<T, 3> angVelG; // Angular velocity of the G frame wrt Inertial I
    Eigen::Vector<T, 3> rAG_G;   // A CoM displacement frame G frame origin (from G to A) in G frame

    Vector6T dXdt_G = Vector6T::Zero(); // Where X := [r,v] in G frame

    // Evaluate position derivative
    // Check which is the correct operation: it may not be just a cross prodcut

    dXdt_G(posIdx) = xTranslState_G(velIdx) - angVelG.cross(xTranslState_G(posIdx));

    // Evaluate velocity derivative --> requires acceleration model
    dXdt_G(velIdx);
};

/*
Inertial relative acceleration model: TODO
Coded by PeterC - DD/03/2024
*/

template <class T>
Eigen::Vector<T, 3> inertialAccelModel(double tNow, Eigen::Vector<T, 6> xTranslState_G)
{
    // TEMP to avoid error
    Eigen::Vector<T, 3> angVelG; // Angular velocity of the G frame wrt Inertial I
    Eigen::Vector<T, 3> rAG_G;   // A CoM displacement frame G frame origin (from G to A) in G frame

    // Define local aliases
    using posIdx = Eigen::seq(0, 2);
    using velIdx = Eigen::seq(3, 5);
}

/*
Inertial relative acceleration model: TODO
Coded by PeterC - DD/03/2024
*/

template <class T>
Eigen::Vector<T, 3> nonInertialAccel(double tNow, Eigen::Vector<T, 6> xTranslState_G)
{
    // Define local aliases
    using posIdx = Eigen::seq(0, 2);
    using velIdx = Eigen::seq(3, 5);

    // TEMP to avoid error
    Eigen::Vector<T, 3> angVelG; // Angular velocity of the G frame wrt Inertial I
    Eigen::Vector<T, 3> rAG_G;   // A CoM displacement frame G frame origin (from G to A) in G frame

    // angVelG replace by xRotationState_GI ?
    // using angVelIdx = Eigen::seq(4, 6); assuming [7x1 vector]

    Eigen::Vector<T, 3> coriolisAccel, offsetAccel;

    // Coriolis-like term
    coriolisAccel = -angVelG.cross(xTranslState_G(velIdx));

    // Additional non-inertial term due to offset of reference frame from centre of mass of the body
    offsetAccel;

    return coriolisAccel + offsetAccel;
}

template <class T>
Eigen::Vector<T, 3> computeBodyAngAccel(Eigen::Matrix<T, 3, 3> inertiaMat, Eigen::Vector<T, 3> extTorque, Eigen::Vector<T, 3> angVelG)
{
    return;
}