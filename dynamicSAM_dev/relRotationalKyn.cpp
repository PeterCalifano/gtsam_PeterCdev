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
Eigen::Vector<T, 6> relRotationKinRHS(double tNow, Eigen::Vector<T, 6> xTranslState_G)
{

    // TEMP to avoid error
    Eigen::Vector<T, 3> angVelG; // Angular velocity of the G frame wrt Inertial I
    Eigen::Vector<T, 3> rAG_G;   // A CoM displacement frame G frame origin (from G to A) in G frame

    // Dor uses the rotation matrices and exponential map --> must deepen into this to understand how to implement it
};