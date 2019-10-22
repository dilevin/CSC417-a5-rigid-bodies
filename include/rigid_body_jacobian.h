#include <Eigen/Dense>
#include <EigenTypes.h>

//Input:
//  R - rotation matrix for rigid body
//  p - world space position of center-of-mass
//  X -  undeformed position at which to compute the Jacobian. 
//Output:
//  J - the rigid body jacobian acting on the undeformed space point X.

void rigid_body_jacobian(Eigen::Matrix36d &J, 
                         Eigen::Ref<const Eigen::Matrix3d> R, Eigen::Ref<const Eigen::Vector3d> p, 
                         Eigen::Ref<const Eigen::Vector3d> X);
