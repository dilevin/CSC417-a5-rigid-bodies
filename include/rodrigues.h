#include <Eigen/Dense>
#include <EigenTypes.h>

//  R - rotation matrix 
//  omega - angular velocity vector
void rodrigues(Eigen::Matrix3d &R, Eigen::Ref<const Eigen::Vector3d> omega);