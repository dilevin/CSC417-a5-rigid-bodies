#include <Eigen/Dense>
#include <EigenTypes.h>

//Input:
//  V - the nx3 matrix of vertices.
//  F - the mx3 matrix of triangle vertex indices.
//  density - the material density.
//Output:
//  I - the 3x3 angular inertia matrix
//  center - the center of mass of the object
//  mass - the total mass of the object

void inertia_matrix(Eigen::Matrix3d &I, Eigen::Vector3d & center, double &mass, Eigen::Ref<const Eigen::MatrixXd> V, Eigen::Ref<const Eigen::MatrixXi> F, double density);