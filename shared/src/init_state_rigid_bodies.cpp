#include <init_state_rigid_bodies.h>

void init_state_rigid_bodies(Eigen::VectorXd &q, Eigen::VectorXd &qdot, unsigned int num_rigid_bodies) {

    q.resize(12*num_rigid_bodies);
    qdot.resize(6*num_rigid_bodies);

    q.setZero();
    qdot.setZero();

    //set rotation matrices to identity
    for(unsigned int ii=0;ii<num_rigid_bodies;++ii){ 
        q(12*ii) = 1.;
        q(12*ii+4) = 1.;
        q(12*ii+8) = 1.;
    }
}