/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
	rbdl_check_api_version (RBDL_API_VERSION);

	Model* model = new Model();

    if (!Addons::URDFReadFromFile(argv[1], model, false)) {
        std::cerr << "Error loading model " << argv[1] << std::endl;
		abort();
	}

    std::cout << "initializing variables... " << std::endl;
    double mass = 0;
    Vector3d CoM = Vector3d::Zero();
//    VectorNd Q = VectorNd::Zero (model->dof_count);
    VectorNd Q = VectorNd::Ones (model->dof_count) * 1.57;
    VectorNd QDot = VectorNd::Ones (model->dof_count) * 1.0;
    VectorNd QDDot = VectorNd::Ones (model->dof_count) * 2.0;
    VectorNd Tau = VectorNd::Zero (model->dof_count);
    MatrixNd M;
    Vector3d ref_point = Vector3d::Zero();
    MatrixNd J;
    std::cout << "Q: " << Q.transpose() << std::endl;
    std::cout << "QDot: " << QDot.transpose() << std::endl;
    std::cout << "QDDot: " << QDDot.transpose() << std::endl;

    std::cout << "computing... " << std::endl;
    // Fill robot mass
    RigidBodyDynamics::Utils::CalcCenterOfMass(*model, Q, QDot, mass, CoM, nullptr, nullptr, false);
    std::cout << "robot mass: " << mass << std::endl;
    std::cout << "robot CoM: " << CoM.transpose() << std::endl;

//    inverse dynamics
    InverseDynamics (*model, Q, QDot, QDDot, Tau, nullptr);
//    inertia matrix
    M.setZero(model->dof_count, model->dof_count);
    CompositeRigidBodyAlgorithm(*model, Q, M);
//    jacobian
    unsigned int id = model->GetBodyId("gripper_A");
    J.setZero(6, model->dof_count);
    CalcPointJacobian6D(*model, Q, id, ref_point, J);
//forward kinematics
    Matrix3d _tmp_matrix3d = RigidBodyDynamics::CalcBodyWorldOrientation(*model,
                                                                Q,
                                                                id);

    Vector3d _tmp_vector3d = RigidBodyDynamics::CalcBodyToBaseCoordinates(*model,
                                                                 Q,
                                                                 id,
                                                                 ref_point);

    _tmp_matrix3d.transposeInPlace();

    std::cout << "Tau (inv. dyn.): \n" << Tau.transpose() << std::endl;
    std::cout << "Inertia matrix: \n" << M << std::endl;
    std::cout << "Jacobian gripper_A: \n" << J << std::endl;
    std::cout << "forward kinematics gripper_A: \n" << _tmp_vector3d.transpose() << std::endl;
    std::cout << "forward kinematics orientation gripper_A: \n" << _tmp_matrix3d << std::endl;

	delete model;

 	return 0;
}

