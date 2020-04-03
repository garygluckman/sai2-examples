#ifndef _PANDA_H
#define _PANDA_H

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

#include <string>
#include <vector>
#include "keys.h"


void init_joint_task(Sai2Primitives::JointTask *joint_task, 
                     RedisClient& redis_client, int index, int read_id, 
                     int write_id)
{
    int dof = joint_task->_robot->dof();

    // initialize joint_task object
    joint_task->_kp = 50.0;
    joint_task->_kv = 14.0;
    joint_task->_desired_position = joint_task->_robot->_q;
    joint_task->_use_isotropic_gains = true;
    joint_task->_use_velocity_saturation_flag = false;
    joint_task->_use_interpolation_flag = false;
    joint_task->setDynamicDecouplingFull();
    
    // update values when we read all parameters on a new controller cycle
    redis_client.addDoubleToReadCallback(read_id, KP_JOINT_KEYS[index], joint_task->_kp);
    redis_client.addDoubleToReadCallback(read_id, KV_JOINT_KEYS[index], joint_task->_kv);
    redis_client.addEigenToReadCallback(read_id, DESIRED_JOINT_POS_KEYS[index], joint_task->_desired_position);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addDoubleToWriteCallback(write_id, KP_JOINT_KEYS[index], joint_task->_kp);
    redis_client.addDoubleToWriteCallback(write_id, KV_JOINT_KEYS[index], joint_task->_kv);
    redis_client.addEigenToWriteCallback(write_id, DESIRED_JOINT_POS_KEYS[index], joint_task->_desired_position);
}

////////////////// POSORI TASK VARIABLES //////////////////
std::array<int, N_ROBOTS> posori_use_interpolation;
std::array<double, N_ROBOTS> posori_interpolation_max_linear_velocity;
std::array<double, N_ROBOTS> posori_interpolation_max_linear_acceleration;
std::array<double, N_ROBOTS> posori_interpolation_max_linear_jerk;
std::array<double, N_ROBOTS> posori_interpolation_max_angular_velocity;
std::array<double, N_ROBOTS> posori_interpolation_max_angular_acceleration;
std::array<double, N_ROBOTS> posori_interpolation_max_angular_jerk;
std::array<int, N_ROBOTS> posori_use_velocity_saturation;
std::array<Eigen::Vector3d, N_ROBOTS> posori_euler_angles;
std::array<Eigen::Vector2d, N_ROBOTS> posori_velocity_saturation;
std::array<std::string, N_ROBOTS> posori_dynamic_decoupling_mode;

void init_posori_task(Sai2Primitives::PosOriTask *posori_task, 
                      RedisClient& redis_client, int index, int read_id, 
                      int write_id)
{
    Matrix3d initial_orientation;
    Vector3d initial_position;
    Vector3d initial_velocity;

    int dof = posori_task->_robot->dof();
    posori_task->_robot->rotation(initial_orientation, posori_task->_link_name);
    posori_task->_robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
    posori_task->_robot->linearVelocity(initial_velocity, posori_task->_link_name, posori_task->_control_frame.translation());

    // initialize global variables
    posori_use_interpolation[index] = 0;
    posori_use_velocity_saturation[index] = 0;
    posori_velocity_saturation[index] = M_PI / 3.0 * Vector2d::Ones();
    posori_dynamic_decoupling_mode[index] = "full";
    posori_interpolation_max_linear_velocity[index] = 0.4;
    posori_interpolation_max_linear_acceleration[index] = 1.0;
    posori_interpolation_max_linear_jerk[index] = 3.0;
    posori_interpolation_max_angular_velocity[index] = M_PI;
    posori_interpolation_max_angular_acceleration[index] = M_PI;
    posori_interpolation_max_angular_jerk[index] = 3 * M_PI;
    posori_velocity_saturation[index](0) = posori_task->_linear_saturation_velocity;
    posori_velocity_saturation[index](1) = posori_task->_angular_saturation_velocity;

    // we are doing ZYX, but we store XYZ
    posori_euler_angles[index] = initial_orientation.eulerAngles(2, 1, 0).reverse();

    // initialize posori_task object
    posori_task->_use_interpolation_flag = bool(posori_use_interpolation[index]);
    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation[index]);
    posori_task->_kp_pos = 200.0;
    posori_task->_kv_pos = 25.0;
    posori_task->_ki_pos = 0.0;
    posori_task->_kp_ori = 200.0;
    posori_task->_kv_ori = 20.0;
    posori_task->_ki_ori = 0.0;
    posori_task->_use_isotropic_gains_position = true;
    posori_task->_use_isotropic_gains_orientation = true;
    posori_task->setDynamicDecouplingFull();

	posori_task->_otg->setMaxLinearVelocity(posori_interpolation_max_linear_velocity[index]);
	posori_task->_otg->setMaxLinearAcceleration(posori_interpolation_max_linear_acceleration[index]);
	posori_task->_otg->setMaxLinearJerk(posori_interpolation_max_linear_jerk[index]);
	posori_task->_otg->setMaxAngularVelocity(posori_interpolation_max_angular_velocity[index]);
	posori_task->_otg->setMaxAngularAcceleration(posori_interpolation_max_angular_acceleration[index]);
	posori_task->_otg->setMaxAngularJerk(posori_interpolation_max_angular_jerk[index]);

    // prepare redis callback
    redis_client.addIntToReadCallback(read_id, POSORI_USE_INTERPOLATION_KEYS[index], posori_use_interpolation[index]);
    redis_client.addDoubleToReadCallback(read_id, POSORI_INTERPOLATION_MAX_LINEAR_VEL_KEYS[index], posori_interpolation_max_linear_velocity[index]);
    redis_client.addDoubleToReadCallback(read_id, POSORI_INTERPOLATION_MAX_LINEAR_ACCEL_KEYS[index], posori_interpolation_max_linear_acceleration[index]);
    redis_client.addDoubleToReadCallback(read_id, POSORI_INTERPOLATION_MAX_LINEAR_JERK_KEYS[index], posori_interpolation_max_linear_jerk[index]);
    redis_client.addDoubleToReadCallback(read_id, POSORI_INTERPOLATION_MAX_ANGULAR_VEL_KEYS[index], posori_interpolation_max_angular_velocity[index]);
    redis_client.addDoubleToReadCallback(read_id, POSORI_INTERPOLATION_MAX_ANGULAR_ACCEL_KEYS[index], posori_interpolation_max_angular_acceleration[index]);
    redis_client.addDoubleToReadCallback(read_id, POSORI_INTERPOLATION_MAX_ANGULAR_JERK_KEYS[index], posori_interpolation_max_angular_jerk[index]);
    redis_client.addIntToReadCallback(read_id, USE_VEL_SAT_POSORI_KEYS[index], posori_use_velocity_saturation[index]);
    redis_client.addEigenToReadCallback(read_id, VEL_SAT_POSORI_KEYS[index], posori_velocity_saturation[index]);
    redis_client.addDoubleToReadCallback(read_id, KP_POS_KEYS[index], posori_task->_kp_pos);
    redis_client.addDoubleToReadCallback(read_id, KV_POS_KEYS[index], posori_task->_kv_pos);
    redis_client.addDoubleToReadCallback(read_id, KI_POS_KEYS[index], posori_task->_ki_pos);
    redis_client.addDoubleToReadCallback(read_id, KP_ORI_KEYS[index], posori_task->_kp_ori);
    redis_client.addDoubleToReadCallback(read_id, KV_ORI_KEYS[index], posori_task->_kv_ori);
    redis_client.addDoubleToReadCallback(read_id, KI_ORI_KEYS[index], posori_task->_ki_ori);
    redis_client.addStringToReadCallback(read_id, DYN_DEC_POSORI_KEYS[index], posori_dynamic_decoupling_mode[index]);
    redis_client.addEigenToReadCallback(read_id, DESIRED_POS_KEYS[index], posori_task->_desired_position); 
    redis_client.addEigenToReadCallback(read_id, DESIRED_ORI_KEYS[index], posori_euler_angles[index]);
    redis_client.addEigenToReadCallback(read_id, DESIRED_VEL_KEYS[index], posori_task->_desired_velocity);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addIntToWriteCallback(write_id, POSORI_USE_INTERPOLATION_KEYS[index], posori_use_interpolation[index]);
    redis_client.addDoubleToWriteCallback(write_id, POSORI_INTERPOLATION_MAX_LINEAR_VEL_KEYS[index], posori_interpolation_max_linear_velocity[index]);
    redis_client.addDoubleToWriteCallback(write_id, POSORI_INTERPOLATION_MAX_LINEAR_ACCEL_KEYS[index], posori_interpolation_max_linear_acceleration[index]);
    redis_client.addDoubleToWriteCallback(write_id, POSORI_INTERPOLATION_MAX_LINEAR_JERK_KEYS[index], posori_interpolation_max_linear_jerk[index]);
    redis_client.addDoubleToWriteCallback(write_id, POSORI_INTERPOLATION_MAX_ANGULAR_VEL_KEYS[index], posori_interpolation_max_angular_velocity[index]);
    redis_client.addDoubleToWriteCallback(write_id, POSORI_INTERPOLATION_MAX_ANGULAR_ACCEL_KEYS[index], posori_interpolation_max_angular_acceleration[index]);
    redis_client.addDoubleToWriteCallback(write_id, POSORI_INTERPOLATION_MAX_ANGULAR_JERK_KEYS[index], posori_interpolation_max_angular_jerk[index]);
    redis_client.addIntToWriteCallback(write_id, USE_VEL_SAT_POSORI_KEYS[index], posori_use_velocity_saturation[index]);
    redis_client.addEigenToWriteCallback(write_id, VEL_SAT_POSORI_KEYS[index], posori_velocity_saturation[index]);
    redis_client.addDoubleToWriteCallback(write_id, KP_POS_KEYS[index], posori_task->_kp_pos);
    redis_client.addDoubleToWriteCallback(write_id, KV_POS_KEYS[index], posori_task->_kv_pos);
    redis_client.addDoubleToWriteCallback(write_id, KI_POS_KEYS[index], posori_task->_ki_pos);
    redis_client.addDoubleToWriteCallback(write_id, KP_ORI_KEYS[index], posori_task->_kp_ori);
    redis_client.addDoubleToWriteCallback(write_id, KV_ORI_KEYS[index], posori_task->_kv_ori);
    redis_client.addDoubleToWriteCallback(write_id, KI_ORI_KEYS[index], posori_task->_ki_ori);
    redis_client.addStringToWriteCallback(write_id, DYN_DEC_POSORI_KEYS[index], posori_dynamic_decoupling_mode[index]);
    redis_client.addEigenToWriteCallback(write_id, DESIRED_POS_KEYS[index], posori_task->_desired_position); 
    redis_client.addEigenToWriteCallback(write_id, DESIRED_ORI_KEYS[index], posori_euler_angles[index]);
    redis_client.addEigenToWriteCallback(write_id, DESIRED_VEL_KEYS[index], posori_task->_desired_velocity);
}

void update_posori_task(Sai2Primitives::PosOriTask *posori_task, int index)
{
    auto dof = posori_task->_robot->dof();

    if (posori_use_interpolation[index] && !posori_task->_use_interpolation_flag)
        posori_task->reInitializeTask();

    posori_task->_use_interpolation_flag = bool(posori_use_interpolation[index]);
    posori_task->_otg->setMaxLinearVelocity(posori_interpolation_max_linear_velocity[index]);
	posori_task->_otg->setMaxLinearAcceleration(posori_interpolation_max_linear_acceleration[index]);
	posori_task->_otg->setMaxLinearJerk(posori_interpolation_max_linear_jerk[index]);
	posori_task->_otg->setMaxAngularVelocity(posori_interpolation_max_angular_velocity[index]);
	posori_task->_otg->setMaxAngularAcceleration(posori_interpolation_max_angular_acceleration[index]);
	posori_task->_otg->setMaxAngularJerk(posori_interpolation_max_angular_jerk[index]);

    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation[index]);
    posori_task->_linear_saturation_velocity = posori_velocity_saturation[index](0);
    posori_task->_angular_saturation_velocity = posori_velocity_saturation[index](1);

    Matrix3d desired_rmat;
    desired_rmat = Eigen::AngleAxisd(posori_euler_angles[index](2), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(posori_euler_angles[index](1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(posori_euler_angles[index](0), Eigen::Vector3d::UnitX());
    posori_task->_desired_orientation = desired_rmat;

    if (posori_dynamic_decoupling_mode[index] == "full")
        posori_task->setDynamicDecouplingFull();
    else if (posori_dynamic_decoupling_mode[index] == "partial")
        posori_task->setDynamicDecouplingPartial();
    else if (posori_dynamic_decoupling_mode[index] == "inertia_saturation")
        posori_task->setDynamicDecouplingInertiaSaturation();
    else if (posori_dynamic_decoupling_mode[index] == "none")
        posori_task->setDynamicDecouplingNone();
}

#endif 
