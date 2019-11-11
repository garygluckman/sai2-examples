#ifndef _KEYS_H
#define _KEYS_H

#include <string>

const std::string CURRENT_EE_POS_KEY = "sai2::examples::current_ee_pos";
const std::string CURRENT_EE_VEL_KEY = "sai2::examples::current_ee_vel";

// controller initialization
const std::string CONTROL_STATE_KEY = "sai2::examples::control_state";
const std::string CONTROL_STATE_INITIALIZING = "initializing";
const std::string CONTROL_STATE_INITIALIZED = "initialized";
const std::string CONTROL_STATE_READY = "ready";

// posori task parameters
const std::string DESIRED_POS_KEY = "sai2::examples::desired_position";
const std::string DESIRED_ORI_KEY = "sai2::examples::desired_orientation";
const std::string DESIRED_VEL_KEY = "sai2::examples::desired_velocity";
const std::string KP_POS_KEY = "sai2::examples::kp_pos";
const std::string KV_POS_KEY = "sai2::examples::kv_pos";
const std::string KI_POS_KEY = "sai2::examples::ki_pos";
const std::string KP_ORI_KEY = "sai2::examples::kp_ori";
const std::string KV_ORI_KEY = "sai2::examples::kv_ori";
const std::string KI_ORI_KEY = "sai2::examples::ki_ori";
const std::string KP_NONISOTROPIC_POS_KEY = "sai2::examples::kp_nonisotropic_pos";
const std::string KV_NONISOTROPIC_POS_KEY = "sai2::examples::kv_nonisotropic_pos";
const std::string KI_NONISOTROPIC_POS_KEY = "sai2::examples::ki_nonisotropic_pos";
const std::string USE_ISOTROPIC_POS_GAINS_KEY = "sai2::examples::use_isotropic_pos_gains";
const std::string POSORI_USE_INTERPOLATION = "sai2::examples::posori_use_interpolation";
const std::string USE_VEL_SAT_POSORI_KEY = "sai2::examples::use_posori_velocity_saturation";
const std::string VEL_SAT_POSORI_KEY = "sai2::examples::posori_velocity_saturation";
const std::string DYN_DEC_POSORI_KEY = "sai2::examples::posori_dynamic_decoupling";

// joint task parameters
const std::string DESIRED_JOINT_POS_KEY = "sai2::examples::desired_joint_position";
const std::string KP_JOINT_KEY = "sai2::examples::kp_joint";
const std::string KV_JOINT_KEY = "sai2::examples::kv_joint";
const std::string KP_NON_ISOTROPIC_JOINT_KEY = "sai2::examples::kp_nonisotropic_joint";
const std::string KV_NON_ISOTROPIC_JOINT_KEY = "sai2::examples::kv_nonisotropic_joint";
const std::string USE_ISOTROPIC_JOINT_GAINS_KEY = "sai2::examples::use_isotropic_joint_gains";
const std::string JOINT_USE_INTERPOLATION = "sai2::examples::joint_use_interpolation";
const std::string USE_VEL_SAT_JOINT_KEY = "sai2::examples::use_joint_velocity_saturation";
const std::string VEL_SAT_JOINT_KEY = "sai2::examples::joint_velocity_saturation";
const std::string DYN_DEC_JOINT_KEY = "sai2::examples::joint_dynamic_decoupling";

// robot file
const std::string ROBOT_FILE = "resources/panda_arm.urdf";

// state-related keys
const std::string PRIMITIVE_KEY = "sai2::examples::primitive";
const std::string PRIMITIVE_JOINT_TASK = "primitive_joint_task";
const std::string PRIMITIVE_POSORI_TASK = "primitive_posori_task";
const std::string PRIMITIVE_TRAJECTORY_TASK = "primitive_trajectory_task";
const std::string PRIMITIVE_FLOATING_TASK = "primitive_floating_task";

#endif 