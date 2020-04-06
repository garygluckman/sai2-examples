#ifndef _KEYS_H
#define _KEYS_H

#include <string>
#include <array>

constexpr int N_ROBOTS = 2;

// simulation keys
constexpr std::array<const char *, N_ROBOTS> SIM_JOINT_ANGLES_KEYS {
    "sai2::examples::panda1::sensors::q",
    "sai2::examples::panda2::sensors::q",
};

constexpr std::array<const char *, N_ROBOTS> SIM_JOINT_VELOCITIES_KEYS {
    "sai2::examples::panda1::sensors::dq",
    "sai2::examples::panda2::sensors::dq",
};

constexpr std::array<const char *, N_ROBOTS> SIM_JOINT_TORQUES_COMMANDED_KEYS {
    "sai2::examples::panda1::actuators::fgc",
    "sai2::examples::panda2::actuators::fgc"
};

constexpr std::array<const char *, N_ROBOTS> SIM_SENSED_FORCES_KEYS {
	"sai2::examples::panda1::sensors::wrist_force_moment",
	"sai2::examples::panda2::sensors::wrist_force_moment",
};

// used when on real hardware
constexpr std::array<const char *, N_ROBOTS> HW_JOINT_ANGLES_KEYS {
    "sai2::FrankaPanda::Bonnie::sensors::q",
    "sai2::FrankaPanda::Clyde::sensors::q"
};

constexpr std::array<const char *, N_ROBOTS> HW_JOINT_VELOCITIES_KEYS {
    "sai2::FrankaPanda::Bonnie::sensors::dq",
    "sai2::FrankaPanda::Clyde::sensors::dq"
};

constexpr std::array<const char *, N_ROBOTS> HW_JOINT_TORQUES_COMMANDED_KEYS {
    "sai2::FrankaPanda::Bonnie::actuators::fgc",
    "sai2::FrankaPanda::Clyde::actuators::fgc"
};

constexpr std::array<const char *, N_ROBOTS> MASSMATRIX_KEYS {
    "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix",
    "sai2::FrankaPanda::Clyde::sensors::model::massmatrix"
};

constexpr std::array<const char *, N_ROBOTS> ROBOT_GRAVITY_KEYS {
    "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity",
    "sai2::FrankaPanda::Clyde::sensors::model::robot_gravity"
};

constexpr std::array<const char *, N_ROBOTS> CORIOLIS_KEYS {
    "sai2::FrankaPanda::Bonnie::sensors::model::coriolis",
    "sai2::FrankaPanda::Clyde::sensors::model::coriolis"
};

#if false
// XXX: need to determine correct keys for sensed forces
constexpr std::array<const char *, N_ROBOTS> HW_SENSED_FORCES_KEYS {
    "sai2::FrankaPanda::Bonnie::sensors::model::wrist_force_moment",
    "sai2::FrankaPanda::Clyde::sensors::model::wrist_force_moment"
};
#endif

// useful logging variables
constexpr std::array<const char *, N_ROBOTS> CURRENT_EE_POS_KEYS {
    "sai2::examples::panda1::current_ee_pos",
    "sai2::examples::panda2::current_ee_pos"
};

constexpr std::array<const char *, N_ROBOTS> CURRENT_EE_VEL_KEYS {
    "sai2::examples::panda1::current_ee_vel",
    "sai2::examples::panda2::current_ee_vel"
};

// controller initialization
constexpr const char *CONTROL_STATE_KEY = "sai2::examples::control_state";
constexpr const char *CONTROL_STATE_INITIALIZING = "initializing";
constexpr const char *CONTROL_STATE_INITIALIZED = "initialized";
constexpr const char *CONTROL_STATE_READY = "ready";

// posori task parameters
constexpr std::array<const char *, N_ROBOTS> DESIRED_POS_KEYS {
    "sai2::examples::panda1::desired_position",
    "sai2::examples::panda2::desired_position"
};

constexpr std::array<const char *, N_ROBOTS> DESIRED_ORI_KEYS {
    "sai2::examples::panda1::desired_orientation",
    "sai2::examples::panda2::desired_orientation"
};

constexpr std::array<const char *, N_ROBOTS> DESIRED_VEL_KEYS {
    "sai2::examples::panda1::desired_velocity",
    "sai2::examples::panda2::desired_velocity"
};

constexpr std::array<const char *, N_ROBOTS> KP_POS_KEYS {
    "sai2::examples::panda1::kp_pos",
    "sai2::examples::panda2::kp_pos"
};

constexpr std::array<const char *, N_ROBOTS> KV_POS_KEYS {
    "sai2::examples::panda1::kv_pos",
    "sai2::examples::panda2::kv_pos"
};

constexpr std::array<const char *, N_ROBOTS> KI_POS_KEYS {
    "sai2::examples::panda1::ki_pos",
    "sai2::examples::panda2::ki_pos"
};

constexpr std::array<const char *, N_ROBOTS> KP_ORI_KEYS {
    "sai2::examples::panda1::kp_ori",
    "sai2::examples::panda2::kp_ori"
};

constexpr std::array<const char *, N_ROBOTS> KV_ORI_KEYS {
    "sai2::examples::panda1::kv_ori",
    "sai2::examples::panda2::kv_ori"
};

constexpr std::array<const char *, N_ROBOTS> KI_ORI_KEYS {
    "sai2::examples::panda1::ki_ori",
    "sai2::examples::panda2::ki_ori"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_USE_INTERPOLATION_KEYS = {
    "sai2::examples::panda1::posori_use_interpolation",
    "sai2::examples::panda2::posori_use_interpolation"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_INTERPOLATION_MAX_LINEAR_VEL_KEYS {
    "sai2::examples::panda1::posori_interpolation_max_linear_vel",
    "sai2::examples::panda2::posori_interpolation_max_linear_vel"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_INTERPOLATION_MAX_LINEAR_ACCEL_KEYS {
    "sai2::examples::panda1::posori_interpolation_max_linear_accel",
    "sai2::examples::panda2::posori_interpolation_max_linear_accel"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_INTERPOLATION_MAX_LINEAR_JERK_KEYS {
    "sai2::examples::panda1::posori_interpolation_max_linear_jerk",
    "sai2::examples::panda2::posori_interpolation_max_linear_jerk"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_INTERPOLATION_MAX_ANGULAR_VEL_KEYS {
    "sai2::examples::panda1::posori_interpolation_max_angular_vel",
    "sai2::examples::panda2::posori_interpolation_max_angular_vel"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_INTERPOLATION_MAX_ANGULAR_ACCEL_KEYS {
    "sai2::examples::panda1::posori_interpolation_max_angular_accel",
    "sai2::examples::panda2::posori_interpolation_max_angular_accel"
};

constexpr std::array<const char *, N_ROBOTS> POSORI_INTERPOLATION_MAX_ANGULAR_JERK_KEYS {
    "sai2::examples::panda1::posori_interpolation_max_angular_jerk",
    "sai2::examples::panda2::posori_interpolation_max_angular_jerk"
};

constexpr std::array<const char *, N_ROBOTS> USE_VEL_SAT_POSORI_KEYS = {
    "sai2::examples::panda1::use_posori_velocity_saturation",
    "sai2::examples::panda2::use_posori_velocity_saturation"
};

constexpr std::array<const char *, N_ROBOTS> VEL_SAT_POSORI_KEYS = {
    "sai2::examples::panda1::posori_velocity_saturation",
    "sai2::examples::panda2::posori_velocity_saturation"
};

constexpr std::array<const char *, N_ROBOTS> DYN_DEC_POSORI_KEYS = {
    "sai2::examples::panda1::posori_dynamic_decoupling",
    "sai2::examples::panda2::posori_dynamic_decoupling"
};

// joint task parameters
constexpr std::array<const char *, N_ROBOTS> DESIRED_JOINT_POS_KEYS = {
    "sai2::examples::panda1::desired_joint_position",
    "sai2::examples::panda2::desired_joint_position"
};

constexpr std::array<const char *, N_ROBOTS> KP_JOINT_KEYS = {
    "sai2::examples::panda1::kp_joint",
    "sai2::examples::panda2::kp_joint"
};

constexpr std::array<const char *, N_ROBOTS> KV_JOINT_KEYS = {
    "sai2::examples::panda1::kv_joint",
    "sai2::examples::panda2::kv_joint"
};

// robot file
constexpr std::array<const char *, N_ROBOTS> ROBOT_FILES = {
    "./resources/panda_arm_flat_ee.urdf",
    "./resources/panda_arm_flat_ee.urdf"
};

constexpr std::array<const char *, N_ROBOTS> ROBOT_NAMES = {
    "PANDA1",
    "PANDA2",
};

// two-hand related keys
constexpr const char *TWO_HAND_USE_INTERNAL_FORCE_KEY = "sai2::examples::use_internal_force";

constexpr const char *DESIRED_INTERNAL_SEPARATION_KEY = "sai2::examples::two_hand_desired_internal_separation";
constexpr const char *DESIRED_INTERNAL_ANGLES_KEY = "sai2::examples::two_hand_desired_internal_angles";

constexpr const char *DESIRED_INTERNAL_TENSION_KEY = "sai2::examples::two_hand_desired_internal_tension";
constexpr const char *DESIRED_OBJECT_POSITION_KEY = "sai2::examples::two_hand_desired_object_position";
constexpr const char *DESIRED_OBJECT_ORIENTATION_KEY = "sai2::examples::two_hand_desired_object_orientation";

constexpr const char *TWO_HAND_KP_POS_KEY = "sai2::examples::two_hand_kp_pos";
constexpr const char *TWO_HAND_KV_POS_KEY = "sai2::examples::two_hand_kv_pos";
constexpr const char *TWO_HAND_KI_POS_KEY = "sai2::examples::two_hand_ki_pos";

constexpr const char *TWO_HAND_KP_ORI_KEY = "sai2::examples::two_hand_kp_ori";
constexpr const char *TWO_HAND_KV_ORI_KEY = "sai2::examples::two_hand_kv_ori";
constexpr const char *TWO_HAND_KI_ORI_KEY = "sai2::examples::two_hand_ki_ori";

constexpr const char *TWO_HAND_KP_FORCE_KEY = "sai2::examples::two_hand_kp_force";
constexpr const char *TWO_HAND_KV_FORCE_KEY = "sai2::examples::two_hand_kv_force";
constexpr const char *TWO_HAND_KI_FORCE_KEY = "sai2::examples::two_hand_ki_force";

constexpr const char *TWO_HAND_KP_MOMENT_KEY = "sai2::examples::two_hand_kp_moment";
constexpr const char *TWO_HAND_KV_MOMENT_KEY = "sai2::examples::two_hand_kv_moment";
constexpr const char *TWO_HAND_KI_MOMENT_KEY = "sai2::examples::two_hand_ki_moment";

constexpr const char *TWO_HAND_KP_INTERNAL_SEPARATION_KEY = "sai2::examples::two_hand_kp_internal_separation";
constexpr const char *TWO_HAND_KV_INTERNAL_SEPARATION_KEY = "sai2::examples::two_hand_kv_internal_separation";

constexpr const char *TWO_HAND_KP_INTERNAL_ORI_KEY = "sai2::examples::two_hand_kp_internal_ori";
constexpr const char *TWO_HAND_KV_INTERNAL_ORI_KEY = "sai2::examples::two_hand_kv_internal_ori";

constexpr const char *TWO_HAND_USE_VEL_SAT_KEY = "sai2::examples::two_hand_use_velocity_saturation";
constexpr const char *TWO_HAND_LINEAR_VEL_SAT_KEY = "sai2::examples::two_hand_linear_velocity_sat";
constexpr const char *TWO_HAND_ANGULAR_VEL_SAT_KEY = "sai2::examples::two_hand_angular_velocity_sat";

constexpr const char *TWO_HAND_USE_INTERPOLATION_POS_KEY = "sai2::examples::two_hand_use_interpolation_pos";
constexpr const char *TWO_HAND_INTERPOLATION_POS_MAX_VEL_KEY = "sai2::examples::two_hand_pos_interpolation_max_vel";
constexpr const char *TWO_HAND_INTERPOLATION_POS_MAX_ACCEL_KEY = "sai2::examples::two_hand_pos_interpolation_max_accel";
constexpr const char *TWO_HAND_INTERPOLATION_POS_MAX_JERK_KEY = "sai2::examples::two_hand_pos_interpolation_max_jerk";

constexpr const char *TWO_HAND_USE_INTERPOLATION_ORI_KEY = "sai2::examples::two_hand_use_interpolation_ori";
constexpr const char *TWO_HAND_INTERPOLATION_ORI_MAX_VEL_KEY = "sai2::examples::two_hand_ori_interpolation_max_vel";
constexpr const char *TWO_HAND_INTERPOLATION_ORI_MAX_ACCEL_KEY = "sai2::examples::two_hand_ori_interpolation_max_accel";
constexpr const char *TWO_HAND_INTERPOLATION_ORI_MAX_JERK_KEY = "sai2::examples::two_hand_ori_interpolation_max_jerk";

// state-related keys
constexpr const char *PRIMITIVE_KEY = "sai2::examples::primitive";
constexpr const char *PRIMITIVE_COORDINATED_TASK = "primitive_coordinated_task";
constexpr const char *PRIMITIVE_INDEPENDENT_TASK = "primitive_independent_task";
constexpr const char *PRIMITIVE_FLOATING_TASK = "primitive_floating_task";
#endif