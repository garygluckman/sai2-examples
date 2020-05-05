#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "tasks/TwoHandTwoRobotsTask.h"

#include <iostream>
#include <string>
#include <csignal>
#include <vector>
#include <memory>
#include <mutex>

#include "keys.h"
#include "panda.h"

using namespace Eigen;

////////////////////// CONSTANTS //////////////////////
/** Redis write callback ID used when setting all Redis keys to their initial values */
constexpr int INIT_WRITE_CALLBACK_ID = 0;

/** Redis read callback ID used when reading Redis keys each controller cycle */
constexpr int READ_CALLBACK_ID = 0;

/** Redis write callback ID used when writing some Redis keys each controller cycle */
constexpr int CYCLE_WRITE_CALLBACK_ID = 1;

/** Flag to determine if we are in simulation or grabbing values from the real robot */
constexpr bool flag_simulation = true;
// constexpr const bool flag_simulation = false;

/** Damping when dragging the robot in the floating task */
constexpr double FLOATING_TASK_KV = 2.5;

////////////////// ACTUAL REDIS KEYS //////////////////
constexpr std::array<const char *, N_ROBOTS> JOINT_ANGLES_KEYS = (flag_simulation) ? SIM_JOINT_ANGLES_KEYS : HW_JOINT_ANGLES_KEYS;
constexpr std::array<const char *, N_ROBOTS> JOINT_VELOCITIES_KEYS = (flag_simulation) ? SIM_JOINT_VELOCITIES_KEYS : HW_JOINT_VELOCITIES_KEYS;
constexpr std::array<const char *, N_ROBOTS> JOINT_TORQUES_COMMANDED_KEYS = (flag_simulation) ? SIM_JOINT_TORQUES_COMMANDED_KEYS : HW_JOINT_TORQUES_COMMANDED_KEYS;
constexpr std::array<const char *, N_ROBOTS> SENSED_FORCES_KEYS = SIM_SENSED_FORCES_KEYS;

////////////////// GLOBAL VARIABLES //////////////////
bool runloop = false;
std::string currentPrimitive = PRIMITIVE_INDEPENDENT_TASK;
unsigned long long controller_counter = 0;
RedisClient redis_client;

std::vector<std::shared_ptr<Sai2Model::Sai2Model>> robots;
std::vector<std::shared_ptr<Sai2Primitives::JointTask>> joint_tasks;
std::vector<std::shared_ptr<Sai2Primitives::PosOriTask>> posori_tasks;
std::shared_ptr<Sai2Primitives::TwoHandTwoRobotsTask> two_hand_task;
std::vector<std::shared_ptr<Sai2Primitives::JointTask>> floating_tasks;
std::vector<VectorXd> coriolis;
std::mutex model_mutex; 

////////////////////// FUNCTIONS //////////////////////
/** 
 * Custom signal handler: used here to terminate the controller.
 * @param signal The signal (e.g. SIGINT) that was raised.
 */
void sighandler(int)
{
    runloop = false;
}

////////////////////// TWO HAND TASK //////////////////////
/** Flag to use internal force control or not */
int two_hand_use_internal_force_flag;

/** Flag to use interpolation on object position */
int two_hand_use_interpolation_pos_flag;

/** Flag to use interpolation on object orientation */
int two_hand_use_interpolation_ori_flag;

/** Flag to use velocity saturation on object pos & ori */
int two_hand_use_velocity_saturation_flag;

/** Max OTG linear velocity of object position */
double two_hand_interpolation_pos_max_vel;

/** Max OTG linear acceleration of object position */
double two_hand_interpolation_pos_max_accel;

/** Max OTG linear jerk of object position */
double two_hand_interpolation_pos_max_jerk;

/** Max OTG angular velocity of object orientation */
double two_hand_interpolation_ori_max_vel;

/** Max OTG angular acceleration of object orientation */
double two_hand_interpolation_ori_max_accel;

/** Max OTG angular jerk of object orientation */
double two_hand_interpolation_ori_max_jerk;

/** Each robot's sensed force/moments at the end effectors */
std::vector<VectorXd> sensed_force_moments;

/**
 * Initializes the TwoHandTwoRobotsTask object with default values
 * 
 * @param two_hand_task     The TwoHandTwoRobotsTask object to initialize with default values
 * @param redis             The RedisClient instance to use when setting callbacks
 */
void init_two_handed_task(Sai2Primitives::TwoHandTwoRobotsTask *two_hand_task, RedisClient& redis)
{
    // initialize global variables
    two_hand_use_internal_force_flag = 1;
    for (int i = 0; i < N_ROBOTS; i++)
    {
        sensed_force_moments.push_back(VectorXd::Zero(robots[i]->dof()));
    }

    two_hand_use_interpolation_pos_flag = 1;
    two_hand_use_interpolation_ori_flag = 1;
    two_hand_use_velocity_saturation_flag = 0;
    two_hand_interpolation_pos_max_vel = 0.3;
    two_hand_interpolation_pos_max_accel = 0.6;
    two_hand_interpolation_pos_max_jerk = 1.2;
    two_hand_interpolation_ori_max_vel = M_PI / 4;
    two_hand_interpolation_ori_max_accel = M_PI / 2;
    two_hand_interpolation_ori_max_jerk = M_PI;

    // initialize two hand two robot task
    two_hand_task->_internal_force_control_flag = false;
    two_hand_task->_use_interpolation_pos_flag = bool(two_hand_use_interpolation_pos_flag);
    two_hand_task->_use_interpolation_ori_flag = bool(two_hand_use_interpolation_ori_flag);
    two_hand_task->_use_velocity_saturation_flag = bool(two_hand_use_velocity_saturation_flag);

#ifdef USING_OTG
    two_hand_task->_otg_pos->setMaxVelocity(two_hand_interpolation_pos_max_vel);
    two_hand_task->_otg_pos->setMaxAcceleration(two_hand_interpolation_pos_max_accel);
    two_hand_task->_otg_pos->setMaxJerk(two_hand_interpolation_pos_max_jerk);
    two_hand_task->_otg_ori->setMaxVelocity(two_hand_interpolation_ori_max_vel);
    two_hand_task->_otg_ori->setMaxAcceleration(two_hand_interpolation_ori_max_accel);
    two_hand_task->_otg_ori->setMaxJerk(two_hand_interpolation_ori_max_jerk);
#endif

    // update values when we read all parameters on a new controller cycle
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KP_POS_KEY, two_hand_task->_kp_pos);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KV_POS_KEY, two_hand_task->_kv_pos);    
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KI_POS_KEY, two_hand_task->_ki_pos);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KP_ORI_KEY, two_hand_task->_kp_ori);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KV_ORI_KEY, two_hand_task->_kv_ori);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KI_ORI_KEY, two_hand_task->_ki_ori);

    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KP_FORCE_KEY, two_hand_task->_kp_force);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KV_FORCE_KEY, two_hand_task->_kv_force);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KI_FORCE_KEY, two_hand_task->_ki_force);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KP_MOMENT_KEY, two_hand_task->_kp_moment);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KV_MOMENT_KEY, two_hand_task->_kv_moment);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KI_MOMENT_KEY, two_hand_task->_ki_moment);

    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KP_INTERNAL_SEPARATION_KEY, two_hand_task->_kp_internal_separation);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KV_INTERNAL_SEPARATION_KEY, two_hand_task->_kv_internal_separation);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KP_INTERNAL_ORI_KEY, two_hand_task->_kp_internal_ori);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_KV_INTERNAL_ORI_KEY, two_hand_task->_kv_internal_ori);

    redis.addIntToReadCallback(READ_CALLBACK_ID, TWO_HAND_USE_VEL_SAT_KEY, two_hand_use_velocity_saturation_flag);    
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_LINEAR_VEL_SAT_KEY, two_hand_task->_linear_saturation_velocity);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_ANGULAR_VEL_SAT_KEY, two_hand_task->_angular_saturation_velocity);

    redis.addIntToReadCallback(READ_CALLBACK_ID, TWO_HAND_USE_INTERPOLATION_POS_KEY, two_hand_use_interpolation_pos_flag);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_INTERPOLATION_POS_MAX_VEL_KEY, two_hand_interpolation_pos_max_vel);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_INTERPOLATION_POS_MAX_ACCEL_KEY, two_hand_interpolation_pos_max_accel);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_INTERPOLATION_POS_MAX_JERK_KEY, two_hand_interpolation_pos_max_jerk);

    redis.addIntToReadCallback(READ_CALLBACK_ID, TWO_HAND_USE_INTERPOLATION_ORI_KEY, two_hand_use_interpolation_ori_flag);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_INTERPOLATION_ORI_MAX_VEL_KEY, two_hand_interpolation_ori_max_vel);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_INTERPOLATION_ORI_MAX_ACCEL_KEY, two_hand_interpolation_ori_max_accel);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, TWO_HAND_INTERPOLATION_ORI_MAX_JERK_KEY, two_hand_interpolation_ori_max_jerk);    

    redis.addIntToReadCallback(READ_CALLBACK_ID, TWO_HAND_USE_INTERNAL_FORCE_KEY, two_hand_use_internal_force_flag);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, DESIRED_INTERNAL_SEPARATION_KEY, two_hand_task->_desired_internal_separation);
    redis.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_INTERNAL_ANGLES_KEY, two_hand_task->_desired_internal_angles);
    redis.addDoubleToReadCallback(READ_CALLBACK_ID, DESIRED_INTERNAL_TENSION_KEY, two_hand_task->_desired_internal_tension);
    redis.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_OBJECT_POSITION_KEY, two_hand_task->_desired_object_position);
    redis.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_OBJECT_ORIENTATION_KEY, two_hand_task->_desired_object_orientation);

    for (int i = 0; i < N_ROBOTS; i++)
    {
        redis.addEigenToReadCallback(READ_CALLBACK_ID, SENSED_FORCES_KEYS[i], sensed_force_moments[i]);
    }
    
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KP_POS_KEY, two_hand_task->_kp_pos);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KV_POS_KEY, two_hand_task->_kv_pos);    
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KI_POS_KEY, two_hand_task->_ki_pos);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KP_ORI_KEY, two_hand_task->_kp_ori);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KV_ORI_KEY, two_hand_task->_kv_ori);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KI_ORI_KEY, two_hand_task->_ki_ori);

    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KP_FORCE_KEY, two_hand_task->_kp_force);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KV_FORCE_KEY, two_hand_task->_kv_force);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KI_FORCE_KEY, two_hand_task->_ki_force);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KP_MOMENT_KEY, two_hand_task->_kp_moment);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KV_MOMENT_KEY, two_hand_task->_kv_moment);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KI_MOMENT_KEY, two_hand_task->_ki_moment);

    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KP_INTERNAL_SEPARATION_KEY, two_hand_task->_kp_internal_separation);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KV_INTERNAL_SEPARATION_KEY, two_hand_task->_kv_internal_separation);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KP_INTERNAL_ORI_KEY, two_hand_task->_kp_internal_ori);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_KV_INTERNAL_ORI_KEY, two_hand_task->_kv_internal_ori);

    redis.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_USE_VEL_SAT_KEY, two_hand_use_velocity_saturation_flag);    
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_LINEAR_VEL_SAT_KEY, two_hand_task->_linear_saturation_velocity);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_ANGULAR_VEL_SAT_KEY, two_hand_task->_angular_saturation_velocity);

    redis.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_USE_INTERPOLATION_POS_KEY, two_hand_use_interpolation_pos_flag);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_INTERPOLATION_POS_MAX_VEL_KEY, two_hand_interpolation_pos_max_vel);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_INTERPOLATION_POS_MAX_ACCEL_KEY, two_hand_interpolation_pos_max_accel);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_INTERPOLATION_POS_MAX_JERK_KEY, two_hand_interpolation_pos_max_jerk);

    redis.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_USE_INTERPOLATION_ORI_KEY, two_hand_use_interpolation_ori_flag);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_INTERPOLATION_ORI_MAX_VEL_KEY, two_hand_interpolation_ori_max_vel);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_INTERPOLATION_ORI_MAX_ACCEL_KEY, two_hand_interpolation_ori_max_accel);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_INTERPOLATION_ORI_MAX_JERK_KEY, two_hand_interpolation_ori_max_jerk);    

    redis.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, TWO_HAND_USE_INTERNAL_FORCE_KEY, two_hand_use_internal_force_flag);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_INTERNAL_SEPARATION_KEY, two_hand_task->_desired_internal_separation);
    redis.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_INTERNAL_ANGLES_KEY, two_hand_task->_desired_internal_angles);
    redis.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_INTERNAL_TENSION_KEY, two_hand_task->_desired_internal_tension);
    redis.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_OBJECT_POSITION_KEY, two_hand_task->_desired_object_position);
    redis.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_OBJECT_ORIENTATION_KEY, two_hand_task->_desired_object_orientation);
}

/**
 * Updates the given TwoHandTwoRobotsTask after a controller cycle.
 * @param two_hand_task     The TwoHandTwoRobotsTask to update after a controller cycle
 */
void update_two_handed_task(Sai2Primitives::TwoHandTwoRobotsTask *two_hand_task)
{
    two_hand_task->_internal_force_control_flag = bool(two_hand_use_internal_force_flag);
    two_hand_task->_use_interpolation_pos_flag = bool(two_hand_use_interpolation_pos_flag);
    two_hand_task->_use_interpolation_ori_flag = bool(two_hand_use_interpolation_ori_flag);
    two_hand_task->_use_velocity_saturation_flag = bool(two_hand_use_velocity_saturation_flag);
#ifdef USING_OTG
    two_hand_task->_otg_pos->setMaxVelocity(two_hand_interpolation_pos_max_vel);
    two_hand_task->_otg_pos->setMaxAcceleration(two_hand_interpolation_pos_max_accel);
    two_hand_task->_otg_pos->setMaxJerk(two_hand_interpolation_pos_max_jerk);
    two_hand_task->_otg_ori->setMaxVelocity(two_hand_interpolation_ori_max_vel);
    two_hand_task->_otg_ori->setMaxAcceleration(two_hand_interpolation_ori_max_accel);
    two_hand_task->_otg_ori->setMaxJerk(two_hand_interpolation_ori_max_jerk);
#endif

    if (two_hand_task->_internal_force_control_flag)
    { 
        Affine3d T_world_com = Affine3d::Identity();
        T_world_com.linear() = two_hand_task->_current_object_orientation;
        T_world_com.translation() = two_hand_task->_current_object_position;
        two_hand_task->setObjectMassPropertiesAndInitialInertialFrameLocation(1.0, T_world_com, 0.1 * Matrix3d::Identity());


        two_hand_task->updateSensedForcesAndMoments(
            sensed_force_moments[0].head(3), sensed_force_moments[0].tail(3),
            sensed_force_moments[1].head(3), sensed_force_moments[1].tail(3)
        );
    }
}

/**
 * Updates each task's (joint, posori, two hand) task model at a slower 200 Hz
 * in a separate thread.
 */
void updateModelThread();

int main()
{
    redis_client.connect();

    // set up redis callbacks
    redis_client.createReadCallback(READ_CALLBACK_ID);
    redis_client.createWriteCallback(INIT_WRITE_CALLBACK_ID);
    redis_client.createWriteCallback(CYCLE_WRITE_CALLBACK_ID);

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // initialize controller state
    redis_client.set(PRIMITIVE_KEY, currentPrimitive);

    // notify UI that we are initializing
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZING);

    // position of robots in world
    std::vector<Affine3d> robot_pose_in_world;
    Affine3d pose = Affine3d::Identity();
    pose.translation() = Vector3d(0, -0.5, 0.0);
    pose.linear() = AngleAxisd(0.3010693, Vector3d::UnitZ()).toRotationMatrix();
    robot_pose_in_world.push_back(pose);

    pose.translation() = Vector3d(-0.06, 0.57, 0.0);
    pose.linear() = AngleAxisd(-1.0864675, Vector3d::UnitZ()).toRotationMatrix();
    robot_pose_in_world.push_back(pose);

    // load robots
    for (int i = 0; i < N_ROBOTS; i++)
    {
        robots.push_back(std::make_shared<Sai2Model::Sai2Model>(ROBOT_FILES[i], false, robot_pose_in_world[i]));
        redis_client.addEigenToReadCallback(READ_CALLBACK_ID, JOINT_ANGLES_KEYS[i], robots[i]->_q);
        redis_client.addEigenToReadCallback(READ_CALLBACK_ID, JOINT_VELOCITIES_KEYS[i], robots[i]->_dq);   
    }

    redis_client.executeReadCallback(READ_CALLBACK_ID);

    for (int i = 0; i < N_ROBOTS; i++)
    {
        robots[i]->updateModel();
    }
    
    // bind current state to what redis says
    redis_client.addStringToReadCallback(READ_CALLBACK_ID, PRIMITIVE_KEY, currentPrimitive);

    // prepare task controllers
    std::vector<VectorXd> command_torques;    
    std::vector<VectorXd> joint_torques;
    std::vector<VectorXd> posori_torques;
    std::vector<VectorXd> two_hand_torques;
    std::vector<Vector3d> current_ee_pos;
    std::vector<Vector3d> current_ee_vel;

    const std::string link_name = "link7";
    const Eigen::Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.125);

    for (int i = 0; i < N_ROBOTS; i++)
    {
        auto robot = robots[i].get();
        auto dof = robot->dof();

        auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
        auto posori_task = std::make_shared<Sai2Primitives::PosOriTask>(robot, link_name, pos_in_link);
        auto floating_task = std::make_shared<Sai2Primitives::JointTask>(robot);

        init_joint_task(joint_task.get(), redis_client, i, READ_CALLBACK_ID, INIT_WRITE_CALLBACK_ID);
        init_posori_task(posori_task.get(), redis_client, i, READ_CALLBACK_ID, INIT_WRITE_CALLBACK_ID);

        floating_task->_kp = 0;
        floating_task->_kv = FLOATING_TASK_KV;
        floating_task->_use_interpolation_flag = false;

        joint_tasks.push_back(joint_task);
        posori_tasks.push_back(posori_task);
        floating_tasks.push_back(floating_task);

        command_torques.push_back(VectorXd::Zero(dof));
        joint_torques.push_back(VectorXd::Zero(dof));
        posori_torques.push_back(VectorXd::Zero(dof));
        coriolis.push_back(VectorXd::Zero(dof));
        two_hand_torques.push_back(VectorXd::Zero(dof));

        current_ee_pos.push_back(Vector3d::Zero());
        current_ee_vel.push_back(Vector3d::Zero());
    }

    // initialize logging variables & command_torque writes
    for (int i = 0; i < N_ROBOTS; i++)
    {
        redis_client.addEigenToWriteCallback(CYCLE_WRITE_CALLBACK_ID, JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
        redis_client.addEigenToWriteCallback(CYCLE_WRITE_CALLBACK_ID, CURRENT_EE_POS_KEYS[i], current_ee_pos[i]);
        redis_client.addEigenToWriteCallback(CYCLE_WRITE_CALLBACK_ID, CURRENT_EE_VEL_KEYS[i], current_ee_vel[i]);
    }

    // two hand task
    two_hand_task = std::make_shared<Sai2Primitives::TwoHandTwoRobotsTask>(
        robots[0].get(), robots[1].get(),
        posori_tasks[0]->_link_name, posori_tasks[1]->_link_name,
        posori_tasks[0]->_control_frame, posori_tasks[1]->_control_frame);

    init_two_handed_task(two_hand_task.get(), redis_client);

    // object properties
    double object_mass = 1.0;
    Matrix3d object_inertia = 0.1 * Matrix3d::Identity();

    // set the sensor frames
    const std::string sensor_link_name = "link7";
    Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
    T_link_sensor.translation() = Vector3d(0, 0, 0.113);
    two_hand_task->setForceSensorFrames(sensor_link_name, T_link_sensor, sensor_link_name, T_link_sensor);

    // set goal positions for the first state in world frame
    Matrix3d robot1_desired_orientation_in_world;
    Matrix3d robot2_desired_orientation_in_world;
    robot1_desired_orientation_in_world << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    robot2_desired_orientation_in_world << 1, 0, 0, 0, 0, -1, 0, 1, 0;

    posori_desired_position_world[0] = Vector3d(0.2, -0.2, 0.15);
    posori_desired_orientation_world[0] = robot1_desired_orientation_in_world;
    posori_desired_position_world[1] = Vector3d(0.2,  0.2, 0.15);
    posori_desired_orientation_world[1] = robot2_desired_orientation_in_world;

    // initialization complete
    redis_client.executeWriteCallback(INIT_WRITE_CALLBACK_ID);
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZED);

    runloop = true;

    // start update_model thread
    std::thread model_update_thread(updateModelThread);

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    double current_time = 0;
    double prev_time = 0;
    double dt = 0;
    bool fTimerDidSleep = true;
    double start_time = timer.elapsedTime(); //secs
    double initial_time = 0;

    while (runloop)
    {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        current_time = timer.elapsedTime() - start_time;
        dt = current_time - prev_time;

        std::string oldPrimitive = currentPrimitive;
        redis_client.executeReadCallback(READ_CALLBACK_ID);

        // we lock in main controller loop as the slower task model update 
        // is not thread-safe
        std::lock_guard<std::mutex> lock(model_mutex);

        for (int i = 0; i < N_ROBOTS; i++)
        {
            update_posori_task(posori_tasks[i].get(), i);
        }

        update_two_handed_task(two_hand_task.get());

        // if primitive changes, reset & reinit
        if (currentPrimitive != oldPrimitive)
        {
            if (currentPrimitive == PRIMITIVE_INDEPENDENT_TASK)
            {
                for (int i = 0; i < N_ROBOTS; i++)
                {
                    auto posori_task = posori_tasks[i];
                    posori_task->reInitializeTask();
                    redis_client.setEigenMatrixJSON(DESIRED_POS_KEYS[i], posori_task->_current_position);
                    redis_client.setEigenMatrixJSON(DESIRED_ORI_KEYS[i], posori_task->_current_orientation);
                }
            }
            else if (currentPrimitive == PRIMITIVE_COORDINATED_TASK)
            {
                two_hand_task->reInitializeTask();
                for (int i = 0; i < N_ROBOTS; i++)
                {
                    joint_tasks[i]->reInitializeTask();
                }

                redis_client.executeWriteCallback(INIT_WRITE_CALLBACK_ID);
            }
            else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
            {
                for (int i = 0; i < N_ROBOTS; i++)
                {
                    auto floating_task = floating_tasks[i];
                    floating_task->reInitializeTask();
                }
            }
        }

        // steady-state operations for each task
        else if (currentPrimitive == PRIMITIVE_INDEPENDENT_TASK)
        {
            for (int i = 0; i < N_ROBOTS; i++)
            {
                auto joint_task = joint_tasks[i];
                auto posori_task = posori_tasks[i];

                // we also need to read linear & angular velocity
                posori_task->_desired_angular_velocity.setZero();

                // compute torques
                posori_task->computeTorques(posori_torques[i]);
                joint_task->computeTorques(joint_torques[i]);
                command_torques[i] = posori_torques[i] + joint_torques[i] + coriolis[i];
            }
        }
        else if (currentPrimitive == PRIMITIVE_COORDINATED_TASK)
        {
            // compute the torques
            two_hand_task->computeTorques(two_hand_torques[0], two_hand_torques[1]);

            for (int i = 0; i < N_ROBOTS; i++)
            {
                joint_tasks[i]->computeTorques(joint_torques[i]);
                command_torques[i] = two_hand_torques[i] + joint_torques[i];
            }
        }
        else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
        {
            for (int i = 0; i < N_ROBOTS; i++)
            {
                Eigen::VectorXd floating_task_torques;
                floating_tasks[i]->computeTorques(floating_task_torques);
                command_torques[i] = floating_task_torques + coriolis[i];
            }
        }

        // -------------------------------------------
        // log current EE position and velocity to redis
        for (int i = 0; i < N_ROBOTS; i++) 
        {
            robots[i]->position(current_ee_pos[i], link_name, pos_in_link);
            robots[i]->linearVelocity(current_ee_vel[i], link_name, pos_in_link);
        }

        // write command_torques, current_ee_pos, current_ee_vel
        redis_client.executeWriteCallback(CYCLE_WRITE_CALLBACK_ID);

        // -------------------------------------------
        if (controller_counter % 500 == 0)
        {
            std::cout << "current primitive: " << currentPrimitive << std::endl;
            if (currentPrimitive == PRIMITIVE_INDEPENDENT_TASK)
            {
                std::cout << current_time << std::endl;
                for (int i = 0; i < N_ROBOTS; i++)
                {
                    auto posori_task = posori_tasks[i];
                    std::cout << "[Panda " << i + 1 << "] torques: " <<  command_torques[i].transpose() << std::endl;
                    std::cout << "[Panda " << i + 1 << "] desired position : " << posori_task->_desired_position.transpose() << std::endl;
                    std::cout << "[Panda " << i + 1 << "] current position : " << posori_task->_current_position.transpose() << std::endl;
                    std::cout << "[Panda " << i + 1 << "] position error : " << (posori_task->_desired_position - posori_task->_current_position).norm() << std::endl;
                }
                std::cout << std::endl;
            }
            else if (currentPrimitive == PRIMITIVE_COORDINATED_TASK)
            {
                // TODO
                std::cout << current_time << std::endl;
                std::cout << std::endl;
            }
        }

        prev_time = current_time;
        controller_counter++;
    }

    for (int i = 0; i < N_ROBOTS; i++)
    {
        command_torques[i].setZero();
        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
    }

    model_update_thread.join();

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";

    return 0;
}

void updateModelThread()
{
    // prepare task controllers
    std::vector<MatrixXd> N_prec;
    for (int i = 0; i < N_ROBOTS; i++)
    {
        int dof = robots[i]->dof();
        N_prec.push_back(MatrixXd::Identity(dof, dof));
    }

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(200);

    while (runloop)
    {
        timer.waitForNextLoop();

        // updating task model is not thread-safe: we must lock so that the
        // main controller thread does not compute torques while we update
        // internal the task models
        std::lock_guard<std::mutex> lock(model_mutex);

        // read robot state from redis and update robot model
        for (int i = 0; i < N_ROBOTS; i++)
        {
            if (flag_simulation)
            {
                robots[i]->updateModel();
            }
            else
            {
                robots[i]->updateKinematics();
                robots[i]->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEYS[i]);
                robots[i]->_M_inv = robots[i]->_M.inverse();
                coriolis[i] = redis_client.getEigenMatrixJSON(CORIOLIS_KEYS[i]);
            }

            robots[i]->coriolisForce(coriolis[i]);
        }

        if (currentPrimitive == PRIMITIVE_INDEPENDENT_TASK)
        {
            for (int i = 0; i < N_ROBOTS; i++)
            {
                N_prec[i].setIdentity();
                posori_tasks[i]->updateTaskModel(N_prec[i]);
                N_prec[i] = posori_tasks[i]->_N;
                joint_tasks[i]->updateTaskModel(N_prec[i]);
            }
        }
        else if (currentPrimitive == PRIMITIVE_COORDINATED_TASK)
        {
            for (int i = 0; i < N_ROBOTS; i++)
            {
                N_prec[i].setIdentity();
            }
            
            two_hand_task->updateTaskModel(N_prec[0], N_prec[1]);
            N_prec[0] = two_hand_task->_N_1;
            N_prec[1] = two_hand_task->_N_2;

            for (int i = 0; i < N_ROBOTS; i++)
            {
                joint_tasks[i]->updateTaskModel(N_prec[i]);
            }
        }
        else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
        {
            for (int i = 0; i < N_ROBOTS; i++)
            {
                N_prec[i].setIdentity();
                auto floating_task = floating_tasks[i];
                const int dof = floating_task->_robot->dof();
                floating_task->updateTaskModel(N_prec[i]);
            }
        }
    }
}
