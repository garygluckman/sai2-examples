#include <iostream>
#include <string>
#include <csignal>
#include <utility>

#include "Sai2Model.h"
#include <dynamics3d.h>

#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"

#include "keys.h"

using namespace Eigen;

////////////////////// CONSTANTS //////////////////////

/** Redis write callback ID: used for setting keys for the first time  */
constexpr int INIT_WRITE_CALLBACK_ID = 0;

/** Redis read callback ID: used to grab updated values on each controller cycle */
constexpr int READ_CALLBACK_ID = 0;

/** Flag to determine if we are in simulation or grabbing values from the real robot */
constexpr bool flag_simulation = true;
// constexpr const bool flag_simulation = false;

/** Damping when dragging the robot in the floating task */
constexpr double FLOATING_TASK_KV = 2.5;

// Joint redis keys: select the correct key if we are in simulation or not
constexpr const char *JOINT_ANGLES_KEY = (flag_simulation) ? SIM_JOINT_ANGLES_KEY : HW_JOINT_ANGLES_KEY;
constexpr const char *JOINT_VELOCITIES_KEY = (flag_simulation) ? SIM_JOINT_VELOCITIES_KEY : HW_JOINT_VELOCITIES_KEY;
constexpr const char *JOINT_TORQUES_COMMANDED_KEY = (flag_simulation) ? SIM_JOINT_TORQUES_COMMANDED_KEY : HW_JOINT_TORQUES_COMMANDED_KEY;

////////////////// GLOBAL VARIABLES //////////////////
bool runloop = false;
std::string currentPrimitive = PRIMITIVE_JOINT_TASK;
RedisClient redis_client;

////////////////////// FUNCTIONS //////////////////////

/** 
 * Custom signal handler: used here to terminate the controller.
 * @param signal The signal (e.g. SIGINT) that was raised.
 */
void sighandler(int)
{
    runloop = false;
}


////////////////// JOINT TASK VARIABLES //////////////////
/**
 * Initializes the JointTask with default gains and settings and sets up the Redis callbacks.
 * @param joint_task    An uninitialized JointTask instance
 * @param redis_client  A RedisClient instance to set up callbacks
 */
void init_joint_task(Sai2Primitives::JointTask *joint_task, RedisClient& redis_client)
{
    int dof = joint_task->_robot->dof();

    // initialize joint_task object
    joint_task->_kp = 100.0;
    joint_task->_kv = 20.0;
    joint_task->_desired_position = joint_task->_robot->_q;
    joint_task->_use_isotropic_gains = true;
    joint_task->_use_interpolation_flag = false;
    
    // update values when we read all parameters on a new controller cycle
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KP_JOINT_KEY, joint_task->_kp);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KV_JOINT_KEY, joint_task->_kv);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_JOINT_POS_KEY, joint_task->_desired_position);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_JOINT_KEY, joint_task->_kp);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_JOINT_KEY, joint_task->_kv);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_JOINT_POS_KEY, joint_task->_desired_position);
}

/**
 * Updates the JointTask after a controller cycle.
 * @param joint_task    The JointTask instance to update after a controller cycle
 */
void update_joint_task(Sai2Primitives::JointTask *joint_task)
{
    // no-op: RedisClient::executeReadCallback updates all necessary variables
}


////////////////// POSORI TASK VARIABLES //////////////////
/** Boolean flag to determine if PosOri task uses velocity saturation */
int posori_use_velocity_saturation;

/** Tuple of (linear velocity, angular velocity) limits for velocity saturation*/
Eigen::Vector2d posori_velocity_saturation;

/** 
 * Initializes the PosOriTask with default gains and settings as well as sets up Redis callbacks.
 * @param posori_task   The PosOriTask object to initialize with default values
 * @param redis_client  The RedisClient object to set up callbacks
 */
void init_posori_task(Sai2Primitives::PosOriTask *posori_task, RedisClient& redis_client)
{
    Vector3d initial_position;
    Vector3d initial_velocity;

    int dof = posori_task->_robot->dof();
    posori_task->_robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
    posori_task->_robot->linearVelocity(initial_velocity, posori_task->_link_name, posori_task->_control_frame.translation());

    // initialize global variables
    posori_use_velocity_saturation = 0;

    // initialize posori_task object
    posori_task->_use_interpolation_flag = false;
    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation);
    posori_task->_kp_pos = 50.0;
    posori_task->_kv_pos = 12.0;
    posori_task->_ki_pos = 0.0;
    posori_task->_kp_ori = 50.0;
    posori_task->_kv_ori = 12.0;
    posori_task->_ki_ori = 0.0;

    posori_velocity_saturation(0) = posori_task->_linear_saturation_velocity;
    posori_velocity_saturation(1) = posori_task->_angular_saturation_velocity;

    // prepare redis read callback
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, USE_VEL_SAT_POSORI_KEY, posori_use_velocity_saturation);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, VEL_SAT_POSORI_KEY, posori_velocity_saturation);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KP_POS_KEY, posori_task->_kp_pos);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KV_POS_KEY, posori_task->_kv_pos);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_POS_KEY, posori_task->_desired_position);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_VEL_KEY, posori_task->_desired_velocity);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, USE_VEL_SAT_POSORI_KEY, posori_use_velocity_saturation);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, VEL_SAT_POSORI_KEY, posori_velocity_saturation);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_POS_KEY, posori_task->_kp_pos);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_POS_KEY, posori_task->_kv_pos);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_POS_KEY, posori_task->_desired_position);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_VEL_KEY, posori_task->_desired_velocity);
}

/**
 * Updates a PosOriTask instance after a controller cycle.
 * @param posori_task   The PosOriTask instance to update after a controller cycle
 */
void update_posori_task(Sai2Primitives::PosOriTask *posori_task)
{
    auto dof = posori_task->_robot->dof();

    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation);
    posori_task->_linear_saturation_velocity = posori_velocity_saturation(0);
    posori_task->_angular_saturation_velocity = posori_velocity_saturation(1);
}

int main(int argc, char **argv) 
{
    // open redis
    redis_client.connect();

    redis_client.createReadCallback(READ_CALLBACK_ID);
    redis_client.createWriteCallback(INIT_WRITE_CALLBACK_ID);
    
    // set up signal handlers
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // initialize controller state
    redis_client.set(PRIMITIVE_KEY, currentPrimitive);

    // notify UI that we are initializing
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZING);

    // load robots
    auto robot = new Sai2Model::Sai2Model(ROBOT_FILE, false);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.executeReadCallback(READ_CALLBACK_ID);
    robot->updateModel();

    // bind current state to what redis says
    redis_client.addStringToReadCallback(READ_CALLBACK_ID, PRIMITIVE_KEY, currentPrimitive);

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    VectorXd posori_task_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);

    const std::string link_name = "link7";
    const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.12);

    // initialize tasks
    Sai2Primitives::PosOriTask *posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
    init_posori_task(posori_task, redis_client);

    Sai2Primitives::JointTask *joint_task = new Sai2Primitives::JointTask(robot);
    init_joint_task(joint_task, redis_client);

    Sai2Primitives::JointTask *floating_task = new Sai2Primitives::JointTask(robot);
    floating_task->_use_interpolation_flag = false;
    floating_task->_kp = 0;
    floating_task->_kv = FLOATING_TASK_KV;

    // initialization complete
    redis_client.executeWriteCallback(INIT_WRITE_CALLBACK_ID);
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZED);

    MatrixXd N_prec;

    // create a loop timer
    double control_freq = 1000;
    LoopTimer timer;
    timer.setLoopFrequency(control_freq);   // 1 KHz
    double last_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;
    timer.initializeTimer(1000000); // 1 ms pause before starting loop

    unsigned long long controller_counter = 0;

    runloop = true;
    while (runloop) 
    { 
        fTimerDidSleep = timer.waitForNextLoop();

        // update time
        double curr_time = timer.elapsedTime();
        double loop_dt = curr_time - last_time;

        std::string oldPrimitive = currentPrimitive;

        // update all values tied to redis
        redis_client.executeReadCallback(READ_CALLBACK_ID);
        update_joint_task(joint_task);
        update_posori_task(posori_task);

        if (flag_simulation)
        {
            robot->updateModel();
            robot->coriolisForce(coriolis);
        }
        else
        {
            robot->updateKinematics();
            robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
            robot->_M_inv = robot->_M.inverse();
            coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
        }

        N_prec.setIdentity(dof, dof);

        // if we just changed primitives, reset & reinit
        if (currentPrimitive != oldPrimitive)
        {
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                joint_task->_current_position = robot->_q;
                joint_task->reInitializeTask();
                redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q); 
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
            {
                posori_task->reInitializeTask();
                redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_current_position);
            }
            else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
            {
                floating_task->_current_position = robot->_q;
                floating_task->reInitializeTask();
            }
        }

        // steady-state operations for each task
        else if (currentPrimitive == PRIMITIVE_JOINT_TASK)
        {
            joint_task->updateTaskModel(N_prec);
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
        }
        else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
        {
            joint_task->_use_isotropic_gains = true;
            posori_task->updateTaskModel(N_prec);
            N_prec = posori_task->_N;
            joint_task->updateTaskModel(N_prec);

            // we also need to read linear & angular velocity
            posori_task->_desired_angular_velocity.setZero();

            // compute torques
            posori_task->computeTorques(posori_task_torques);
            joint_task->computeTorques(joint_task_torques);
            command_torques = posori_task_torques + joint_task_torques + coriolis;
        }
        else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
        {
            floating_task->updateTaskModel(MatrixXd::Identity(dof, dof));
            Eigen::VectorXd floating_task_torques;
            floating_task->computeTorques(floating_task_torques);
            command_torques = floating_task_torques + coriolis;
        }
    
        // -------------------------------------------
        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

        // log current EE position and velocity to redis
        Vector3d current_pos;
        robot->position(current_pos, link_name, pos_in_link);

        Vector3d current_vel;
        robot->linearVelocity(current_vel, link_name, pos_in_link);

        redis_client.setEigenMatrixJSON(CURRENT_EE_POS_KEY, current_pos);
        redis_client.setEigenMatrixJSON(CURRENT_EE_VEL_KEY, current_vel);
        
        // -------------------------------------------
        if (controller_counter % 500 == 0)
        {
            std::cout << "current primitive: " << currentPrimitive << std::endl;
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                std::cout << curr_time << std::endl;
                std::cout << "desired position : " << joint_task->_desired_position.transpose() << std::endl;
                std::cout << "current position : " << joint_task->_current_position.transpose() << std::endl;
                std::cout << "position error : " << (joint_task->_desired_position - joint_task->_current_position).norm() << std::endl;
                std::cout << std::endl;
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
            {
                std::cout << curr_time << std::endl;
                std::cout << "desired position : " << posori_task->_desired_position.transpose() << std::endl;
                std::cout << "current position : " << posori_task->_current_position.transpose() << std::endl;
                std::cout << "position error : " << (posori_task->_desired_position - posori_task->_current_position).norm() << std::endl;
                std::cout << std::endl;
            }
        }

        controller_counter++;

        // -------------------------------------------
        // update last time
        last_time = curr_time;
    }

    command_torques.setZero();
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << std::endl;
    std::cout << "Control Loop run time  : " << end_time << " seconds" << std::endl;
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << std::endl;
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz" << std::endl;

    delete robot;
    delete joint_task;
    delete posori_task;
    delete floating_task;
    return 0;
}
