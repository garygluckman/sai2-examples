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

using namespace std;
using namespace Eigen;

////////////////// GLOBAL VARIABLES //////////////////
bool runloop = false;
string currentPrimitive = PRIMITIVE_JOINT_TASK;
RedisClient redis_client;

////////////////////// FUNCTIONS //////////////////////
void sighandler(int)
{
    runloop = false;
}


////////////////// JOINT TASK VARIABLES //////////////////
Eigen::VectorXd joint_kp_nonisotropic;
Eigen::VectorXd joint_kv_nonisotropic;
int joint_use_interpolation;
int joint_use_velocity_saturation;
int joint_use_isotropic_gains;
std::string joint_dynamic_decoupling_mode;

void init_joint_task(Sai2Primitives::JointTask *joint_task, RedisClient& redis_client)
{
    int dof = joint_task->_robot->dof();

    // initialize global variables
    joint_kp_nonisotropic = 100.0 * VectorXd::Ones(dof);
    joint_kv_nonisotropic = 20.0 * VectorXd::Ones(dof);
    joint_use_interpolation = 1;
    joint_use_velocity_saturation = 0;
    joint_use_isotropic_gains = 1;
    joint_dynamic_decoupling_mode = "inertia_saturation";

    // initialize joint_task object
    joint_task->_kp = 100.0;
    joint_task->_kv = 20.0;
    joint_task->setNonIsotropicGains(joint_kp_nonisotropic, joint_kv_nonisotropic, VectorXd::Zero(dof));
    joint_task->_desired_position = joint_task->_robot->_q;
    joint_task->_use_isotropic_gains = bool(joint_use_isotropic_gains);
    joint_task->_use_velocity_saturation_flag = bool(joint_use_velocity_saturation);
    joint_task->_use_interpolation_flag = bool(joint_use_interpolation);
    joint_task->setDynamicDecouplingInertiaSaturation();
    
    // update values when we read all parameters on a new controller cycle
    redis_client.addDoubleToRead(KP_JOINT_KEY, joint_task->_kp);
    redis_client.addDoubleToRead(KV_JOINT_KEY, joint_task->_kv);
    redis_client.addEigenToRead(KP_NON_ISOTROPIC_JOINT_KEY, joint_kp_nonisotropic);
    redis_client.addEigenToRead(KV_NON_ISOTROPIC_JOINT_KEY, joint_kv_nonisotropic);
    redis_client.addIntToRead(JOINT_USE_INTERPOLATION, joint_use_interpolation);
    redis_client.addIntToRead(USE_ISOTROPIC_JOINT_GAINS_KEY, joint_use_isotropic_gains);
    redis_client.addIntToRead(USE_VEL_SAT_JOINT_KEY, joint_use_velocity_saturation);
    redis_client.addStringToRead(DYN_DEC_JOINT_KEY, joint_dynamic_decoupling_mode);
    redis_client.addEigenToRead(DESIRED_JOINT_POS_KEY, joint_task->_desired_position);
    redis_client.addEigenToRead(VEL_SAT_JOINT_KEY, joint_task->_saturation_velocity);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addDoubleToWrite(KP_JOINT_KEY, joint_task->_kp);
    redis_client.addDoubleToWrite(KV_JOINT_KEY, joint_task->_kv);
    redis_client.addEigenToWrite(KP_NON_ISOTROPIC_JOINT_KEY, joint_kp_nonisotropic);
    redis_client.addEigenToWrite(KV_NON_ISOTROPIC_JOINT_KEY, joint_kv_nonisotropic);
    redis_client.addIntToWrite(JOINT_USE_INTERPOLATION, joint_use_interpolation);
    redis_client.addIntToWrite(USE_ISOTROPIC_JOINT_GAINS_KEY, joint_use_isotropic_gains);
    redis_client.addIntToWrite(USE_VEL_SAT_JOINT_KEY, joint_use_velocity_saturation);
    redis_client.addStringToWrite(DYN_DEC_JOINT_KEY, joint_dynamic_decoupling_mode);
    redis_client.addEigenToWrite(DESIRED_JOINT_POS_KEY, joint_task->_desired_position);
    redis_client.addEigenToWrite(VEL_SAT_JOINT_KEY, joint_task->_saturation_velocity);
}

void update_joint_task(Sai2Primitives::JointTask *joint_task)
{
    if (joint_use_interpolation && !joint_task->_use_interpolation_flag)
        joint_task->reInitializeTask();

    joint_task->_use_interpolation_flag = bool(joint_use_interpolation);
    joint_task->_use_isotropic_gains = bool(joint_use_isotropic_gains);
    joint_task->_use_velocity_saturation_flag = bool(joint_use_velocity_saturation);
    joint_task->setNonIsotropicGains(
        joint_kp_nonisotropic,
        joint_kv_nonisotropic,
        VectorXd::Zero(joint_task->_robot->dof())
    );

    if (joint_dynamic_decoupling_mode == "full")
        joint_task->setDynamicDecouplingFull();
    else if (joint_dynamic_decoupling_mode == "inertia_saturation")
        joint_task->setDynamicDecouplingInertiaSaturation();
    else if (joint_dynamic_decoupling_mode == "none")
        joint_task->setDynamicDecouplingNone();   
}

////////////////// POSORI TASK VARIABLES //////////////////
int posori_use_interpolation;
int posori_use_velocity_saturation;
int posori_use_isotropic_gains;
Eigen::Vector3d posori_euler_angles;
Eigen::Vector2d posori_velocity_saturation;
Eigen::Vector3d posori_kp_nonisotropic;
Eigen::Vector3d posori_kv_nonisotropic;
Eigen::Vector3d posori_ki_nonisotropic;
std::string posori_dynamic_decoupling_mode;

void init_posori_task(Sai2Primitives::PosOriTask *posori_task, RedisClient& redis_client)
{
    Matrix3d initial_orientation;
    Vector3d initial_position;
    Vector3d initial_velocity;

    int dof = posori_task->_robot->dof();
    posori_task->_robot->rotation(initial_orientation, posori_task->_link_name);
    posori_task->_robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
    posori_task->_robot->linearVelocity(initial_velocity, posori_task->_link_name, posori_task->_control_frame.translation());

    // initialize global variables
    posori_use_interpolation = 1;
    posori_use_velocity_saturation = 0;
    posori_use_isotropic_gains = 1;
    posori_velocity_saturation = M_PI / 3.0 * Vector2d::Ones();
    posori_kp_nonisotropic = 100.0 * Vector3d::Ones();
    posori_kv_nonisotropic = 20.0 * Vector3d::Ones();
    posori_ki_nonisotropic = 2.0 * Vector3d::Ones();
    posori_dynamic_decoupling_mode = "inertia_saturation";

    // we are doing ZYX, but we store XYZ
    posori_euler_angles = initial_orientation.eulerAngles(2, 1, 0).reverse();

    // initialize posori_task object
    posori_task->_use_interpolation_flag = bool(posori_use_interpolation);
    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation);
    posori_task->_kp_pos = 100.0;
    posori_task->_kv_pos = 20.0;
    posori_task->_ki_pos = 2.0;
    posori_task->_kp_ori = 100.0;
    posori_task->_kv_ori = 20.0;
    posori_task->_ki_ori = 2.0;
    posori_task->setNonIsotropicGainsPosition(
        Matrix3d::Identity(), 
        posori_kp_nonisotropic,
        posori_kv_nonisotropic, 
        posori_ki_nonisotropic
    );
    posori_task->_use_isotropic_gains_position = bool(posori_use_isotropic_gains);
    posori_task->_use_isotropic_gains_orientation = true;
    posori_task->setDynamicDecouplingInertiaSaturation();

    // prepare redis callback
    redis_client.addIntToRead(POSORI_USE_INTERPOLATION, posori_use_interpolation);
    redis_client.addIntToRead(USE_VEL_SAT_POSORI_KEY, posori_use_velocity_saturation);
    redis_client.addEigenToRead(VEL_SAT_POSORI_KEY, posori_velocity_saturation);
    redis_client.addDoubleToRead(KP_POS_KEY, posori_task->_kp_pos);
    redis_client.addDoubleToRead(KV_POS_KEY, posori_task->_kv_pos);
    redis_client.addDoubleToRead(KI_POS_KEY, posori_task->_ki_pos);
    redis_client.addDoubleToRead(KP_ORI_KEY, posori_task->_kp_ori);
    redis_client.addDoubleToRead(KV_ORI_KEY, posori_task->_kv_ori);
    redis_client.addDoubleToRead(KI_ORI_KEY, posori_task->_ki_ori);
    redis_client.addEigenToRead(KP_NONISOTROPIC_POS_KEY, posori_kp_nonisotropic);
    redis_client.addEigenToRead(KV_NONISOTROPIC_POS_KEY, posori_kv_nonisotropic);
    redis_client.addEigenToRead(KI_NONISOTROPIC_POS_KEY, posori_ki_nonisotropic);
    redis_client.addIntToRead(USE_ISOTROPIC_POS_GAINS_KEY, posori_use_isotropic_gains);
    redis_client.addStringToRead(DYN_DEC_POSORI_KEY, posori_dynamic_decoupling_mode);
    redis_client.addEigenToRead(DESIRED_POS_KEY, posori_task->_desired_position); 
    redis_client.addEigenToRead(DESIRED_ORI_KEY, posori_euler_angles);
    redis_client.addEigenToRead(DESIRED_VEL_KEY, posori_task->_desired_velocity);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addIntToWrite(POSORI_USE_INTERPOLATION, posori_use_interpolation);
    redis_client.addIntToWrite(USE_VEL_SAT_POSORI_KEY, posori_use_velocity_saturation);
    redis_client.addEigenToWrite(VEL_SAT_POSORI_KEY, posori_velocity_saturation);
    redis_client.addDoubleToWrite(KP_POS_KEY, posori_task->_kp_pos);
    redis_client.addDoubleToWrite(KV_POS_KEY, posori_task->_kv_pos);
    redis_client.addDoubleToWrite(KI_POS_KEY, posori_task->_ki_pos);
    redis_client.addDoubleToWrite(KP_ORI_KEY, posori_task->_kp_ori);
    redis_client.addDoubleToWrite(KV_ORI_KEY, posori_task->_kv_ori);
    redis_client.addDoubleToWrite(KI_ORI_KEY, posori_task->_ki_ori);
    redis_client.addEigenToWrite(KP_NONISOTROPIC_POS_KEY, posori_kp_nonisotropic);
    redis_client.addEigenToWrite(KV_NONISOTROPIC_POS_KEY, posori_kv_nonisotropic);
    redis_client.addEigenToWrite(KI_NONISOTROPIC_POS_KEY, posori_ki_nonisotropic);
    redis_client.addIntToWrite(USE_ISOTROPIC_POS_GAINS_KEY, posori_use_isotropic_gains);
    redis_client.addStringToWrite(DYN_DEC_POSORI_KEY, posori_dynamic_decoupling_mode);
    redis_client.addEigenToWrite(DESIRED_POS_KEY, posori_task->_desired_position); 
    redis_client.addEigenToWrite(DESIRED_ORI_KEY, posori_euler_angles);
    redis_client.addEigenToWrite(DESIRED_VEL_KEY, posori_task->_desired_velocity);
}

void update_posori_task(Sai2Primitives::PosOriTask *posori_task)
{
    posori_task->setNonIsotropicGainsPosition(
        Matrix3d::Identity(), 
        posori_kp_nonisotropic,
        posori_kv_nonisotropic, 
        posori_ki_nonisotropic
    );

    if (posori_use_interpolation && !posori_task->_use_interpolation_flag)
        posori_task->reInitializeTask();

    posori_task->_use_interpolation_flag = bool(posori_use_interpolation);
    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation);
    posori_task->_use_isotropic_gains_position = bool(posori_use_isotropic_gains);
    posori_task->_linear_saturation_velocity = posori_velocity_saturation(0);
    posori_task->_angular_saturation_velocity = posori_velocity_saturation(1);


    Matrix3d desired_rmat;
    desired_rmat = Eigen::AngleAxisd(posori_euler_angles(2), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(posori_euler_angles(1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(posori_euler_angles(0), Eigen::Vector3d::UnitX());
    posori_task->_desired_orientation = desired_rmat;

    if (posori_dynamic_decoupling_mode == "full")
        posori_task->setDynamicDecouplingFull();
    else if (posori_dynamic_decoupling_mode == "partial")
        posori_task->setDynamicDecouplingPartial();
    else if (posori_dynamic_decoupling_mode == "inertia_saturation")
        posori_task->setDynamicDecouplingInertiaSaturation();
    else if (posori_dynamic_decoupling_mode == "none")
        posori_task->setDynamicDecouplingNone();
}

int main(int argc, char **argv) 
{
    // open redis connection
    redis_client.connect();

    // set up signal handlers
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // initialize controller state
    redis_client.set(PRIMITIVE_KEY, currentPrimitive);

    // notify UI that we are initializing
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZING);

    // load robots & bind joint angles/velocity to redis
    auto robot = new Sai2Model::Sai2Model(ROBOT_FILE, false);
    redis_client.addEigenToRead(JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToRead(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.readAllSetupValues();
    robot->updateModel();

    // bind current state to what redis says
    redis_client.addStringToRead(PRIMITIVE_KEY, currentPrimitive);

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    VectorXd posori_task_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);

    const string link_name = "end-effector";
    const Vector3d pos_in_link = Vector3d(0.07, 0.0, 0.0);

    // initialize tasks
    Sai2Primitives::PosOriTask *posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
    init_posori_task(posori_task, redis_client);

    Sai2Primitives::JointTask *joint_task = new Sai2Primitives::JointTask(robot);
    init_joint_task(joint_task, redis_client);

    redis_client.writeAllSetupValues();

    // initialization complete
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZED);

    redis_client.set(UI_FORCE_ENABLED_KEY, "0");
    
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

        string oldPrimitive = currentPrimitive;

        // update all values tied to redis
        redis_client.readAllSetupValues();
        update_joint_task(joint_task);
        update_posori_task(posori_task);

        robot->updateModel();
        robot->coriolisForce(coriolis);

        N_prec.setIdentity(dof, dof);

        // if we just changed primitives, reset & reinit
        if (currentPrimitive != oldPrimitive)
        {
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                joint_task->_current_position = robot->_q;
                joint_task->reInitializeTask();
                redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q); 
                redis_client.set(UI_FORCE_ENABLED_KEY, "0");
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
            {
                posori_task->reInitializeTask();
                redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_current_position);

                // ZYX euler angles, but stored as XYZ
                Vector3d angles = posori_task->_current_orientation.eulerAngles(2, 1, 0).reverse();
                redis_client.setEigenMatrixJSON(DESIRED_ORI_KEY, angles);
                redis_client.set(UI_FORCE_ENABLED_KEY, "0");
            }
            else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
            {
                redis_client.set(UI_FORCE_ENABLED_KEY, "1");
            }
        }

        // normal operation for each task
        else if (currentPrimitive == PRIMITIVE_JOINT_TASK)
        {
            joint_task->updateTaskModel(N_prec);
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
        }

        else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
        {
            posori_task->updateTaskModel(N_prec);
            
#ifdef USING_OTG
            // disable OTG for trajectory task 
            if (currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
                redis_client.set(POSORI_USE_INTERPOLATION, "0");
#endif
            posori_task->_desired_angular_velocity.setZero();

            // compute torques
            posori_task->computeTorques(posori_task_torques);
            command_torques = posori_task_torques + coriolis;
        }
        else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
        {
            command_torques.setZero(dof);
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
            cout << "current primitive: " << currentPrimitive << endl;
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                cout << time << endl;
                cout << "desired position : " << joint_task->_desired_position.transpose() << endl;
                cout << "current position : " << joint_task->_current_position.transpose() << endl;
                cout << "position error : " << (joint_task->_desired_position - joint_task->_current_position).norm() << endl;
                cout << endl;
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
            {
                cout << time << endl;
                cout << "desired position : " << posori_task->_desired_position.transpose() << endl;
                cout << "current position : " << posori_task->_current_position.transpose() << endl;
                cout << "position error : " << (posori_task->_desired_position - posori_task->_current_position).norm() << endl;
                cout << endl;
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

    if (robot)
        delete robot;
    if (joint_task)
        delete joint_task;
    if (posori_task)
        delete posori_task;
    return 0;
}
