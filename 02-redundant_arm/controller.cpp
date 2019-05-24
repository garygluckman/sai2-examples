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


using namespace std;
using namespace Eigen;


// redis keys
// redis keys
const string JOINT_ANGLES_KEY = "sai2::examples::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::examples::sensors::dq";
const string JOINT_TORQUES_COMMANDED_KEY = "sai2::examples::actuators::fgc";

const string CURRENT_EE_POS_KEY = "sai2::examples::current_ee_pos";
const string CURRENT_EE_VEL_KEY = "sai2::examples::current_ee_vel";

// controller initialization
const string CONTROL_STATE_KEY = "sai2::examples::control_state";
const string CONTROL_STATE_INITIALIZING = "initializing";
const string CONTROL_STATE_INITIALIZED = "initialized";
const string CONTROL_STATE_READY = "ready";

// posori task parameters
const string DESIRED_POS_KEY = "sai2::examples::desired_position";
const string DESIRED_ORI_KEY = "sai2::examples::desired_orientation";
const string KP_POS_KEY = "sai2::examples::kp_pos";
const string KV_POS_KEY = "sai2::examples::kv_pos";
const string KI_POS_KEY = "sai2::examples::ki_pos";
const string KP_ORI_KEY = "sai2::examples::kp_ori";
const string KV_ORI_KEY = "sai2::examples::kv_ori";
const string KI_ORI_KEY = "sai2::examples::ki_ori";
const string POSORI_USE_INTERPOLATION = "sai2::examples::posori_use_interpolation";

// joint task parameters
const string DESIRED_JOINT_POS_KEY = "sai2::examples::desired_joint_position";
const string KP_JOINT_KEY = "sai2::examples::kp_joint";
const string KV_JOINT_KEY = "sai2::examples::kv_joint";
const string JOINT_USE_INTERPOLATION = "sai2::examples::joint_use_interpolation";

// robot file
const string ROBOT_FILE = "resources/kuka_iiwa.urdf";

// state-related keys
const string PRIMITIVE_KEY = "sai2::examples::primitive";
const string PRIMITIVE_JOINT_TASK = "primitive_joint_task";
const string PRIMITIVE_POSORI_TASK = "primitive_posori_task";


////////////////// GLOBAL VARIABLES //////////////////
bool runloop = false;
string currentPrimitive = PRIMITIVE_JOINT_TASK;

////////////////////// FUNCTIONS //////////////////////
void sighandler(int)
{
    runloop = false;
}

void read_posori_parameters(
    Sai2Primitives::PosOriTask *posori_task,
    RedisClient &redis_client)
{
    const std::vector<std::string> query_keys {
        KP_POS_KEY,
        KV_POS_KEY,
        KI_POS_KEY,
        KP_ORI_KEY,
        KV_ORI_KEY,
        KI_ORI_KEY,
#ifdef USING_OTG
        POSORI_USE_INTERPOLATION, // KEEP THIS LAST 
#endif
    };

    auto key_values = redis_client.mget(query_keys);
    posori_task->_kp_pos = std::stod(key_values[0]);
    posori_task->_kv_pos = std::stod(key_values[1]);
    posori_task->_ki_pos = std::stod(key_values[2]);
    posori_task->_kp_ori = std::stod(key_values[3]);
    posori_task->_kv_ori = std::stod(key_values[4]);
    posori_task->_ki_ori = std::stod(key_values[5]);
#ifdef USING_OTG
    posori_task->_use_interpolation_flag = static_cast<bool>(
        std::stoi(key_values.back())
    );
#endif
}

void init_posori_task(
    Sai2Primitives::PosOriTask *posori_task,
    Sai2Model::Sai2Model *robot,
    RedisClient& redis_client,
    Matrix3d &initial_orientation,
    Vector3d &initial_position) 
{
#ifdef USING_OTG
    posori_task->_use_interpolation_flag = true;
#endif
    posori_task->_kp_pos = 100.0;
    posori_task->_kv_pos = 20.0;
    posori_task->_ki_pos = 2.0;
    posori_task->_kp_ori = 100.0;
    posori_task->_kv_ori = 20.0;
    posori_task->_ki_ori = 2.0;

    redis_client.set(KP_POS_KEY, std::to_string(posori_task->_kp_pos));
    redis_client.set(KV_POS_KEY, std::to_string(posori_task->_kv_pos));
    redis_client.set(KI_POS_KEY, std::to_string(posori_task->_ki_pos));
    redis_client.set(KP_ORI_KEY, std::to_string(posori_task->_kp_ori));
    redis_client.set(KV_ORI_KEY, std::to_string(posori_task->_kv_ori));
    redis_client.set(KI_ORI_KEY, std::to_string(posori_task->_ki_ori));

    robot->rotation(initial_orientation, posori_task->_link_name);
    robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
    redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, initial_position); 
    redis_client.setEigenMatrixJSON(DESIRED_ORI_KEY, Vector3d::Zero());
}

void read_joint_parameters(
    Sai2Primitives::JointTask *joint_task,
    RedisClient& redis_client)
{
    const std::vector<std::string> redis_query_keys {
        KP_JOINT_KEY,
        KV_JOINT_KEY,
#ifdef USING_OTG
        JOINT_USE_INTERPOLATION // KEEP THIS LAST
#endif
    };

    auto key_values = redis_client.mget(redis_query_keys);
    joint_task->_kp = std::stod(key_values[0]);
    joint_task->_kv = std::stod(key_values[1]);
#ifdef USING_OTG
    joint_task->_use_interpolation_flag = static_cast<bool>(
        std::stoi(key_values.back())
    );
#endif
}

void init_joint_task(
    Sai2Primitives::JointTask *joint_task,
    Sai2Model::Sai2Model *robot,
    RedisClient& redis_client)
{
    joint_task->_kp = 50.;
    joint_task->_kv = 14.;
#ifdef USING_OTG
    joint_task->_use_interpolation_flag = true;
#endif
    redis_client.set(KP_JOINT_KEY, std::to_string(joint_task->_kp));
    redis_client.set(KV_JOINT_KEY, std::to_string(joint_task->_kv));
#ifdef USING_OTG
    redis_client.set(JOINT_USE_INTERPOLATION, std::to_string(static_cast<int>(
        joint_task->_use_interpolation_flag
    )));
#endif
    redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q); 
}

int main(int argc, char **argv)
{
    // create redis client
    auto redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // notify UI that we are initializing
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZING);

    // load robots
    auto robot = new Sai2Model::Sai2Model(ROBOT_FILE, false);

    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
    robot->updateModel();

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    VectorXd coriolis = VectorXd::Zero(dof);

    const string link_name = "link6";
    const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0);

    // initialize tasks
    Matrix3d initial_orientation;
    Vector3d initial_position;
    auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
    init_posori_task(posori_task, robot, redis_client, initial_orientation, initial_position);

    // joint task
    auto joint_task = new Sai2Primitives::JointTask(robot);
    init_joint_task(joint_task, robot, redis_client);

    // initialization complete
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZED);

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

        // read joint positions, velocities, update model
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        robot->updateModel();
        robot->coriolisForce(coriolis);

        N_prec = MatrixXd::Identity(dof, dof);

        // read current state
        string interfacePrimitive = redis_client.get(PRIMITIVE_KEY);

        if (interfacePrimitive == PRIMITIVE_JOINT_TASK)
        {
            if (currentPrimitive != interfacePrimitive)
                joint_task->reInitializeTask();

            joint_task->updateTaskModel(N_prec);

            read_joint_parameters(joint_task, redis_client);
            joint_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_JOINT_POS_KEY);
            joint_task->computeTorques(command_torques);
        }
        else if (interfacePrimitive == PRIMITIVE_POSORI_TASK)
        {
            if (currentPrimitive != interfacePrimitive)
                posori_task->reInitializeTask();

            posori_task->updateTaskModel(N_prec);
            N_prec = posori_task->_N;

            read_posori_parameters(posori_task, redis_client);
            posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);
            
            // interface specifies fixed angles, convert to rot matrix
            MatrixXd desired_fixed = redis_client.getEigenMatrixJSON(DESIRED_ORI_KEY);
            Matrix3d desired_rmat;
            desired_rmat = Eigen::AngleAxisd(desired_fixed(2), Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(desired_fixed(1), Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(desired_fixed(0), Eigen::Vector3d::UnitX());
            posori_task->_desired_orientation = desired_rmat;

            // we also need to read linear & angular velocity
            posori_task->_desired_angular_velocity = Vector3d::Zero();

            // compute torques
            posori_task->computeTorques(command_torques);
        }
        currentPrimitive = interfacePrimitive;

        // -------------------------------------------
        command_torques = command_torques + coriolis;
        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

        // log current EE position and velocity to redis
        Vector3d current_pos = Vector3d::Zero();
        robot->position(current_pos, link_name, pos_in_link);

        Vector3d current_vel = Vector3d::Zero();
        robot->linearVelocity(current_vel, link_name, pos_in_link);

        redis_client.setEigenMatrixJSON(CURRENT_EE_POS_KEY, current_pos);
        redis_client.setEigenMatrixJSON(CURRENT_EE_VEL_KEY, current_vel);
        
        // -------------------------------------------
        if (controller_counter % 500 == 0)
        {
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                cout << time << endl;
                cout << "desired position : " << joint_task->_desired_position.transpose() << endl;
                cout << "current position : " << joint_task->_current_position.transpose() << endl;
                cout << "position error : " << (joint_task->_desired_position - joint_task->_current_position).norm() << endl;
                cout << endl;
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK)
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
    return 0;
}
