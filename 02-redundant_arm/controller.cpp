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

const bool flag_simulation = true;
// const bool flag_simulation = false;

// redis keys
string JOINT_ANGLES_KEY = "sai2::examples::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::examples::sensors::dq";
string JOINT_TORQUES_COMMANDED_KEY = "sai2::examples::actuators::fgc";

string MASSMATRIX_KEY;
string ROBOT_GRAVITY_KEY;
string CORIOLIS_KEY;


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
const string DESIRED_VEL_KEY = "sai2::examples::desired_velocity";
const string KP_POS_KEY = "sai2::examples::kp_pos";
const string KV_POS_KEY = "sai2::examples::kv_pos";
const string KI_POS_KEY = "sai2::examples::ki_pos";
const string KP_ORI_KEY = "sai2::examples::kp_ori";
const string KV_ORI_KEY = "sai2::examples::kv_ori";
const string KI_ORI_KEY = "sai2::examples::ki_ori";
const string KP_NONISOTROPIC_POS_KEY = "sai2::examples::kp_nonisotropic_pos";
const string KV_NONISOTROPIC_POS_KEY = "sai2::examples::kv_nonisotropic_pos";
const string KI_NONISOTROPIC_POS_KEY = "sai2::examples::ki_nonisotropic_pos";
const string KP_NONISOTROPIC_ORI_KEY = "sai2::examples::kp_nonisotropic_ori";
const string KV_NONISOTROPIC_ORI_KEY = "sai2::examples::kv_nonisotropic_ori";
const string KI_NONISOTROPIC_ORI_KEY = "sai2::examples::ki_nonisotropic_ori";
const string USE_ISOTROPIC_POS_GAINS_KEY = "sai2::examples::use_isotropic_pos_gains";
const string USE_ISOTROPIC_ORI_GAINS_KEY = "sai2::examples::use_isotropic_ori_gains";
const string POSORI_USE_INTERPOLATION = "sai2::examples::posori_use_interpolation";
const string USE_VEL_SAT_POSORI_KEY = "sai2::examples::use_posori_velocity_saturation";
const string VEL_SAT_POSORI_KEY = "sai2::examples::posori_velocity_saturation";
const string DYN_DEC_POSORI_KEY = "sai2::examples::posori_dynamic_decoupling";

// joint task parameters
const string DESIRED_JOINT_POS_KEY = "sai2::examples::desired_joint_position";
const string KP_JOINT_KEY = "sai2::examples::kp_joint";
const string KV_JOINT_KEY = "sai2::examples::kv_joint";
const string KP_NON_ISOTROPIC_JOINT_KEY = "sai2::examples::kp_nonisotropic_joint";
const string KV_NON_ISOTROPIC_JOINT_KEY = "sai2::examples::kv_nonisotropic_joint";
const string USE_ISOTROPIC_JOINT_GAINS_KEY = "sai2::examples::use_isotropic_joint_gains";
const string JOINT_USE_INTERPOLATION = "sai2::examples::joint_use_interpolation";
const string USE_VEL_SAT_JOINT_KEY = "sai2::examples::use_joint_velocity_saturation";
const string VEL_SAT_JOINT_KEY = "sai2::examples::joint_velocity_saturation";
const string DYN_DEC_JOINT_KEY = "sai2::examples::joint_dynamic_decoupling";

// robot file
const string ROBOT_FILE = "resources/panda_arm.urdf";

// state-related keys
const string PRIMITIVE_KEY = "sai2::examples::primitive";
const string PRIMITIVE_JOINT_TASK = "primitive_joint_task";
const string PRIMITIVE_POSORI_TASK = "primitive_posori_task";
const string PRIMITIVE_TRAJECTORY_TASK = "primitive_trajectory_task";
const string PRIMITIVE_FLOATING_TASK = "primitive_floating_task";

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
        KP_NONISOTROPIC_POS_KEY,
        KV_NONISOTROPIC_POS_KEY,
        KI_NONISOTROPIC_POS_KEY,
        KP_NONISOTROPIC_ORI_KEY,
        KV_NONISOTROPIC_ORI_KEY,
        KI_NONISOTROPIC_ORI_KEY,
        USE_ISOTROPIC_POS_GAINS_KEY,
        USE_ISOTROPIC_ORI_GAINS_KEY,
#ifdef USING_OTG
        USE_VEL_SAT_POSORI_KEY,
        VEL_SAT_POSORI_KEY,
        DYN_DEC_POSORI_KEY,
        POSORI_USE_INTERPOLATION, // KEEP THIS LAST 
#endif
    };

    auto identity = Matrix3d::Identity();
    auto key_values = redis_client.mget(query_keys);
    posori_task->setIsotropicGainsPosition(
        std::stod(key_values[0]),
        std::stod(key_values[1]),
        std::stod(key_values[2])
    );
    posori_task->setIsotropicGainsOrientation(
        std::stod(key_values[3]),
        std::stod(key_values[4]),
        std::stod(key_values[5])
    );
    
    posori_task->setNonIsotropicGainsPosition(
        identity,
        redis_client.decodeEigenMatrixJSON(key_values[6]).col(0),
        redis_client.decodeEigenMatrixJSON(key_values[7]).col(0),
        redis_client.decodeEigenMatrixJSON(key_values[8]).col(0)
    );

    posori_task->setNonIsotropicGainsOrientation(
        identity,
        redis_client.decodeEigenMatrixJSON(key_values[9]).col(0),
        redis_client.decodeEigenMatrixJSON(key_values[10]).col(0),
        redis_client.decodeEigenMatrixJSON(key_values[11]).col(0)
    );

    bool use_isotropic_pos_gains = static_cast<bool>(std::stoi(key_values[12]));
    bool use_isotropic_ori_gains = static_cast<bool>(std::stoi(key_values[13]));

    posori_task->_use_isotropic_gains_position = use_isotropic_pos_gains;
    posori_task->_use_isotropic_gains_orientation = use_isotropic_ori_gains;

#ifdef USING_OTG
    posori_task->_use_velocity_saturation_flag = static_cast<bool>(
        std::stoi(key_values[14])
    );
    auto velocity_sat = redis_client.decodeEigenMatrixJSON(key_values[15]);
	posori_task->_linear_saturation_velocity = velocity_sat(0, 0);
	posori_task->_angular_saturation_velocity = velocity_sat(1, 0);

    auto posori_dynamic_decoupling = key_values[16];
    if (posori_dynamic_decoupling == "full")
        posori_task->setDynamicDecouplingFull();
    else if (posori_dynamic_decoupling == "partial")
        posori_task->setDynamicDecouplingPartial();
    else if (posori_dynamic_decoupling == "inertia_saturation")
        posori_task->setDynamicDecouplingInertiaSaturation();
    else if (posori_dynamic_decoupling == "none")
        posori_task->setDynamicDecouplingNone();

    bool new_posori_interpolation_flag = static_cast<bool>(std::stoi(key_values.back()));
    if (new_posori_interpolation_flag && !posori_task->_use_interpolation_flag)
    {
        posori_task->reInitializeTask();
    }

    posori_task->_use_interpolation_flag = new_posori_interpolation_flag;
#endif
}

void init_posori_task(
    Sai2Primitives::PosOriTask *posori_task,
    Sai2Model::Sai2Model *robot,
    RedisClient& redis_client,
    Matrix3d &initial_orientation,
    Vector3d &initial_position,
    Vector3d &initial_velocity) 
{
#ifdef USING_OTG
    posori_task->_use_interpolation_flag = true;
    posori_task->_use_velocity_saturation_flag = false;
#endif
    posori_task->_kp_pos = 50.0;
    posori_task->_kv_pos = 12.0;
    posori_task->_ki_pos = 0.0;
    posori_task->_kp_ori = 50.0;
    posori_task->_kv_ori = 12.0;
    posori_task->_ki_ori = 0.0;
    posori_task->_use_isotropic_gains_position = true;
    posori_task->_use_isotropic_gains_orientation = true;
    posori_task->setDynamicDecouplingInertiaSaturation();

#ifdef USING_OTG
    redis_client.set(POSORI_USE_INTERPOLATION, std::to_string(static_cast<int>(posori_task->_use_interpolation_flag)));
    redis_client.set(USE_VEL_SAT_POSORI_KEY, "0");
    redis_client.setEigenMatrixJSON(VEL_SAT_POSORI_KEY, M_PI / 3.0 * Vector2d::Ones());
#endif
    redis_client.set(KP_POS_KEY, std::to_string(posori_task->_kp_pos));
    redis_client.set(KV_POS_KEY, std::to_string(posori_task->_kv_pos));
    redis_client.set(KI_POS_KEY, std::to_string(posori_task->_ki_pos));
    redis_client.set(KP_ORI_KEY, std::to_string(posori_task->_kp_ori));
    redis_client.set(KV_ORI_KEY, std::to_string(posori_task->_kv_ori));
    redis_client.set(KI_ORI_KEY, std::to_string(posori_task->_ki_ori));

    redis_client.setEigenMatrixJSON(KP_NONISOTROPIC_POS_KEY, 50.0 * Vector3d::Ones());
    redis_client.setEigenMatrixJSON(KV_NONISOTROPIC_POS_KEY, 12.0 * Vector3d::Ones());
    redis_client.setEigenMatrixJSON(KI_NONISOTROPIC_POS_KEY, Vector3d::Zero());
    redis_client.setEigenMatrixJSON(KP_NONISOTROPIC_ORI_KEY, 50.0 * Vector3d::Ones());
    redis_client.setEigenMatrixJSON(KV_NONISOTROPIC_ORI_KEY, 12.0 * Vector3d::Ones());
    redis_client.setEigenMatrixJSON(KI_NONISOTROPIC_ORI_KEY, Vector3d::Zero());

    redis_client.set(USE_ISOTROPIC_POS_GAINS_KEY, "1");
    redis_client.set(USE_ISOTROPIC_ORI_GAINS_KEY, "1");
    redis_client.set(DYN_DEC_POSORI_KEY, "inertia_saturation");

    robot->rotation(initial_orientation, posori_task->_link_name);
    robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
    robot->linearVelocity(initial_velocity, posori_task->_link_name, posori_task->_control_frame.translation());
    redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, initial_position);

    // we are doing ZYX, but we store XYZ
    Vector3d initial_euler_angles = initial_orientation.eulerAngles(2, 1, 0).reverse();

    redis_client.setEigenMatrixJSON(DESIRED_ORI_KEY, initial_euler_angles);
    redis_client.setEigenMatrixJSON(DESIRED_VEL_KEY, Vector3d::Zero());
}

void read_joint_parameters(
    Sai2Primitives::JointTask *joint_task,
    RedisClient& redis_client)
{
    const std::vector<std::string> redis_query_keys {
        KP_JOINT_KEY,
        KV_JOINT_KEY,
        KP_NON_ISOTROPIC_JOINT_KEY,
        KV_NON_ISOTROPIC_JOINT_KEY,
        USE_ISOTROPIC_JOINT_GAINS_KEY,
        USE_VEL_SAT_JOINT_KEY,
        VEL_SAT_JOINT_KEY,
#ifdef USING_OTG
        DYN_DEC_JOINT_KEY,
        JOINT_USE_INTERPOLATION, // KEEP THIS LAST 
#endif
    };

    auto key_values = redis_client.mget(redis_query_keys);
    joint_task->setIsotropicGains(
        std::stod(key_values[0]),
        std::stod(key_values[1]),
        0.0
    );

    auto raw_kp_non_isotropic = redis_client.decodeEigenMatrixJSON(key_values[2]);
    auto raw_kv_non_isotropic = redis_client.decodeEigenMatrixJSON(key_values[3]);
    joint_task->setNonIsotropicGains(
        raw_kp_non_isotropic.col(0),
        raw_kv_non_isotropic.col(0),
        VectorXd::Zero(joint_task->_robot->dof())
    );

    joint_task->_use_isotropic_gains = static_cast<bool>(std::stoi(key_values[4]));
    joint_task->_use_velocity_saturation_flag = static_cast<bool>(std::stoi(key_values[5]));
    joint_task->_saturation_velocity = redis_client.decodeEigenMatrixJSON(key_values[6]);

    auto joint_dynamic_decoupling = key_values[7];
    if (joint_dynamic_decoupling == "full")
        joint_task->setDynamicDecouplingFull();
    else if (joint_dynamic_decoupling == "inertia_saturation")
        joint_task->setDynamicDecouplingInertiaSaturation();
    else if (joint_dynamic_decoupling == "none")
        joint_task->setDynamicDecouplingNone();

#ifdef USING_OTG
    bool new_joint_interpolation_flag = static_cast<bool>(std::stoi(key_values.back()));
    if(new_joint_interpolation_flag && ! joint_task->_use_interpolation_flag)
    {
        joint_task->reInitializeTask();
        joint_task->_use_interpolation_flag = true;
    }
    joint_task->_use_interpolation_flag = new_joint_interpolation_flag;
#endif
}

void init_joint_task(
    Sai2Primitives::JointTask *joint_task,
    Sai2Model::Sai2Model *robot,
    RedisClient& redis_client)
{
    joint_task->_kp = 100.;
    joint_task->_kv = 20.;

    auto kp_init = 100.0 * VectorXd::Ones(robot->dof());
    auto kv_init = 20.0 * VectorXd::Ones(robot->dof());

    joint_task->setNonIsotropicGains(
        kp_init,
        kv_init,
        VectorXd::Zero(robot->dof())
    );
    joint_task->_use_isotropic_gains = true;
    joint_task->_use_velocity_saturation_flag = false;
    joint_task->setDynamicDecouplingInertiaSaturation();

#ifdef USING_OTG
    joint_task->_use_interpolation_flag = true;
#endif

    redis_client.set(KP_JOINT_KEY, std::to_string(joint_task->_kp));
    redis_client.set(KV_JOINT_KEY, std::to_string(joint_task->_kv));
    redis_client.setEigenMatrixJSON(KP_NON_ISOTROPIC_JOINT_KEY, kp_init);
    redis_client.setEigenMatrixJSON(KV_NON_ISOTROPIC_JOINT_KEY, kv_init);
    redis_client.set(USE_ISOTROPIC_JOINT_GAINS_KEY, "1");
    redis_client.set(USE_VEL_SAT_JOINT_KEY, "0");
    redis_client.set(DYN_DEC_JOINT_KEY, "inertia_saturation");
   
#ifdef USING_OTG
    redis_client.set(JOINT_USE_INTERPOLATION, "1");
#endif

    redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q);
    redis_client.setEigenMatrixJSON(VEL_SAT_JOINT_KEY, joint_task->_saturation_velocity);
}

int main(int argc, char **argv) 
{
    if (!flag_simulation)
    {
        JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
        JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
        JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
        MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
        CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
        ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::robot_gravity";  
    }

    // open redis
    auto redis_client = RedisClient();
    redis_client.connect();

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

    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
    robot->updateModel();

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    VectorXd posori_task_torques = VectorXd::Zero(dof);

    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    VectorXd coriolis = VectorXd::Zero(dof);

    const string link_name = "link7";
    const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.12);

    // initialize tasks
    Matrix3d initial_orientation;
    Vector3d initial_position;
    Vector3d initial_velocity;
    Sai2Primitives::PosOriTask *posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
    init_posori_task(posori_task, robot, redis_client, initial_orientation, initial_position, initial_velocity);

    Sai2Primitives::JointTask *joint_task = new Sai2Primitives::JointTask(robot);
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

        if(flag_simulation)
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

        MatrixXd N_prec = MatrixXd::Identity(dof, dof);

        // read the current state
        string interfacePrimitive = redis_client.get(PRIMITIVE_KEY);

        // if we just changed primitives, reset & reinit
        if (currentPrimitive != interfacePrimitive)
        {
            if (interfacePrimitive == PRIMITIVE_JOINT_TASK)
            {
                joint_task->_current_position = robot->_q;
                joint_task->reInitializeTask();
                redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q); 
            }
            else if (interfacePrimitive == PRIMITIVE_POSORI_TASK)
            {
                posori_task->reInitializeTask();
                redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_current_position);

                // ZYX euler angles, but stored as XYZ
                Vector3d angles = posori_task->_current_orientation.eulerAngles(2, 1, 0).reverse();
                redis_client.setEigenMatrixJSON(DESIRED_ORI_KEY, angles);
            }
        }

        // steady-state operations for each task
        else if (interfacePrimitive == PRIMITIVE_JOINT_TASK)
        {
            joint_task->updateTaskModel(N_prec);
            read_joint_parameters(joint_task, redis_client);
            joint_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_JOINT_POS_KEY);
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
        }
        else if (interfacePrimitive == PRIMITIVE_POSORI_TASK || interfacePrimitive == PRIMITIVE_TRAJECTORY_TASK)
        {
            posori_task->updateTaskModel(N_prec);
            N_prec = posori_task->_N;
            joint_task->updateTaskModel(N_prec);

            read_posori_parameters(posori_task, redis_client);
            posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POS_KEY);
            posori_task->_desired_velocity = redis_client.getEigenMatrixJSON(DESIRED_VEL_KEY);
            
#ifdef USING_OTG
            // disable OTG for trajectory task 
            if (interfacePrimitive == PRIMITIVE_TRAJECTORY_TASK)
                redis_client.set(POSORI_USE_INTERPOLATION, "0");
#endif
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
            posori_task->computeTorques(posori_task_torques);
            joint_task->computeTorques(joint_task_torques);
            command_torques = posori_task_torques + joint_task_torques + coriolis;
        }
        else if (interfacePrimitive == PRIMITIVE_FLOATING_TASK)
        {
            command_torques = VectorXd::Zero(robot->dof());
        }
        currentPrimitive = interfacePrimitive;
    
        // -------------------------------------------
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
    return 0;
}
