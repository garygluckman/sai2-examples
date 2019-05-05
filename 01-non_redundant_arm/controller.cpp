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
const string JOINT_ANGLES_KEY = "sai2::examples::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::examples::sensors::dq";
const string JOINT_TORQUES_COMMANDED_KEY = "sai2::examples::actuators::fgc";

// controller initialization
const string CONTROL_STATE_KEY = "sai2::examples::control_state";
const string CONTROL_STATE_INITIALIZING = "initializing";
const string CONTROL_STATE_INITIALIZED = "initialized";
const string CONTROL_STATE_READY = "ready";

// operational space control
const string DESIRED_POS_KEY = "sai2::examples::desired_position";
const string DESIRED_ORI_KEY = "sai2::examples::desired_orientation";
const string KP_POS_KEY = "sai2::examples::kp_pos";
const string KV_POS_KEY = "sai2::examples::kv_pos";
const string KI_POS_KEY = "sai2::examples::ki_pos";
const string KP_ORI_KEY = "sai2::examples::kp_ori";
const string KV_ORI_KEY = "sai2::examples::kv_ori";
const string KI_ORI_KEY = "sai2::examples::ki_ori";
const string POSORI_USE_INTERPOLATION = "sai2::examples::posori_use_interpolation";

// joint space control
const string DESIRED_JOINT_POS_KEY = "sai2::examples::desired_joint_position";
const string KP_JOINT_KEY = "sai2::examples::kp_joint";
const string KV_JOINT_KEY = "sai2::examples::kv_joint";

// robot file
const string ROBOT_FILE = "resources/puma.urdf";

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


void init_posori_task(
	Sai2Primitives::PosOriTask *posori_task,
	Sai2Model::Sai2Model *robot,
	RedisClient& redis_client,
	bool read_from_redis) 
{
	if (read_from_redis)
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
	else
	{
#ifdef USING_OTG
		posori_task->_use_interpolation_flag = false;
#endif
		posori_task->_kp_pos = 100.0;
		posori_task->_kv_pos = 20.0;
		posori_task->_ki_pos = 2.0;
		posori_task->_kp_ori = 100.0;
		posori_task->_kv_ori = 20.0;
		posori_task->_ki_ori = 2.0;

		redis_client.set(KP_POS_KEY, std::to_string(posori_task->_kp_pos));
		redis_client.set(KV_POS_KEY, std::to_string(posori_task->_kp_pos));
		redis_client.set(KI_POS_KEY, std::to_string(posori_task->_kp_pos));
		redis_client.set(KP_ORI_KEY, std::to_string(posori_task->_kp_pos));
		redis_client.set(KV_ORI_KEY, std::to_string(posori_task->_kp_pos));
		redis_client.set(KI_ORI_KEY, std::to_string(posori_task->_kp_pos));
	}

	Matrix3d initial_orientation;
	Vector3d initial_position;
	robot->rotation(initial_orientation, posori_task->_link_name);
	robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
	redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, initial_position); 
}

void init_joint_task(
	Sai2Primitives::JointTask *joint_task,
	Sai2Model::Sai2Model *robot,
	RedisClient& redis_client,
	bool read_from_redis)
{
	if (read_from_redis)
	{
		const std::vector<std::string> redis_query_keys {
			KP_JOINT_KEY,
			KV_JOINT_KEY
		};

		auto key_values = redis_client.mget(redis_query_keys);
		joint_task->_kp = std::stod(key_values[0]);
		joint_task->_kv = std::stod(key_values[1]);
	}
	else
	{
		joint_task->_kp = 100.;
		joint_task->_kv = 20.;
		redis_client.set(KP_JOINT_KEY, std::to_string(joint_task->_kp));
		redis_client.set(KV_JOINT_KEY, std::to_string(joint_task->_kv));
	}

	redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q); 
}

int main(int argc, char **argv) 
{
	// open redis
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handlers
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

	const string link_name = "end-effector";
	const Vector3d pos_in_link = Vector3d(0.07,0.0,0.0);

	// initialize tasks
	Sai2Primitives::PosOriTask *posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	init_posori_task(posori_task, robot, redis_client, false);

	Sai2Primitives::JointTask *joint_task = new Sai2Primitives::JointTask(robot);
	init_joint_task(joint_task, robot, redis_client, false);

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

		// read the current state
		MatrixXd N_prec = MatrixXd::Identity(dof, dof);
		
		string interfacePrimitive = redis_client.get(PRIMITIVE_KEY);
		if (interfacePrimitive == PRIMITIVE_JOINT_TASK)
		{
			if (currentPrimitive != interfacePrimitive)
			{
				init_joint_task(joint_task, robot, redis_client, true);
				joint_task->reInitializeTask();
			}
				
			joint_task->updateTaskModel(N_prec);

			// compute torques
			// read gains from redis
			joint_task->_kp = std::stod(redis_client.get(KP_JOINT_KEY));
			joint_task->_kv = std::stod(redis_client.get(KV_JOINT_KEY));
			
			// position part
			joint_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_JOINT_POS_KEY);

			// torques
			joint_task->computeTorques(command_torques);

		}
		else if (interfacePrimitive == PRIMITIVE_POSORI_TASK)
		{
			if (currentPrimitive == interfacePrimitive)
			{
				init_posori_task(posori_task, robot, redis_client, true);
				posori_task->reInitializeTask();
			}
				
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;

			// compute torques
		}
		currentPrimitive = interfacePrimitive;
	
		// -------------------------------------------
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		
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
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
	return 0;
}