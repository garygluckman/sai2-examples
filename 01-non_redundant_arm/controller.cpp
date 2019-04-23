#include <iostream>
#include <string>

#include "Sai2Model.h"
#include <dynamics3d.h>

#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/PosOriTask.h"

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

using namespace std;
using namespace Eigen;

// redis keys
const string JOINT_ANGLES_KEY = "sai2::examples::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::examples::sensors::dq";
const string JOINT_TORQUES_COMMANDED_KEY = "sai2::examples::actuators::fgc";

const string robot_file = "resources/puma.urdf";

int main (int argc, char** argv) {

	// open redis
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

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

	// PosOri task
	Sai2Primitives::PosOriTask* posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
#ifdef USING_OTG
	posori_task->_use_interpolation_flag = false;
#endif
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_ki_pos = 2.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;
	posori_task->_ki_ori = 2.0;
	Matrix3d initial_orientation;
	Vector3d initial_position;
	robot->rotation(initial_orientation, posori_task->_link_name);
	robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());

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

		// update tasks model
		N_prec = MatrixXd::Identity(dof,dof);

		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;

		// -------------------------------------------
		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		// orientation part
		if(controller_counter < 3000)
		{
			posori_task->_desired_orientation = initial_orientation;
		}
		else if(controller_counter <= 4000)
		{
			Matrix3d R;
			double theta = M_PI/2.0/1000.0 * (controller_counter-3000);
			R << cos(theta) , 0 , sin(theta),
			          0     , 1 ,     0     ,
			    -sin(theta) , 0 , cos(theta);

			posori_task->_desired_orientation = R.transpose()*initial_orientation;
		}
		else if(controller_counter > 8500)
		{
			Matrix3d R;
			double theta = M_PI/2.0/1000.0;
			R << cos(theta) , -sin(theta),  0,
			     sin(theta) ,  cos(theta),  0,
			          0     ,       0    ,  1;

			posori_task->_desired_orientation = R.transpose()*posori_task->_desired_orientation;
		}
		posori_task->_desired_angular_velocity = Vector3d::Zero();

		// position part
		double circle_radius = 0.03;
		double circle_freq = 0.25;
		posori_task->_desired_position = initial_position + circle_radius * Vector3d(0.0, sin(2*M_PI*circle_freq*time), 1-cos(2*M_PI*circle_freq*time));
		posori_task->_desired_velocity = 2*M_PI*circle_freq*0.001*Vector3d(0.0, cos(2*M_PI*circle_freq*time), sin(2*M_PI*circle_freq*time));

		// torques
		posori_task->computeTorques(posori_task_torques);
		
		//------ Final torques
		command_torques = posori_task_torques + coriolis;
		// command_torques.setZero();

		// -------------------------------------------
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		

		// -------------------------------------------
		if(controller_counter % 500 == 0)
		{
			cout << time << endl;
			cout << "desired position : " << posori_task->_desired_position.transpose() << endl;
			cout << "current position : " << posori_task->_current_position.transpose() << endl;
			cout << "position error : " << (posori_task->_desired_position - posori_task->_current_position).norm() << endl;
			cout << endl;
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