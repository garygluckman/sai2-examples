// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world.urdf";
const vector<string> robot_files = {
	"./resources/panda_arm_flat_ee.urdf",
	"./resources/panda_arm_flat_ee.urdf",
};
const vector<string> robot_names = {
	"PANDA1",
	"PANDA2",
};
const vector<string> object_names = {
	"box",
};
const string camera_name = "camera_fixed";

const string plate_file = "resources/plate.urdf";
const string plate_name = "Plate";

const int n_robots = robot_names.size();
const int n_objects = object_names.size();

// redis keys:
// - write:
const string TIMESTAMP_KEY = "sai2::WarehouseApplications::simulation::timestamp";
const vector<string> JOINT_ANGLES_KEYS  = {
	"sai2::WarehouseApplications::panda1::sensors::q",
	"sai2::WarehouseApplications::panda2::sensors::q",
};
const vector<string> JOINT_VELOCITIES_KEYS = {
	"sai2::WarehouseApplications::panda1::sensors::dq",
	"sai2::WarehouseApplications::panda2::sensors::dq",
};

const vector<string> SENSED_FORCES_KEYS = {
	"sai2::WarehouseApplications::panda1::sensors::wrist_force_moment",
	"sai2::WarehouseApplications::panda2::sensors::wrist_force_moment",
};

const string OBJECT_POSITION_KEY = "sai2::WarehouseApplications::object_position";

// - read
const vector<string> TORQUES_COMMANDED_KEYS = {
	"sai2::WarehouseApplications::panda1::actuators::fgc",
	"sai2::WarehouseApplications::panda2::actuators::fgc",
};


RedisClient redis_client;
vector<Vector3d> object_positions;
vector<Quaterniond> object_orientations;

// simulation function prototype
// void simulation(vector<Sai2Model::Sai2Model*> robots, Sai2Model::Sai2Model* plate, Simulation::Sai2Simulation* sim);
void simulation(vector<Sai2Model::Sai2Model*> robots, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.8);
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();

	// load robots
	vector<Sai2Model::Sai2Model*> robots;
	for(int i=0 ; i<n_robots ; i++)
	{
		robots.push_back(new Sai2Model::Sai2Model(robot_files[i], false, sim->getRobotBaseTransform(robot_names[i]), world_gravity));
	}

	// // load plate
	// auto plate = new Sai2Model::Sai2Model(plate_file, false, sim->getRobotBaseTransform(plate_name), world_gravity);


	// read joint positions, velocities, update model
	for(int i=0 ; i<n_robots ; i++)
	{
		sim->getJointPositions(robot_names[i], robots[i]->_q);
		sim->getJointVelocities(robot_names[i], robots[i]->_dq);
		robots[i]->updateKinematics();
	}

	// read objects initial positions
	for(int i=0 ; i< n_objects ; i++)
	{
		Vector3d obj_pos = Vector3d::Zero();
		Quaterniond obj_ori = Quaterniond::Identity();
		sim->getObjectPosition(object_names[i], obj_pos, obj_ori);
		object_positions.push_back(obj_pos);
		object_orientations.push_back(obj_ori);
	}

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - WarehouseApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	fSimulationRunning = true;
	// thread sim_thread(simulation, robots, plate, sim);
	thread sim_thread(simulation, robots, sim);

	// while window is open:
	while (!glfwWindowShouldClose(window))
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		for(int i=0 ; i<n_robots ; i++)
		{
			graphics->updateGraphics(robot_names[i], robots[i]);
		}
		// graphics->updateGraphics(plate_name, plate);
		for(int i=0 ; i< n_objects ; i++)
		{
			graphics->updateObjectGraphics(object_names[i], object_positions[i], object_orientations[i]);
		}
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
// void simulation(vector<Sai2Model::Sai2Model*> robots, Sai2Model::Sai2Model* plate, Simulation::Sai2Simulation* sim) {
void simulation(vector<Sai2Model::Sai2Model*> robots, Simulation::Sai2Simulation* sim) {

	// initialize redis values
	for(int i=0 ; i<n_robots ; i++)
	{
		redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i], VectorXd::Zero(robots[i]->dof()));
	}

	// plate controller
	Eigen::Vector2d plate_qd = Eigen::Vector2d::Zero();
	Eigen::Vector2d plate_torques = Eigen::Vector2d::Zero();

	// create virtual force sensors
	string sensor_link_name = "link7";
	Affine3d T_link_sensor = Affine3d::Identity();
	T_link_sensor.translation() = Vector3d(0, 0, 0.113);
	vector<ForceSensorSim*> fsensors;
	vector<VectorXd> sensed_forces_moments;

	for(int i=0 ; i<n_robots ; i++)
	{
		fsensors.push_back(new ForceSensorSim(robot_names[i], sensor_link_name, T_link_sensor, robots[i]));
		fsensors[i]->enableFilter(0.01);
		sensed_forces_moments.push_back(VectorXd::Zero(6));
	}

	// create a timer
	double simulation_freq = 1000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(simulation_freq); 
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs
	double last_time = start_time;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		double time = timer.elapsedTime() - start_time;

		for(int i=0 ; i<n_robots ; i++)
		{
			int dof = robots[i]->dof();
			VectorXd command_torques = VectorXd::Zero(dof);
			// read arm torques from redis
			command_torques = redis_client.getEigenMatrixJSON(TORQUES_COMMANDED_KEYS[i]);

			// set robot torques to simulation
			VectorXd gravity_torques = VectorXd::Zero(dof);
			robots[i]->gravityVector(gravity_torques);
			sim->setJointTorques(robot_names[i], command_torques + gravity_torques);
		}

		// // plate controller
		// sim->getJointPositions(plate_name, plate->_q);
		// sim->getJointVelocities(plate_name, plate->_dq);
		// plate->updateKinematics();

		// plate_qd(0) = 5.0/180.0*M_PI*sin(2*M_PI*0.28*time);
		// plate_qd(1) = 7.0/180.0*M_PI*sin(2*M_PI*0.22*time);

		// plate_torques = -1000.0*(plate->_q - plate_qd) - 75.0*plate->_dq;

		// sim->setJointTorques(plate_name, plate_torques);
		
		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		// sim->integrate(loop_dt);
		sim->integrate(1/simulation_freq);

		// read joint positions, velocities, update model, read sensor forces
		for(int i=0 ; i<n_robots ; i++)
		{
			sim->getJointPositions(robot_names[i], robots[i]->_q);
			sim->getJointVelocities(robot_names[i], robots[i]->_dq);
			robots[i]->updateKinematics();

			fsensors[i]->update(sim);
			fsensors[i]->getForceMomentLocalFrame(sensed_forces_moments[i]);
		}

		// get object positions from simulation
		for(int i=0 ; i< n_objects ; i++)
		{
			sim->getObjectPosition(object_names[i], object_positions[i], object_orientations[i]);
		}

		// write new robot state to redis
		for(int i=0 ; i<n_robots ; i++)
		{
			redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEYS[i], robots[i]->_q);
			redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i], robots[i]->_dq);

			redis_client.setEigenMatrixJSON(SENSED_FORCES_KEYS[i], -sensed_forces_moments[i]);
		}
		redis_client.set(TIMESTAMP_KEY, to_string(curr_time));

		//update last time
		last_time = curr_time;

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

