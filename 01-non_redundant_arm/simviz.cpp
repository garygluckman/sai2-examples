#include <iostream>
#include <string>
#include <thread>
#include <cmath>
#include <csignal>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "uiforce/UIForceWidget.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/PosOriTask.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

#include "keys.h"

using namespace Eigen;

// redis keys
constexpr const char *world_file = "resources/world.urdf";
constexpr const char *robot_name = "PUMA";
constexpr const char *camera_name = "camera";
constexpr const char *sim_title = "SAI2.0 - Puma Example";

RedisClient redis_client;

bool fSimulationRunning = false;

/** 
 * Custom signal handler: used here to terminate the simulation.
 * @param signal The signal (e.g. SIGINT) that was raised.
 */
void sighandler(int) 
{ 
	fSimulationRunning = false; 
}

// simulation and control loop
void simulation(Sai2Model::Sai2Model *robot, Simulation::Sai2Simulation *sim, 
				UIForceWidget *ui_force_widget);

// initialize window manager
GLFWwindow *glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char *description);

// callback when a key is pressed
void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow *window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;


int main(int argc, char **argv)
{
	std::cout << "Loading URDF world model file: " << world_file << std::endl;

	// open redis
	redis_client.connect();
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	auto robot = new Sai2Model::Sai2Model(ROBOT_FILE, false, sim->getRobotBaseTransform(robot_name), world_gravity);

	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow *window = glfwInitialize();

	double last_cursorx, last_cursory;

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	// start the simulation thread first
	fSimulationRunning = true;
	std::thread sim_thread(simulation, robot, sim, ui_force_widget);

	// while window is open - GLFW (OpenGL framework) main loop
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

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
		if (fTransXp)
		{
			camera_pos = camera_pos + 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat + 0.05 * cam_roll_axis;
		}
		if (fTransXn)
		{
			camera_pos = camera_pos - 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat - 0.05 * cam_roll_axis;
		}
		if (fTransYp)
		{
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05 * cam_up_axis;
			camera_lookat = camera_lookat + 0.05 * cam_up_axis;
		}
		if (fTransYn)
		{
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05 * cam_up_axis;
			camera_lookat = camera_lookat - 0.05 * cam_up_axis;
		}
		if (fTransZp)
		{
			camera_pos = camera_pos + 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat + 0.1 * cam_depth_axis;
		}
		if (fTransZn)
		{
			camera_pos = camera_pos - 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat - 0.1 * cam_depth_axis;
		}
		if (fRotPanTilt)
		{
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006 * (cursorx - last_cursorx);
			double azimuth = 0.006 * (cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt;
			m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt * (camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan;
			m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan * (camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		// allows the user to drag on a link and apply a force
		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
			}
		}
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
void simulation(Sai2Model::Sai2Model *robot, Simulation::Sai2Simulation *sim, UIForceWidget *ui_force_widget)
{
	// prepare variables
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	// initialize redis write & read keys
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToWriteCallback(0, UI_FORCE_KEY, ui_force);
	redis_client.addEigenToWriteCallback(0, UI_FORCE_COMMAND_TORQUES_KEY, ui_force_command_torques);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	fSimulationRunning = true;
	while (fSimulationRunning)
	{
		fTimerDidSleep = timer.waitForNextLoop();
		redis_client.executeReadCallback(0);

		ui_force_widget->getUIForce(ui_force);
		ui_force_widget->getUIJointTorques(ui_force_command_torques);

		if (fRobotLinkSelect)
			sim->setJointTorques(robot_name, command_torques + ui_force_command_torques);
		else
			sim->setJointTorques(robot_name, command_torques);
		
		// integrate forward
		sim->integrate(0.001);

		// read robot state and update redis
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();

		redis_client.executeWriteCallback(0);
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";
}

//------------------------------------------------------------------------------
GLFWwindow *glfwInitialize()
{
	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor *primary = glfwGetPrimaryMonitor();
	const GLFWvidmode *mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow *window = glfwCreateWindow(windowW, windowH, sim_title, nullptr, nullptr);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char *description)
{
	std::cerr << "GLFW Error: " << description << std::endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// exit application
		glfwSetWindowShouldClose(window, GL_TRUE);
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

void mouseClick(GLFWwindow *window, int button, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button)
	{
	// left click pans and tilts
	case GLFW_MOUSE_BUTTON_LEFT:
		fRotPanTilt = set;
		break;
	// if right click: don't handle. this is for menu selection
	case GLFW_MOUSE_BUTTON_RIGHT:
		fRobotLinkSelect = set;
		//TODO: menu
		break;
	// if middle click: don't handle. doesn't work well on laptops
	case GLFW_MOUSE_BUTTON_MIDDLE:
		break;
	default:
		break;
	}
}
