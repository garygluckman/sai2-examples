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
#include "force_sensor/ForceSensorSim.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "keys.h"

/**
 * @struct Object
 * Represents an object within the simulation environment.
 */
struct Object 
{
    /** The object name (e.g. "box") which also defines its shape */
    std::string name;

    /** The object position wrt world frame */
    Vector3d position;

    /** The object orientation wrt world frame */
    Quaterniond orientation;

    /**
     * Creates an object instance with the specified name, position, and orientation
     * in the world frame.
     * 
     * @param name The object name
     * @param pos  The object position wrt world
     * @param ori  The object orientation wrt world
     */
    Object(const std::string& name, const Vector3d& pos, const Quaterniond& ori) 
        : name(name), position(pos), orientation(ori)
    {
    }
};

// simulation constants
constexpr const char *world_file = "./resources/world.urdf";
constexpr const char *camera_name = "camera_fixed";
constexpr const char *sim_title = "SAI2.0 - Two Arms";

RedisClient redis_client;
std::vector<std::shared_ptr<Sai2Model::Sai2Model>> robots;
std::vector<std::shared_ptr<UIForceWidget>> ui_force_widgets;
std::vector<bool> fRobotLinkSelect;
std::vector<Object> objects;

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;


bool fSimulationRunning = false;

void sighandler(int)
{
    fSimulationRunning = false;
}

/**
 * Creates all objects that should be placed within the simulation and adds
 * them to the global vector @a objects.
 */
void initialize_objects()
{
    objects.push_back(Object("box", Vector3d::Zero(), Quaterniond::Identity()));
};

// simulation function prototype
void simulation(Simulation::Sai2Simulation *sim);

// glfw callbacks
void glfwError(int error, const char *description);
void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods);
void mouseClick(GLFWwindow *window, int button, int action, int mods);


int main()
{
    std::cout << "Loading URDF world model file: " << world_file << std::endl;

    // start redis client
    redis_client.connect();
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
    Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

    // load simulation world
    auto sim = new Simulation::Sai2Simulation(world_file, false);
    sim->setCollisionRestitution(0);
    sim->setCoeffFrictionStatic(0.8);
    Vector3d world_gravity = sim->_world->getGravity().eigen();

    // load robot
    for (int i = 0; i < N_ROBOTS; i++)
    {
        robots.push_back(std::make_shared<Sai2Model::Sai2Model>(
            ROBOT_FILES[i],
            false,
            sim->getRobotBaseTransform(ROBOT_NAMES[i]),
            world_gravity
        ));

        ui_force_widgets.push_back(std::make_shared<UIForceWidget>(
            ROBOT_NAMES[i],
            robots[i].get(),
            graphics
        ));

        ui_force_widgets[i]->setEnable(false);
        fRobotLinkSelect.push_back(false);
    }

    // read joint positions, velocities, update model
    for (int i = 0; i < N_ROBOTS; i++)
    {
        sim->getJointPositions(ROBOT_NAMES[i], robots[i]->_q);
        sim->getJointVelocities(ROBOT_NAMES[i], robots[i]->_dq);
        robots[i]->updateKinematics();
    }

    // read objects initial positions
    initialize_objects();

    for (Object& obj : objects)
    {
        sim->getObjectPosition(obj.name, obj.position, obj.orientation);
    }

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

    // set callbacks
    glfwSetKeyCallback(window, keySelect);
    glfwSetMouseButtonCallback(window, mouseClick);

    // cache variables
    double last_cursorx, last_cursory;

    fSimulationRunning = true;
    std::thread sim_thread(simulation, sim);

    // while window is open:
    while (!glfwWindowShouldClose(window))
    {
        // update graphics. this automatically waits for the correct amount of time
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        for (int i = 0; i < N_ROBOTS; i++)
        {
            graphics->updateGraphics(ROBOT_NAMES[i], robots[i].get());
        }

        for (Object& obj : objects)
        {
            graphics->updateObjectGraphics(obj.name, obj.position, obj.orientation);
        }
        graphics->render(camera_name, width, height);

        // swap buffers
        glfwSwapBuffers(window);

        // wait until all GL commands are completed
        glFinish();

        // check for any OpenGL errors
        assert(glGetError() == GL_NO_ERROR);

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

        for (int i = 0; i < N_ROBOTS; i++)
        {
            ui_force_widgets[i]->setEnable(fRobotLinkSelect[i]);
            if (fRobotLinkSelect[i])
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

                if(!ui_force_widgets[i]->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix))
                {
                    fRobotLinkSelect[i] = false;
                }
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
void simulation(Simulation::Sai2Simulation *sim)
{
    // initialize redis pipeline reads & writes
    std::vector<Eigen::VectorXd> robot_command_torques;
    std::vector<Eigen::VectorXd> sensed_forces_moments;

    for (int i = 0; i < N_ROBOTS; i++)
    {
        robot_command_torques.push_back(VectorXd::Zero(robots[i]->dof()));
        sensed_forces_moments.push_back(VectorXd::Zero(6));

        redis_client.addEigenToReadCallback(0, SIM_JOINT_TORQUES_COMMANDED_KEYS[i], robot_command_torques[i]);

        redis_client.setEigenMatrixJSON(SIM_JOINT_TORQUES_COMMANDED_KEYS[i], robot_command_torques[i]);
        redis_client.addEigenToWriteCallback(0, SIM_JOINT_ANGLES_KEYS[i], robots[i]->_q);
        redis_client.addEigenToWriteCallback(0, SIM_JOINT_VELOCITIES_KEYS[i], robots[i]->_dq);
        redis_client.addEigenToWriteCallback(0, SIM_SENSED_FORCES_KEYS[i], sensed_forces_moments[i]);
    }

    redis_client.executeWriteCallback(0);

    // create virtual force sensors
    std::string sensor_link_name = "link7";
    Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
    T_link_sensor.translation() = Eigen::Vector3d(0, 0, 0.113);
    std::vector<std::shared_ptr<ForceSensorSim>> fsensors;

    for (int i = 0; i < N_ROBOTS; i++)
    {
        fsensors.push_back(std::make_shared<ForceSensorSim>(
            ROBOT_NAMES[i], 
            sensor_link_name, 
            T_link_sensor, 
            robots[i].get())
        );

        fsensors[i]->enableFilter(0.01);
    }

	std::vector<Eigen::VectorXd> ui_force_command_torques;
    for (int i = 0; i < N_ROBOTS; i++)
    {
        ui_force_command_torques.push_back(Eigen::VectorXd::Zero(robots[i]->dof()));
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

    while (fSimulationRunning)
    {
        fTimerDidSleep = timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;

        redis_client.executeReadCallback(0);

        for (int i = 0; i < N_ROBOTS; i++)
        {
            // set robot torques to simulation
            VectorXd gravity_torques = VectorXd::Zero(robots[i]->dof());
            robots[i]->gravityVector(gravity_torques);

            if (fRobotLinkSelect[i])
            {
                ui_force_widgets[i]->getUIJointTorques(ui_force_command_torques[i]);
                sim->setJointTorques(ROBOT_NAMES[i], robot_command_torques[i] + gravity_torques + ui_force_command_torques[i]);
            }
            else 
            {
                sim->setJointTorques(ROBOT_NAMES[i], robot_command_torques[i] + gravity_torques);
            }                
        }

        // integrate forward
        double curr_time = timer.elapsedTime();
        double loop_dt = curr_time - last_time;
        sim->integrate(1 / simulation_freq);

        // read joint positions, velocities, update model, read sensor forces
        for (int i = 0; i < N_ROBOTS; i++)
        {
            sim->getJointPositions(ROBOT_NAMES[i], robots[i]->_q);
            sim->getJointVelocities(ROBOT_NAMES[i], robots[i]->_dq);
            robots[i]->updateKinematics();

            fsensors[i]->update(sim);
            fsensors[i]->getForceMomentLocalFrame(sensed_forces_moments[i]);
            sensed_forces_moments[i] *= -1;
        }

        // get object positions from simulation
        for (Object& obj : objects)
        {
            sim->getObjectPosition(obj.name, obj.position, obj.orientation);
        }

        // write new robot state to redis
        redis_client.executeWriteCallback(0);

        // update last time
        last_time = curr_time;

        simulation_counter++;
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";
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
        for (int i = 0; i < N_ROBOTS; i++)
        {
            fRobotLinkSelect[i] = set;
        }
        break;
    // if middle click: don't handle. doesn't work well on laptops
    case GLFW_MOUSE_BUTTON_MIDDLE:
        break;
    default:
        break;
    }
}
