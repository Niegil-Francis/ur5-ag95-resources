#ifndef HD_KEY_CTRL
#define HD_KEY_CTRL

#include "flight_ctrl.h"
#include "pid_linear.h"
#include "pid_rotate.h"
#include "imumath.h"

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

// Libraries to read from keyboard
#include <termios.h>
#include <thread>

// Maximum velocities in m/s^2 and rad/s
#define MAX_LINEAR_X 2
#define MAX_LINEAR_Y 2
#define MAX_LINEAR_Z 2
#define MAX_YAW 2.5

 #define USE_NEG_ANGLE // Uncomment this if you want to use angle range (-180,180]

// Store setpoints
double set_points[3];				// x,y,z
Cardan set_orient;

// Store velocity commands
float cmd_values[4];				// x,y,z,yaw

// Sensing and related variables
float position[3];					// x,y,z
Cardan cur_orient;					// current orientation 

// VIMAN instance
Viman vm;

// Controller and related
VmPidLinear height_controller_;
VmPidRotate yaw_controller_;
bool isPidRunning;

// Vars to read user cmds
static struct termios old, current;
bool do_not_quit = true;

// Functions to read user input
void display_help(void);
void initTermios(void);
void resetTermios(void);
char getch(void);
void read_input(void);

// Functions to assist semi-autonomous movement
void calibrate(void);
void HeightCallbck(const geometry_msgs::PointStamped&);
void ImuCallbck(const sensor_msgs::Imu&);


#endif // HD_KEY_CTRL
