/**
 * @file helpers.h
 * @author Munzir
 * @date July 8th, 2013
 * @brief This file comtains some helper functions used for balancing	
 */

#pragma once

#include <Eigen/Dense>

#include <dirent.h>
#include <iostream>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <filter.h>
#include <imud.h>
#include <pciod.h>

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
//#include <dynamics/SkeletonDynamics.h>
//#include <robotics/parser/dart_parser/DartLoader.h>
//#include <simulation/World.h>
#include <initModules.h>
//#include <kinematics/BodyNode.h>
//#include <math/UtilsRotation.h>

#include <kore.hpp>

#include <ddp/costs.hpp>
#include <ddp/ddp.hpp>
#include <ddp/mpc.hpp>
#include <ddp/util.hpp>
#include <krangddp.h>
#include <config4cpp/Configuration.h>

using namespace Eigen;
using namespace dart::simulation;
using namespace dart::common;
using namespace dart::math;
using namespace dart::dynamics;
using namespace config4cpp;

using Scalar = double;  
using DDPDynamics = Krang3D<Scalar>;
using DDP_Opt = optimizer::DDP<DDPDynamics>;
using Cost = Krang3DCost<Scalar>;
using TerminalCost = Krang3DTerminalCost<Scalar>;
using StateTrajectory = typename DDPDynamics::StateTrajectory ;
using ControlTrajectory= typename DDPDynamics::ControlTrajectory ;
using State = typename DDPDynamics::State;
using Control = typename DDPDynamics::Control;


bool myDebug;
extern bool debugGlobal;

/* ******************************************************************************************** */
typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/* ******************************************************************************************** */
// Globals for imu, motors and joystick

somatic_d_t daemon_cx;				///< The context of the current daemon

Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
WorldPtr world;			///< the world representation in dart
SkeletonPtr robot;			///< the robot representation in dart
//simulationPtr World3dof;	    ///< the 3DOF world representation in dart

Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc(); ///< Cmds for waist daemon
ach_channel_t js_chan;				///< Read joystick data on this channel

bool start = false;						///< Giving time to the user to get the robot in balancing angle
bool complyTorque = false;
bool joystickControl = false;
bool resetLeftFT = false, resetRightFT = false; 		
bool overwriteFT = false;			///< To apply force downwards on purpose
bool spinFT= false;						///< To apply force sideways on purpose
double spinGoal = 0.0;
double downGoal = 0.0;
double extraU = 0.0;
bool initDDP = false;                        //Initialize initDDP as a global variable
	
double jsFwdAmp;				///< The gains for joystick forward/reverse input
double jsSpinAmp;				///< The gains for joystick left/right spin input

char b [10];						///< Stores the joystick button inputs
double x [6];						///< Stores the joystick axes inputs

double uglobal [2];                            //Input for the wheels global
double counterc = 0;
/* ******************************************************************************************* */
//Parameters for DDP
ControlTrajectory mDDPControlTraj;
StateTrajectory mDDPStateTraj;
DDPDynamics *mDDPDynamics;
Control mMPCControlRef;  // Store all the reference trajectory MPC compute
Control u;  // Actual Control that is being given
State mMPCStateRef;
int mSteps, mBeginStep;
int mMPCSteps;
double mMPCdt;
CSV_writer<Scalar> mMPCWriter;
State mGoalState;
double mFinalTime;
int mDDPMaxIter;
Eigen::Matrix<double, 8, 1> mDDPStatePenalties;
Eigen::Matrix<double, 8, 1> mDDPTerminalStatePenalties;
Eigen::Matrix<double, 2, 1> mDDPControlPenalties;
int mMPCMaxIter;
int mMPCHorizon;
Eigen::Matrix<double, 8, 1> mMPCStatePenalties;
Eigen::Matrix<double, 8, 1> mMPCTerminalStatePenalties;
Eigen::Matrix<double, 2, 1> mMPCControlPenalties;
double time_ddp;
double time_previous = 0;

/* ******************************************************************************************** */
// All the freaking gains

extern size_t MODE;
extern Vector6d K_groundLo;
extern Vector6d K_groundHi;
extern Vector2d J_ground;
extern Vector6d K_stand;
extern Vector2d J_stand;
extern Vector6d K_sit;
extern Vector2d J_sit;
extern Vector6d K_balLow;
extern Vector2d J_balLow;
extern Vector6d K_balHigh;
extern Vector2d J_balHigh;
extern Vector6d K;	

extern Vector6d state;                        //State for MPC
extern Vector6d refstate;		      //RefState for MPC
extern Vector6d AugState;                     //Augment State for MPC

/* ******************************************************************************************** *
// The arm indices to set/get configurations from dart

extern std::vector <int> left_arm_ids;			///< Ids for left arm
extern std::vector <int> right_arm_ids;			///< Ids for right arm
extern std::vector <int> imuWaist_ids;			///< Ids for waist/imu
extern std::vector <int> imuWaistTorso_ids;			///< Ids for waist/imu (needed for temporary adjustment for torso misalignment)

/* ******************************************************************************************** */

/// Makes the small 1e-17 values in a matrix zero for printing
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat);

/* ******************************************************************************************** */
// Constants for end-effector wrench estimation

//static const double eeMass = 2.3 + 0.169 + 0.000;			///< The mass of the robotiq end-effector
static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the Schunk end-effector
static const Vector3d s2com (0.0, -0.008, 0.091); // 0.065 robotiq itself, 0.026 length of ext + 2nd

/* ******************************************************************************************** */
// Helper functions

/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *);

/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin);

/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState);

/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt, Vector3d* com = NULL);

/// Updates the dart robot representation
void updateDart (double imu);

/// Reads imu values from the ach channels and computes the imu values
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
             filter_kalman_t* kf);

/*/// Reads FT data from ach channels
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data);

/// Computes the offset due to the weights of the end-effector in the FT sensor readings
void computeOffset (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& raw, 
                    SkeletonDynamics& robot, Vector6d& offset, bool left);

/// Givent the raw FT data, gives the wrench in the world frame acting on the FT sensor
void computeExternal (const Vector6d& input, SkeletonDynamics& robot, Vector6d& external, bool left);

/// Given the wrench at the FT sensor givers the wrench on the wheels
void computeWheelWrench(const Vector6d& wrenchSensor, SkeletonDynamics& robot, Vector6d& wheelWrench, bool left);
*/
/// ........
void readGains();

// Update the actual position x0,y0 for MPC-DDP
void updateAugStateReference(Vector6d& state, double dt, Vector2d& AugState);

// Computing MPC-DDP function
void *MPCDDPCompute(void *);

/* ******************************************************************************************** */
// Useful macros

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << "\n" << std::endl
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl
#define pm(a) std::cout << #a << ":\n " << fix((a).matrix()) << "\n" << std::endl
#define pmr(a) std::cout << #a << ":\n " << fix((a)) << "\n" << std::endl
#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
              llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);
#define darm (cout << "dq: "<<llwa.vel[0] << ", " <<llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
              llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);
#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

/* ******************************************************************************************** */
