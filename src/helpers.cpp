/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

#include "helpers.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/circular_buffer.hpp>

using namespace Eigen;
using namespace std;
using namespace Krang;

/* ******************************************************************************************** */
// Initialize the gains for controller and joystick

size_t MODE = 1;
Vector6d K_groundLo;
Vector6d K_groundHi;
Vector2d J_ground (1.0, 1.0);
Vector6d K_stand;
Vector2d J_stand;
Vector6d K_sit;
Vector2d J_sit;
Vector6d K_balLow;
Vector2d J_balLow;
Vector6d K_balHigh;
Vector2d J_balHigh;
Vector6d K = K_groundLo;

/* ******************************************************************************************** */
// Constants for the robot kinematics


const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches 

/* ******************************************************************************************** */
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < 1e-5) mat2(i,j) = 0.0;
	return mat2;
}

/* ******************************************************************************************** *
// Setup the indices for the motor groups

int left_arm_ids_a [7] = {11, 13, 15, 17, 19, 21, 23}; 
int right_arm_ids_a [7] = {12, 14, 16, 18, 20, 22, 24}; 
int imuWaist_ids_a [3] = {5, 8};
vector <int> left_arm_ids (left_arm_ids_a, left_arm_ids_a + 7);						
vector <int> right_arm_ids (right_arm_ids_a, right_arm_ids_a + 7);	
vector <int> imuWaist_ids (imuWaist_ids_a, imuWaist_ids_a + 2);		

// Needed to temporarily adjust the torso misaligment
int imuWaistTorso_ids_a [3] = {5, 8, 9};
vector <int> imuWaistTorso_ids (imuWaistTorso_ids_a, imuWaistTorso_ids_a + 3);		
/* ******************************************************************************************** */

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt, Vector3d* com_) {

	// Read motor encoders, imu and ft and update dart skeleton
  krang->updateSensors(dt);

	// Calculate the COM	
//	Vector3d com = robot->getWorldCOM();
	Vector3d com = Eigen::Vector3d::Zero();
	com(2) -= 0.264;
//	com(0) += 0.008;
//	com(0) += 0.0054; // (0.0101 - 0.0015 - 0.006 - 0.0025 + 0.005 + 0.0025 + 0.0055);
	if(com_ != NULL) *com_ = com;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	state(0) = atan2(com(0), com(2)) - 0.3 * M_PI / 180.0;;
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1])/2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1])/2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;

	// Making adjustment in com to make it consistent with the hack above for state(0)
	com(0) = com(2) * tan(state(0));
}

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState) {

	// First, set the balancing angle and velocity to zeroes
	refState(0) = refState(1) = 0.0;

	// Set the distance and heading velocities using the joystick input
	refState(3) = js_forw;
	refState(5) = js_spin;

	// Integrate the reference positions with the current reference velocities
	refState(2) += dt * refState(3);
	refState(4) += dt * refState(5);
}


/* ******************************************************************************************** */
/// Update Augment State where dt is last iter time
void updateAugStateReference(Vector6d& state, double dt, Vector2d& AugState){

	double R = 0.25;  /// Radius of wheel 25cm;
	AugState(0) = AugState(0) + dt*(R*state(3)*cos(state(4)));   //x0 = x0 + dt*dx0; and dx0 = dx*cos(psi); 
	AugState(1) = AugState(1) + dt*(R*state(3)*sin(state(4)));   //y0 = y0 + dt*dy0; and dy0 = dx*sin(psi);

}


/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin) {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg = 
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, NULL /*&protobuf_c_system_allocator*/, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Get the values
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));

	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, NULL /*&protobuf_c_system_allocator*/);
	
	// Change the gains with the given joystick input
	double deltaTH = 0.2, deltaX = 0.02, deltaSpin = 0.02;
	if(!joystickControl) {
		for(size_t i = 0; i < 4; i++) {
			if(((b[5] == 0) && (b[7] == 0)) && (b[i] == 1)) K(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
			else if((b[5] == 1) && (b[i] == 1)) K((i % 2) + 2) += ((i < 2) ? deltaX : -deltaX);
			else if((b[7] == 1) && (b[i] == 1)) K((i % 2) + 4) += ((i < 2) ? deltaSpin : -deltaSpin);
		}
	}

		// Reset fts
	if(!resetLeftFT && (x[4] < -0.5)) resetLeftFT = true;
	if(!resetRightFT && (x[4] > 0.5)) resetRightFT = true;
	
	// Update joystick and force-compensation controls
	static int lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];
	if((b[4] == 1) && (b[6] == 0) && (b[0] == 1) && (lastb0 == 0)) { 
		joystickControl = !joystickControl;
		if(joystickControl == true) {
			somatic_motor_reset(&daemon_cx, krang->arms[LEFT]);
			somatic_motor_reset(&daemon_cx, krang->arms[RIGHT]);
		}
	}
	if((b[4] == 1) && (b[6] == 0) && (b[1] == 1) && (lastb1 == 0)) complyTorque = !complyTorque;
	if((b[4] == 1) && (b[6] == 0) && (b[2] == 1) && (lastb2 == 0)) {
		if(MODE == 4) {
			printf("Mode 5\n"); 
			K = K_balHigh;
			MODE = 5;
		}
		else if (MODE == 5) {
			printf("Mode 4\n"); 
			K = K_balLow;
			MODE = 4;
		}
	}
	lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];

	// Ignore the joystick statements for the arm control 
	if((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1)) {
		js_forw = js_spin = 0.0;
		return true;
	}

	// Set the values for the axis
	double* x = &(js_msg->axes->data[0]);
	if(MODE == 1 || MODE == 6) {
		js_forw = -J_ground(0) * x[1], js_spin = J_ground(1) * x[2];
	}
	else if(MODE == 4) {
		js_forw = -J_balLow(0) * x[1], js_spin = J_balLow(1) * x[2];
	}
	else if(MODE == 5) {
		js_forw = -J_balHigh(0) * x[1], js_spin = J_balHigh(1) * x[2];
	}
	else {
		js_forw = -x[1] * jsFwdAmp;
		js_spin = x[2] * jsSpinAmp;; 
	}

	
		
	return true;
}

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	Vector6d* kgains [] = {&K_groundLo, &K_stand, &K_sit, &K_balLow, &K_balHigh, &K_groundHi};
	Vector2d* jgains [] = {&J_ground, &J_stand, &J_sit, &J_balLow, &J_balHigh, &J_ground};
	ifstream file ("../gains.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 6; k_idx++) {
		*kgains[k_idx] = Vector6d::Zero();
		*jgains[k_idx] = Vector2d::Zero();
		file.getline(line, 1024);
		std::stringstream stream(line, std::stringstream::in);
		size_t i = 0;
		double newDouble;
		while ((i < 6) && (stream >> newDouble)) (*kgains[k_idx])(i++) = newDouble;
		while (stream >> newDouble) (*jgains[k_idx])(i++ - 6) = newDouble;
	}
	file.close();

	pv(K_groundLo);
	pv(K_groundHi);
	pv(J_ground);
	pv(K_stand);
	pv(J_stand);
	pv(K_sit);
	pv(J_sit);
	pv(K_balLow);
	pv(J_balLow);
	pv(K_balHigh);
	pv(J_balHigh);

	K = K_groundLo;
}

/* ********************************************************************************************* */
/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *) {
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='s') start = true; 
		else if(input=='t') complyTorque = !complyTorque;
		else if(input=='e') resetLeftFT = true; 
		else if(input=='r') resetRightFT = true; 
		else if(input=='.') readGains();
		else if(input=='q') overwriteFT = !overwriteFT;
		else if(input=='p') spinFT = !spinFT;
		else if(input=='[') spinGoal -= 3.0;
		else if(input==']') spinGoal += 3.0;
		else if(input=='{') downGoal -= 5.0;
		else if(input=='}') downGoal += 5.0;
		else if(input=='(') extraU -= 1.0;
		else if(input==')') extraU += 1.0;
		else if(input=='j') { 
			joystickControl = !joystickControl;
			if(joystickControl == true) {
				somatic_motor_reset(&daemon_cx, krang->arms[LEFT]);
				somatic_motor_reset(&daemon_cx, krang->arms[RIGHT]);
			}
		}
		else if(input=='1') {
			printf("Mode 1\n"); 
			K = K_groundLo;
			MODE = 1;
		}
		else if(input=='2') {
			printf("Mode 2\n"); 
			K = K_stand;
			MODE = 2;
		}
		else if(input=='3') {
			printf("Mode 3\n"); 
			K = K_sit;
			MODE = 3;
		}
		else if(input=='4') {
			printf("Mode 4\n"); 
			K = K_balLow;
			MODE = 4;
		}
		else if(input=='5') {
			printf("Mode 5\n"); 
			K = K_balHigh;
			MODE = 5;
		}
		else if(input=='6') {
			printf("Mode 6\n"); 
			K = K_groundHi;
			MODE = 6;
		}
		else if(input=='m'){
			if(MODE == 4 || MODE == 5){   /// If BalLow Mode or BalHigh Mode
				initDDP = true;        /// Initiailize MPCDDP but donot turn it on yet.
			}	
		}
		else if(input=='n'){
			if(MODE == 7)
				MODE = 4;   /// Change to BalLow Mode
		}


	}
	start = true;
}


/* ********************************************************************************************* */
/// Computes the imu value from the imu readings
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
		filter_kalman_t* kf) {

	// ======================================================================
	// Get the readings

	// Get the value
	int r;
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, 
			NULL /*&protobuf_c_system_allocator*/, IMU_CHANNEL_SIZE, imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	_imu = atan2(newX, imu_msg->data[2]); 
	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, NULL /*&protobuf_c_system_allocator */);

	// ======================================================================
	// Filter the readings

	// Skip if a filter is not provided
	if(kf == NULL) return;

	// Setup the data
	kf->z[0] = _imu, kf->z[1] = _imuSpeed;

	// Setup the time-dependent process matrix
	kf->A[0] = kf->A[3] = 1.0;
	kf->A[2] = dt;

	// Setup the process noise matrix
	static const double k1 = 2.0;
	static const double k1b = 5.0;
	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
	kf->R[3] = (dt*dt) * k1b;
	
	// First make a prediction of what the reading should have been, then correct it
	filter_kalman_predict(kf);
	filter_kalman_correct(kf);

	// Set the values
	_imu = kf->x[0], _imuSpeed = kf->x[1];
}

/* ******************************************************************************************** */
/*void computeExternal (const Vector6d& input, SkeletonDynamics& robot, Vector6d& external, 
		bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame by setting the arm values
	// and the imu/waist values
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d Rsw = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
//	vector <int> dofs;
//	for(size_t i = 0; i < 25; i++) dofs.push_back(i);
//	if(myDebug) cout << "\nq in computeExternal: " << robot.getConfig(dofs).transpose() << endl;
	
	// Create the wrench with computed rotation to change the frame from the world to the sensor
	Matrix6d pSsensor_world = MatrixXd::Identity(6,6); 
	pSsensor_world.topLeftCorner<3,3>() = Rsw;
	pSsensor_world.bottomRightCorner<3,3>() = Rsw;

	// Get the weight vector (note that we use the world frame for gravity so towards -y)
	// static const double eeMass = 0.169;	// kg - ft extension
	Vector6d weightVector_in_world;
	weightVector_in_world << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d wrenchWeight = pTcom_sensor * pSsensor_world * weightVector_in_world;

	// Remove the effect from the sensor value and convert the wrench into the world frame
	external = input - wrenchWeight;
	external = pSsensor_world.transpose() * external;	
}*/

/* ******************************************************************************************** */
/*void computeOffset (double imu, double waist, const somatic_motor_t& lwa, const Vector6d& raw, 
		SkeletonDynamics& robot, Vector6d& offset, bool left) {

	// Get the point transform wrench due to moving the affected position from com to sensor origin
	// The transform is an identity with the bottom left a skew symmetric of the point translation
	Matrix6d pTcom_sensor = MatrixXd::Identity(6,6); 
	pTcom_sensor.bottomLeftCorner<3,3>() << 0.0, -s2com(2), s2com(1), s2com(2), 0.0, -s2com(0), 
		-s2com(1), s2com(0), 0.0;

	// Get the rotation between the world frame and the sensor frame. 
	robot.setConfig(imuWaist_ids, Vector2d(-imu + M_PI_2, waist));
	robot.setConfig(left ? left_arm_ids : right_arm_ids, Map <Vector7d> (lwa.pos));
	const char* nodeName = left ? "lGripper" : "rGripper";
	Matrix3d R = robot.getNode(nodeName)->getWorldTransform().topLeftCorner<3,3>().transpose();
	cout << "Transform : "<< endl << R << endl;
	vector <int> dofs;
	for(size_t i = 0; i < 25; i++) dofs.push_back(i);
	cout << "\nq in computeExternal: " << robot.getConfig(dofs).transpose() << endl;

	// Create the wrench with computed rotation to change the frame from the bracket to the sensor
	Matrix6d pSsensor_bracket = MatrixXd::Identity(6,6); 
	pSsensor_bracket.topLeftCorner<3,3>() = R;
	pSsensor_bracket.bottomRightCorner<3,3>() = R;
	
	// Get the weight vector (note that we use the bracket frame for gravity so towards -y)
	Vector6d weightVector_in_bracket;
	weightVector_in_bracket << 0.0, 0.0, -eeMass * 9.81, 0.0, 0.0, 0.0;
	
	// Compute what the force and torque should be without any external values by multiplying the 
	// position and rotation transforms with the expected effect of the gravity 
	Vector6d expectedFT = pTcom_sensor * pSsensor_bracket * weightVector_in_bracket;
	pv(raw);
	pv(expectedFT);

	// Compute the difference between the actual and expected f/t values
	offset = expectedFT - raw;
	pv(offset);
}*/

/* ******************************************************************************************* */
// Compute the wrench on the wheel as an effect of the wrench acting on the sensor
/*void computeWheelWrench(const Vector6d& wrenchSensor, SkeletonDynamics& robot, Vector6d& wheelWrench, bool left) {
	
	// Get the position vector of the sensor with respect to the wheels
	const char* nodeName = left ? "lGripper" : "rGripper";
	Vector3d Tws = robot.getNode(nodeName)->getWorldTransform().topRightCorner<3,1>();
	if(0 && myDebug) cout << Tws.transpose() << endl;

	// Get the wrench shift operator to move the wrench from sensor origin to the wheel axis
	// TODO: This shifting is to the origin of the world frame in dart. It works now because it is not 
	// being updated by the amc encoders. We need to shift the wrench to a frame having origin at the 
	// wheel axis.
	Tws(2) -= 0.264;
	Matrix6d pTsensor_wheel = MatrixXd::Identity(6,6);
	pTsensor_wheel.bottomLeftCorner<3,3>() << 0.0, -Tws(2), Tws(1), Tws(2), 0.0, -Tws(0),
		-Tws(1), Tws(0), 0.0;

	// Shift the wrench from the sensor origin to the wheel axis
	wheelWrench = pTsensor_wheel * wrenchSensor;
}*/

/* ********************************************************************************************* */
/*bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data) {

	// Check if there is anything to read
	int result;
	size_t numBytes = 0;
	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &ft_chan, &numBytes, &abstimeout, 
		ACH_O_LAST, &result);

	// Return if there is nothing to read
	if(numBytes == 0) return false;

	// Read the message with the base struct to check its type
	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
	if((msg->meta == NULL) || !msg->meta->has_type) return false;
	if(msg->meta->type != SOMATIC__MSG_TYPE__FORCE_MOMENT) return false;

	// Read the force-torque message and write it into the vector
	Somatic__ForceMoment* ftMessage = somatic__force_moment__unpack(&(daemon_cx.pballoc), 
		numBytes, buffer);
	for(size_t i = 0; i < 3; i++) data(i) = ftMessage->force->data[i]; 
	for(size_t i = 0; i < 3; i++) data(i+3) = ftMessage->moment->data[i]; 
	return true;
}*/



SkeletonPtr create3DOF_URDF()
{
  dart::utils::DartLoader loader;
  SkeletonPtr threeDOF = 
      loader.parseSkeleton("/home/munzir/project/09-URDF/3DOF-WIP/3dof.urdf");
  threeDOF->setName("m3DOF");
  
  threeDOF->getJoint(0)->setDampingCoefficient(0, 0.5);
  threeDOF->getJoint(1)->setDampingCoefficient(0, 0.5);

  return threeDOF;
}

void getSimple(SkeletonPtr& threeDOF, SkeletonPtr& krang) {
  // Load the full body with fixed wheel and set the pose q
  // dart::utils::DartLoader loader;
  // SkeletonPtr krangFixedWheel =
  //     loader.parseSkeleton("/home/krang/dart/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf");
  
  // Body Mass
  double mFull = krang->getMass(); 
  double mLWheel = krang->getBodyNode("LWheel")->getMass();
  double mRWheel = krang->getBodyNode("RWheel")->getMass();
  double mBody = mFull - mLWheel - mRWheel;


  Eigen::Vector3d bodyCOM;
  dart::dynamics::Frame* baseFrame = krang->getBodyNode("Base");
  bodyCOM = (mFull*krang->getCOM(baseFrame) - mLWheel*krang->getBodyNode("LWheel")->getCOM(baseFrame) - mLWheel*krang->getBodyNode("RWheel")->getCOM(baseFrame))/(mFull - mLWheel - mRWheel);

  // Body inertia (axis)
  double m;
  Eigen::Matrix3d iMat;
  Eigen::Matrix3d iBody = Eigen::Matrix3d::Zero();
  double ixx, iyy, izz, ixy, ixz, iyz;  
  Eigen::Matrix3d rot;
  Eigen::Vector3d t;
  Eigen::Matrix3d tMat;
  dart::dynamics::BodyNodePtr b;
  int nBodies = krang->getNumBodyNodes();
  for(int i=0; i<nBodies; i++){
    if(i==1 || i==2) continue; // Skip wheels
    b = krang->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation();
    t = krang->getCOM(baseFrame) - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
    m = b->getMass();
    iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
            ixy, iyy, iyz,
            ixz, iyz, izz;
    iMat = rot*iMat*rot.transpose(); // Inertia tensor of the body around its CoM expressed in base frame
    tMat << (t(1)*t(1)+t(2)*t(2)), (-t(0)*t(1)),          (-t(0)*t(2)),
            (-t(0)*t(1)),          (t(0)*t(0)+t(2)*t(2)), (-t(1)*t(2)),
            (-t(0)*t(2)),          (-t(1)*t(2)),          (t(0)*t(0)+t(1)*t(1));
    iMat = iMat + m*tMat; // Parallel Axis Theorem
    iBody += iMat;
  }


  // Aligning threeDOF base frame to have the y-axis pass through the CoM
  double th = atan2(bodyCOM(2), bodyCOM(1));
  rot << 1, 0, 0,
         0, cos(th), sin(th),
         0, -sin(th), cos(th);
  bodyCOM = rot*bodyCOM;
  iBody = rot*iBody*rot.transpose();

  // Set the 3 DOF robot parameters
  threeDOF->getBodyNode("Base")->setMomentOfInertia(iBody(0,0), iBody(1,1), iBody(2,2), iBody(0,1), iBody(0,2), iBody(1,2));
  threeDOF->getBodyNode("Base")->setLocalCOM(bodyCOM);
  threeDOF->getBodyNode("Base")->setMass(mBody);

  // Print them out
  // cout << "mass: " << mBody << endl;
  // cout << "COM: " << bodyCOM(0) << ", " << bodyCOM(1) << ", " << bodyCOM(2) << endl;
  // cout << "ixx: " << iBody(0,0) << ", iyy: " << iBody(1,1) << ", izz: " << iBody(2,2) << endl;
  // cout << "ixy: " << iBody(0,1) << ", ixz: " << iBody(0,2) << ", iyz: " << iBody(1,2) << endl;

  // Update 3DOF state
  // get positions
  Eigen::Matrix3d baseRot = krang->getBodyNode("Base")->getTransform().rotation();
  baseRot = baseRot*rot.transpose();
  Eigen::AngleAxisd aa(baseRot);
  Eigen::Matrix<double, 8, 1> q, dq;
  q << aa.angle()*aa.axis(), krang->getPositions().segment(3, 5);
  threeDOF->setPositions(q);

  // TODO: When joints are unlocked qBody1 of the 3DOF (= dth = COM angular speed) is not the same as qBody1 of the full robot
  dq << rot*krang->getVelocities().head(3), rot*krang->getVelocities().segment(3, 3), krang->getVelocities().segment(6, 2);
  threeDOF->setVelocities(dq);
}

void computeDDPTrajectory(Vector6d& state, Vector2d& AugState) {
  
  param p; 
  double ixx, iyy, izz, ixy, ixz, iyz; 
  Eigen::Vector3d com;
  Eigen::Matrix3d iMat;      
  Eigen::Matrix3d tMat;
  
  dart::dynamics::Frame* baseFrame = m3DOF->getBodyNode("Base");
  p.R = mR; p.L = mL; p.g=9.800000e+00;
  
  p.mw = m3DOF->getBodyNode("LWheel")->getMass(); 
  
  m3DOF->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  
  p.YYw = ixx; p.ZZw = izz; p.XXw = iyy; // Wheel frame of reference in ddp dynamic model is different from the one in DART
  p.m_1 = m3DOF->getBodyNode("Base")->getMass(); 
  com = m3DOF->getBodyNode("Base")->getCOM(baseFrame);
  p.MX_1 = p.m_1*com(0); p.MY_1 = p.m_1*com(1); p.MZ_1 = p.m_1*com(2);
  
  m3DOF->getBodyNode("Base")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  Eigen::Vector3d s = -com; // Position vector from local COM to body COM expressed in base frame
  iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
          ixy, iyy, iyz,
          ixz, iyz, izz;
  tMat << (s(1)*s(1)+s(2)*s(2)), (-s(0)*s(1)),          (-s(0)*s(2)),
          (-s(0)*s(1)),          (s(0)*s(0)+s(2)*s(2)), (-s(1)*s(2)),
          (-s(0)*s(2)),          (-s(1)*s(2)),          (s(0)*s(0)+s(1)*s(1));
  iMat = iMat + p.m_1*tMat; // Parallel Axis Theorem
  p.XX_1 = iMat(0,0); p.YY_1 = iMat(1,1); p.ZZ_1 = iMat(2,2);
  p.XY_1 = iMat(0,1); p.YZ_1 = iMat(1,2); p.XZ_1 = iMat(0,2);
  p.fric_1 = m3DOF->getJoint(0)->getDampingCoefficient(0); // Assuming both joints have same friction coeff (Please make sure that is true)
  
  CSV_writer<Scalar> writer;
  util::DefaultLogger logger;
  bool verbose = true;
  Scalar tf = mFinalTime;
  auto time_steps = util::time_steps(tf, mMPCdt);
  int max_iterations = mDDPMaxIter;
  

  mDDPDynamics = new DDPDynamics(p);
  
  // Initial state 
  // State x0 = getCurrentState();
  // x0 << 0, 0, x0(2), 0, 0, 0, 0, 0; 
  //Initialize the state with the current state of Krang
  State x0; x0 << state(2),state(4),state(0),state(3),state(5),state(1),AugState(0),AugState(1);
  cout << "initState: " << x0.transpose() << endl;
  // Dynamics::State xf; xf << 2, 0, 0, 0, 0, 0, 0.01, 5;
  // Dynamics::State xf; xf << 5, 0, 0, 0, 0, 0, 5, 0;
  DDPDynamics::ControlTrajectory u = DDPDynamics::ControlTrajectory::Zero(2, time_steps);

  // Costs
  Cost::StateHessian Q;
  Q.setZero();
  Q.diagonal() << mDDPStatePenalties;

  Cost::ControlHessian R;
  R.setZero();
  R.diagonal() << mDDPControlPenalties;

  TerminalCost::Hessian Qf;
  Qf.setZero();
  Qf.diagonal() << mDDPTerminalStatePenalties;

  Cost cp_cost(mGoalState, Q, R);
  TerminalCost cp_terminal_cost(mGoalState, Qf);

  // initialize DDP for trajectory planning
  DDP_Opt trej_ddp (mMPCdt, time_steps, max_iterations, &logger, verbose);

  // Get initial trajectory from DDP
  OptimizerResult<DDPDynamics> DDP_traj = trej_ddp.run(x0, u, *mDDPDynamics, cp_cost, cp_terminal_cost);

  
  mDDPStateTraj = DDP_traj.state_trajectory;
  mDDPControlTraj = DDP_traj.control_trajectory;

  writer.save_trajectory(mDDPStateTraj, mDDPControlTraj, "initial_traj.csv");
}

void *MPCDDPCompute(void *) {
  Configuration *  cfg = Configuration::create();
  const char *     scope = "";
  const char *     configFile = "/home/munzir/project/krang/23-mpc-hardware-experiment/src/controlParams.cfg";
  const char * str;
  std::istringstream stream;

  try {
    cfg->parse(configFile);

    str = cfg->lookupString(scope, "goalState"); 
    stream.str(str); for(int i=0; i<8; i++) stream >> mGoalState(i); stream.clear();

    mFinalTime = cfg->lookupFloat(scope, "finalTime");
    
    mDDPMaxIter = cfg->lookupInt(scope, "DDPMaxIter");
    
    str = cfg->lookupString(scope, "DDPStatePenalties"); 
    stream.str(str); for(int i=0; i<8; i++) stream >> mDDPStatePenalties(i); stream.clear();
    
    str = cfg->lookupString(scope, "DDPTerminalStatePenalties"); 
    stream.str(str); for(int i=0; i<8; i++) stream >> mDDPTerminalStatePenalties(i); stream.clear();
    
    str = cfg->lookupString(scope, "DDPControlPenalties"); 
    stream.str(str); for(int i=0; i<2; i++) stream >> mDDPControlPenalties(i); stream.clear();
    
    mBeginStep = cfg->lookupInt(scope, "beginStep");

    mMPCMaxIter = cfg->lookupInt(scope, "MPCMaxIter");
    
    mMPCHorizon = cfg->lookupInt(scope, "MPCHorizon");
    
    str = cfg->lookupString(scope, "MPCStatePenalties"); 
    stream.str(str); for(int i=0; i<8; i++) stream >> mMPCStatePenalties(i); stream.clear();
    
    str = cfg->lookupString(scope, "MPCTerminalStatePenalties"); 
    stream.str(str); for(int i=0; i<8; i++) stream >> mMPCTerminalStatePenalties(i); stream.clear();
    
    str = cfg->lookupString(scope, "MPCControlPenalties"); 
    stream.str(str); for(int i=0; i<2; i++) stream >> mMPCControlPenalties(i); stream.clear();

    str = cfg->lookupString(scope, "tauLim"); 
    stream.str(str); for(int i=0; i<18; i++) stream >> mTauLim(i); stream.clear();
    
  } catch(const ConfigurationException & ex) {
      cerr << ex.c_str() << endl;
      cfg->destroy();
  }
  int mSteps = 0;
  int mMPCSteps = -1; 
  double mMPCdt = 0.01;
    // mdqFilt = new filter(8, 50);
    // mR = 0.25;
    // mL = 0.68;//*6;
  char mpctrajfile[] = "mpc_traj.csv";
  CSV_writer<Scalar> mMPCWriter;
  mMPCWriter.open_file(mpctrajfile);

  while(!initDDP); //Stay here till initDDP is false
// Initialize the simplified robot
  SkeletonPtr threeDOF = create3DOF_URDF();
  // simulation::World* World3dof;
  World3dof = std::make_shared<World>();
  World3dof->addSkeleton(m3DOF);
  getSimple(m3DOF, robot); 
	while(!initDDP);  // Stay here till initDDP is false
  computeDDPTrajectory(state,AugState);

   Eigen::Vector2d ch;
   ch << (mDDPStateTraj.block<2,1>(6,mDDPStateTraj.cols()) - mGoalState.tail(2));
   if(ch.norm() < 0.1){
      	initDDP = false;
      	MODE = 4;
    }
    else
    {
      MODE = 7;  // Change MODE to MPC Mode
      // writer.save_trajectory(ddp_state_traj, ddp_ctl_traj, "initial_traj.csv");
      struct timespec t_prev, t_now = aa_tm_now();
  		t_prev = t_now;
  		double initialtime = (double)aa_tm_timespec2sec(t_now);
  	  Scalar tf = mFinalTime;
      double i = initialtime;
  		while(i<initialtime+tf){
        getSimple(m3DOF, robot);
        State cur_state;  // Initialize the new current state
        cur_state << state(2),state(4),state(0),state(3),state(5),state(1),AugState;
    
       // mMPCSteps = cur_mpc_steps;
        int max_iterations = mMPCMaxIter; 
        bool verbose = true; 
        util::DefaultLogger logger;
        int mpc_horizon = mMPCHorizon; 
        
        DDPDynamics::State target_state;
        target_state = mDDPStateTraj.col(mMPCSteps + mpc_horizon);
        DDPDynamics::ControlTrajectory hor_control = DDPDynamics::ControlTrajectory::Zero(2, mpc_horizon);
        DDPDynamics::StateTrajectory hor_traj_states = mDDPStateTraj.block(0, mMPCSteps, 8, mpc_horizon);
        
        DDP_Opt ddp_horizon(mMPCdt, mpc_horizon, max_iterations, &logger, verbose);
        
        Cost::StateHessian Q_mpc, Qf_mpc;
        Cost::ControlHessian ctl_R;
        
        ctl_R.setZero();
        ctl_R.diagonal() << mMPCControlPenalties;
        Q_mpc.setZero();
        Q_mpc.diagonal() << mMPCStatePenalties;
        Qf_mpc.setZero();
        Qf_mpc.diagonal() << mMPCTerminalStatePenalties;
        Cost running_cost_horizon(target_state, Q_mpc, ctl_R);
        TerminalCost terminal_cost_horizon(target_state, Qf_mpc);
        
        OptimizerResult<DDPDynamics> results_horizon;
        results_horizon.control_trajectory = hor_control;
        
        results_horizon = ddp_horizon.run_horizon(cur_state, hor_control, hor_traj_states, *mDDPDynamics, running_cost_horizon, terminal_cost_horizon);
        mMPCControlRef = results_horizon.control_trajectory.col(0); 
        mMPCStateRef = results_horizon.state_trajectory.col(1);


        mMPCWriter.save_step(cur_state, mMPCControlRef);
        timer = 0;
  			t_now = aa_tm_now();
  			double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));				
  			i = i+dt;		
      }
   }
    MODE = 4;
  }
