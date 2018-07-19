/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

#include "helpers-MPCDDP.h"
#include "krangddp.h"





#include <boost/circular_buffer.hpp>
#include <ddp/costs.hpp>
#include <ddp/ddp.hpp>
#include <ddp/mpc.hpp>
#include <ddp/util.hpp>



#include <iostream>
#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;
using namespace Krang;

using Scalar = double;  
using Dynamics = Krang3D<Scalar>;
using DDP_Opt = optimizer::DDP<Dynamics>;
using Cost = Krang3DCost<Scalar>;
using TerminalCost = Krang3DTerminalCost<Scalar>;
using StateTrajectory = typename Dynamics::StateTrajectory ;
using ControlTrajectory= typename Dynamics::ControlTrajectory ;
using State = typename Dynamics::State;
using Control = typename Dynamics::Control;

/* ****************************************** */




SkeletonPtr create3DOF_URDF()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr threeDOF = 
      //loader.parseSkeleton("/home/krang/dart/09-URDF/3DOF-WIP/3dof.urdf");
      loader.parseSkeleton("/home/krang/dart/09-URDF/3DOF-WIP/3dof.urdf");
  threeDOF->setName("m3DOF");

  // Set parameters of Body that reflect the ones we will actually have 
  Eigen::Matrix<double, 18, 1> qInit;
  qInit << krang->imu, robot->getPositions().tail(17);
  getSimple(threeDOF, qInit);   
  
  threeDOF->getJoint(0)->setDampingCoefficient(0, 0.5);
  threeDOF->getJoint(1)->setDampingCoefficient(0, 0.5);

  // Get it into a useful configuration
  double psiInit = 0, qBody1Init = 0;
  Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  // RotX(pi/2)*RotY(-pi/2+psi)*RotX(-qBody1)
  baseTf.prerotate(Eigen::AngleAxisd(-qBody1Init,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+psiInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));
  Eigen::Matrix<double, 8, 1> q;
//  q << 1.2092, -1.2092, -1.2092, 0, 0, 0.28, 0, 0;
  q << aa.angle()*aa.axis(), 0, 0, 0.28, 0, 0;
  threeDOF->setPositions(q);

  return threeDOF;
}


void getSimple(SkeletonPtr threeDOF, Eigen::Matrix<double, 18, 1> q) 
{
  // Load the full body with fixed wheel and set the pose q
  dart::utils::DartLoader loader;
  SkeletonPtr krangFixedWheel =
      loader.parseSkeleton("/home/krang/dart/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf");
  krangFixedWheel->setName("m18DOF");
  krangFixedWheel->setPositions(q);
  
  // Body Mass
  double mFull = krangFixedWheel->getMass(); 
  double mLWheel = krangFixedWheel->getBodyNode("LWheel")->getMass();
  double mBody = mFull - mLWheel;

  // Body COM
  Eigen::Vector3d bodyCOM;
  dart::dynamics::Frame* baseFrame = krangFixedWheel->getBodyNode("Base");
  bodyCOM = (mFull*krangFixedWheel->getCOM(baseFrame) - mLWheel*krangFixedWheel->getBodyNode("LWheel")->getCOM(baseFrame))/(mFull - mLWheel);

  // Body inertia
  int nBodies = krangFixedWheel->getNumBodyNodes();
  Eigen::Matrix3d iMat;
  Eigen::Matrix3d iBody = Eigen::Matrix3d::Zero();
  double ixx, iyy, izz, ixy, ixz, iyz;  
  Eigen::Matrix3d rot;
  Eigen::Vector3d t;
  Eigen::Matrix3d tMat;
  dart::dynamics::BodyNodePtr b;
  double m;
  for(int i=1; i<nBodies; i++){ // Skipping LWheel
    b = krangFixedWheel->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation(); 
    t = bodyCOM - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
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
  cout << "mass: " << mBody << endl;
  cout << "COM: " << bodyCOM(0) << ", " << bodyCOM(1) << ", " << bodyCOM(2) << endl;
  cout << "ixx: " << iBody(0,0) << ", iyy: " << iBody(1,1) << ", izz: " << iBody(2,2) << endl;
  cout << "ixy: " << iBody(0,1) << ", ixz: " << iBody(0,2) << ", iyz: " << iBody(1,2) << endl;
}

void *MPCDDPCompute(void *) {
	ControlTrajectory ddp_ctl_traj;
	StateTrajectory ddp_state_traj;
	Dynamics *ddp_dyn;
	Control u;

	int steps;
    int mpc_steps;
    double mpc_dt = 0.01;
	while(!initDDP);
	SkeletonPtr threeDOF = create3DOF_URDF();
	Eigen::VectorXd qInit;
	qInit << krang->imu, robot->getPositions().tail(17);

	param p; 
    double ixx, iyy, izz, ixy, ixz, iyz; 
    Eigen::Vector3d com;
    Eigen::Matrix3d iMat;      
    Eigen::Matrix3d tMat;
    dart::dynamics::Frame* baseFrame = m3DOF->getBodyNode("Base");
    p.R = 2.500000e-01; p.L = 6.000000e-01; p.g=9.800000e+00;
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
    Scalar tf = 20;
    auto time_steps = util::time_steps(tf, mpc_dt);
    int max_iterations = 15;             
      

    ddp_dyn = new Dynamics(p);
       // Dynamics ddp_dyn(p);

      // Initial state th, dth, x, dx, desired state, initial control sequence

    //x, psi, theta, dx, dpsi, dtheta, x0, y0
    State x0; x0 << state(2),state(4),state(0),state(3),state(5),state(1),AugState;       
    Dynamics::State xf; xf << 2, 0, 0, 0, 0, 0, 0.01, 5;
    Dynamics::ControlTrajectory u = Dynamics::ControlTrajectory::Zero(2, time_steps);

      // Costs
    Cost::StateHessian Q;
    Q.setZero();
    Q.diagonal() << 0,0.1,0.1,0.1,0.1,0.1,0.1,0.1;

    Cost::ControlHessian R;
    R.setZero();
    R.diagonal() << 0.01, 0.01;

    TerminalCost::Hessian Qf;
    Qf.setZero();
    Qf.diagonal() << 0,1e4,1e4,1e4,1e4,1e4,1e4,1e4;

    Cost cp_cost(xf, Q, R);
    TerminalCost cp_terminal_cost(xf, Qf);
      // initialize DDP for trajectory planning
    DDP_Opt trej_ddp (mpc_dt, time_steps, max_iterations, &logger, verbose);


      // Get initial trajectory from DDP
    OptimizerResult<Dynamics> DDP_traj = trej_ddp.run(x0, u, *ddp_dyn, cp_cost, cp_terminal_cost);

    ddp_state_traj = DDP_traj.state_trajectory;
    ddp_ctl_traj = DDP_traj.control_trajectory;
 	
    MODE = 7;
    if(!norm(ddp_state_traj.block<2,1>(6,ddp_state_traj.cols()) - xf.tail(2)) < 0.1){
    	initDDP = false;
    	MODE = 4;
    }


    writer.save_trajectory(ddp_state_traj, ddp_ctl_traj, "initial_traj.csv");
    else
    {
    	t = aa_tm_now();
    	for(i=t;t<t+tf;t+mpc_dt){
			mpc_steps = cur_mpc_steps; ///  Define mpcsteps  
        	int max_iterations = 15; 
        	bool verbose = true; 
        	util::DefaultLogger logger;
        	int mpc_horizon = 10; 
        
        	Dynamics::State target_state;
        	target_state = ddp_state_traj.col(mpc_steps + mpc_horizon);
        	Dynamics::ControlTrajectory hor_control = Dynamics::ControlTrajectory::Zero(2, mpc_horizon);
        	Dynamics::StateTrajectory hor_traj_states = ddp_state_traj.block(0, mpc_steps, 8, mpc_horizon);
        
        	DDP_Opt ddp_horizon (mpc_dt, mpc_horizon, max_iterations, &logger, verbose);
        
        	Cost::StateHessian Q_mpc, Qf_mpc;
        	Cost::ControlHessian ctl_R;
        
        	ctl_R.setZero();
        	ctl_R.diagonal() << 0.01, 0.01;
        	Q_mpc.setZero();
        	Q_mpc.diagonal() << 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        	Qf_mpc.setZero();
        	Qf_mpc.diagonal() << 0, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4;
        	Cost running_cost_horizon(target_state, Q_mpc, ctl_R);
        	TerminalCost terminal_cost_horizon(target_state, Qf_mpc);
        
        	OptimizerResult<Dynamics> results_horizon;
        	results_horizon.control_trajectory = hor_control;
        
        	results_horizon = ddp_horizon.run_horizon(cur_state, hor_control, hor_traj_states, *ddp_dyn, running_cost_horizon, terminal_cost_horizon);
        	u = results_horizon.control_trajectory.col(0);
        	uglobal = {u(0), u(1)};

        	mpc_writer.save_step(cur_state, u);



			double ddth = u(0);
        	double tau_0 = u(1);
        	State xdot = ddp_dyn->f(cur_state, u);
        	double ddx = xdot(3);
        	double ddpsi = xdot(4);
        	Eigen::Vector3d ddq, dq;
        	ddq << ddx, ddpsi, ddth;
        	dq = cur_state.segment(3,3);
        	c_forces dy_forces = ddp_dyn->dynamic_forces(cur_state, u);
        	//double tau_1 = (dy_forces.A.block<1,3>(2,0)*ddq) + (dy_forces.C.block<1,3>(2,0)*dq) + (dy_forces.Q(2)) - (dy_forces.Gamma_fric(2));
        	double tau_1 = dy_forces.A.block<1,3>(2,0)*ddq;
        	tau_1 += dy_forces.C.block<1,3>(2,0)*dq;
        	tau_1 += dy_forces.Q(2);
        	tau_1 -= dy_forces.Gamma_fric(2);
        	tau_L = -0.5*(tau_1+tau_0);
        	tau_R = -0.5*(tau_1-tau_0);

        	uglobal = {tau_L,tau_R};
        }

    }
    MODE = 4;




}
