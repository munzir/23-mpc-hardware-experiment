/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

//#include "helpers.h"
/*

#include <boost/circular_buffer.hpp>
// #include <ddp/costs.hpp>
// #include <ddp/ddp.hpp>
// #include <ddp/mpc.hpp>
// #include <ddp/util.hpp>



#include <iostream>
#include <fstream>
#include <sstream>

using namespace Eigen;
//using namespace Eigen::Dense;
using namespace std;
using namespace Krang;
//using namespace config4cpp;

*/
/* ****************************************** */


/*

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


void ComputeDDPTrajectory(Vector6d& state, Vector2d& AugState) {
  
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
  State x0; x0 << state(2),state(4),state(0),state(3),state(5),state(1),AugState;
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
	
  /*if(!norm(mDDPStateTraj.block<2,1>(6,mDDPStateTraj.cols()) - mGoalState.tail(2)) < 0.1){
    	initDDP = false;
    	MODE = 4;
  }
  else
  {*//*
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
      counterc = 0;
			t_now = aa_tm_now();
			double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));				
			i = i+dt;		
    }
 // }
  MODE = 4;
}*/
   //  for(i=t;t<t+tf;t+mpc_dt){
			// mpc_steps = cur_mpc_steps; ///  Define mpcsteps  
   //    int max_iterations = 15; 
   //    bool verbose = true; 
   //    util::DefaultLogger logger;
   //    int mpc_horizon = 10; 
        
   //    Dynamics::State target_state;
   //    target_state = ddp_state_traj.col(mpc_steps + mpc_horizon);
   //    Dynamics::ControlTrajectory hor_control = Dynamics::ControlTrajectory::Zero(2, mpc_horizon);
   //    Dynamics::StateTrajectory hor_traj_states = ddp_state_traj.block(0, mpc_steps, 8, mpc_horizon);
      
   //    DDP_Opt ddp_horizon (mpc_dt, mpc_horizon, max_iterations, &logger, verbose);
        
   //    Cost::StateHessian Q_mpc, Qf_mpc;
   //    Cost::ControlHessian ctl_R;
        
   //    ctl_R.setZero();
   //    ctl_R.diagonal() << 0.01, 0.01;
   //    Q_mpc.setZero();
   //    Q_mpc.diagonal() << 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
   //    Qf_mpc.setZero();
   //    Qf_mpc.diagonal() << 0, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4;
   //    Cost running_cost_horizon(target_state, Q_mpc, ctl_R);
   //    TerminalCost terminal_cost_horizon(target_state, Qf_mpc);
        
   //    OptimizerResult<Dynamics> results_horizon;
   //    results_horizon.control_trajectory = hor_control;
        
   //    results_horizon = ddp_horizon.run_horizon(cur_state, hor_control, hor_traj_states, *ddp_dyn, running_cost_horizon, terminal_cost_horizon);
   //    u = results_horizon.control_trajectory.col(0);
   //    uglobal = {u(0), u(1)};

   //    mpc_writer.save_step(cur_state, u);



			// double ddth = u(0);
   //    double tau_0 = u(1);
   //    State xdot = ddp_dyn->f(cur_state, u);
   //    double ddx = xdot(3);
   //    double ddpsi = xdot(4);
   //    Eigen::Vector3d ddq, dq;
   //    ddq << ddx, ddpsi, ddth;
   //    dq = cur_state.segment(3,3);
   //    c_forces dy_forces = ddp_dyn->dynamic_forces(cur_state, u);
   //    //double tau_1 = (dy_forces.A.block<1,3>(2,0)*ddq) + (dy_forces.C.block<1,3>(2,0)*dq) + (dy_forces.Q(2)) - (dy_forces.Gamma_fric(2));
   //    double tau_1 = dy_forces.A.block<1,3>(2,0)*ddq;
   //    tau_1 += dy_forces.C.block<1,3>(2,0)*dq;
   //    tau_1 += dy_forces.Q(2);
   //    tau_1 -= dy_forces.Gamma_fric(2);
   //    tau_L = -0.5*(tau_1+tau_0);
   //   	tau_R = -0.5*(tau_1-tau_0);

   //   	uglobal = {tau_L,tau_R};
   //  }

  // }
  // MODE = 4;
// }
