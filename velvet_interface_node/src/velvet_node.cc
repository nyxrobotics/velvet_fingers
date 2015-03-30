#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/JointState.h>
#include <velvet_interface_node/SmartGrasp.h>
#include <velvet_interface_node/VelvetToPos.h>
#include <velvet_msgs/GripperState.h>
#include <velvet_msgs/SetPos.h>
#include <velvet_msgs/SetCur.h>
#include <velvet_msgs/SetPID.h>
#include <velvet_msgs/ContactPoint.h>
#include <velvet_msgs/ContactPointArray.h>
#include <ft_msgs/FTArray.h>
#include <velvet_msgs/VNodeState.h>
#include <velvet_msgs/VNodeTarget.h>


#include <boost/thread/mutex.hpp>
#include <qhull/qhull.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <velvet_interface_node/velvet_nodeConfig.h>
#define BE_HACKED

class VelvetGripperNode
{
public:
  VelvetGripperNode();
  virtual ~VelvetGripperNode();

private:
  ros::NodeHandle nh_, n_;

  ros::Publisher gripper_status_publisher_;
  ros::Publisher cp_pub_;
  ros::Publisher vnode_state_pub_;
  ros::Subscriber arduino_state_sub_;
  ros::Subscriber force_torque_sub_;
  ros::Subscriber vnode_target_sub_;

  ros::ServiceServer request_grasp_;
  ros::ServiceServer request_pos_;
  
  ros::ServiceClient set_pid_;
  ros::ServiceClient set_pos_;
  ros::ServiceClient set_cur_;

  std::string velvet_state_topic, ft_sensors_topic;
  std::string set_pos_name, set_cur_name, set_pid_name;
  std::string vnode_state_topic, vnode_target_topic;

  //state variables
  boost::mutex data_mutex;
  float my_angle, finger1_angle, finger2_angle, belt1_pos, belt2_pos, belt3_pos, belt4_pos, 
	open_curr, belt1_curr, belt2_curr, belt3_curr, belt4_curr;
  Eigen::Matrix<double,6,1> ftb1, ftb2, ftb3, ftb4;
  Eigen::Matrix<double,6,6> Wlc, Wrc, Wp;
  Eigen::Vector3d cpb1, cpb2, cpb3, cpb4; //contact point locations
  Eigen::Vector3d fnb1, fnb2, fnb3, fnb4; //normalized forces on belts
  double tb1, tb2, tb3, tb4; //torque in local frame
  bool use_ft;
  bool limit_closing_speed;
  double closing_speed;
  double min_ramp_time;
  double contact_force_t;
  double my_vel;
  double prev_oangle;
  double t_prev_read,t_prev_send;
  double target_vel;
  double one_beer;
  bool doVelocityControl;
  bool bHWIfce;
  double setpoint;

  //reconfigure stuff
  dynamic_reconfigure::Server<velvet_interface_node::velvet_nodeConfig> dr_srv;
  dynamic_reconfigure::Server<velvet_interface_node::velvet_nodeConfig>::CallbackType cb;
  ros::Timer heartbeat_setpos_;

  bool request_grasp(velvet_interface_node::SmartGrasp::Request  &req,
             velvet_interface_node::SmartGrasp::Response &res );
  bool request_pos(velvet_interface_node::VelvetToPos::Request  &req,
             velvet_interface_node::VelvetToPos::Response &res );
 
  //callbacks
  void stateCallback( const velvet_msgs::GripperStatePtr& msg); 
  void ftCallback( const ft_msgs::FTArrayPtr& msg); 
  void publishStatus();

  bool isValidGrasp(float open_angle, float open_angle_thresh, float p1, float p2, float min_phalange_delta, bool check_phalanges);
  bool isForceClosure();
  void finishGrasp(velvet_interface_node::SmartGrasp::Response &res, float init_angle, bool success);

  void setVel( const velvet_msgs::VNodeTarget& msg);
  void sendSetPosForVelControl(const ros::TimerEvent& event) {
   
      if(doVelocityControl) {
	//update setpoint for position controller
	  double tnow = getDoubleTime();
	  double dt = tnow - t_prev_send;
	  t_prev_send=tnow;

	  double kp = 0;
	  double eps = 0.05;
	  setpoint = my_angle + dt*target_vel + kp*(target_vel-my_vel);
	  if(setpoint < eps) setpoint = 0;
	  if(setpoint > M_PI/2-eps) setpoint = M_PI/2-eps;
	  velvet_msgs::SetPos poscall;
	  poscall.request.id = 0;
	  data_mutex.lock();
	  poscall.request.pos = setpoint * 1000.; //mRad
	  data_mutex.unlock();
	  poscall.request.time = dt*1000; // who knows?

	  set_pos_.call(poscall);
      }

  }
  double getDoubleTime() {
      struct timeval time;
      gettimeofday(&time,NULL);
      return time.tv_sec + time.tv_usec * 1e-6; 
  }
  void configCallback(velvet_interface_node::velvet_nodeConfig &config, uint32_t level)
  {
      if(config.cid >=0 ) {
	  std::cout<<"setting params of "<<config.cid<<" to "<<config.kp<<" "<<config.ki<<" "<<config.kd<<" "<<config.pos_mode<<std::endl;
	  velvet_msgs::SetPID pidcall;
	  pidcall.request.id = config.cid;
	  pidcall.request.pval = config.kp;
	  pidcall.request.ival = config.ki;
	  pidcall.request.dval = config.kd;
	  pidcall.request.mode = config.pos_mode ? 'P' : 'C';
	  if (!set_pid_.call(pidcall)) {
	      ROS_ERROR("Unable to call set PID service!");
	  }
      }
  } 
  bool calculateFT(Eigen::Matrix<double,6,1> &orig, Eigen::Vector3d &contact_point, Eigen::Vector3d &contact_force, double &t_norm);
};


VelvetGripperNode::VelvetGripperNode()
{
  std::cerr<<"starting node\n";
  nh_ = ros::NodeHandle("~");
  n_ = ros::NodeHandle();

  //read in parameters
  nh_.param("use_ft", use_ft, false);
  nh_.param("bHWIfce", bHWIfce, false);
  nh_.param("limit_closing_speed", limit_closing_speed, false);
  nh_.param("closing_speed_rad_s", closing_speed, 0.5);
  nh_.param("min_ramp_time_ms", min_ramp_time, 1000.);
  nh_.param("contact_force_t", contact_force_t, 10.);
  nh_.param("one_beer", one_beer, 1.15);
  nh_.param<std::string>("velvet_topic_name", velvet_state_topic,"/velvet_state");
  nh_.param<std::string>("ft_topic_name", ft_sensors_topic,"/ft_topic");
  nh_.param<std::string>("set_pos_name", set_pos_name,"/set_pos");
  nh_.param<std::string>("set_cur_name", set_cur_name,"/set_cur");
  nh_.param<std::string>("set_pid_name", set_pid_name,"/set_pid");

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  cb = boost::bind(&VelvetGripperNode::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);
  
  request_grasp_ = nh_.advertiseService("velvet_grasp", &VelvetGripperNode::request_grasp, this);;
  request_pos_ = nh_.advertiseService("gripper_pos", &VelvetGripperNode::request_pos, this);;
  
  gripper_status_publisher_ = ros::NodeHandle().advertise<sensor_msgs::JointState>("joint_states", 10); 
  cp_pub_ = ros::NodeHandle().advertise<velvet_msgs::ContactPointArray>("contact_points", 10); 
  arduino_state_sub_ = n_.subscribe(velvet_state_topic, 10, &VelvetGripperNode::stateCallback, this);
  if(use_ft) {
      force_torque_sub_ = n_.subscribe(ft_sensors_topic, 10, &VelvetGripperNode::ftCallback, this);
  }
  
  set_pos_ = n_.serviceClient<velvet_msgs::SetPos>(set_pos_name);
  set_cur_ = n_.serviceClient<velvet_msgs::SetCur>(set_cur_name);
  set_pid_ = n_.serviceClient<velvet_msgs::SetPID>(set_pid_name);

  if(bHWIfce) {
      nh_.param<std::string>("vnode_state_topic", vnode_state_topic, "vnode_state");
      nh_.param<std::string>("vnode_target_topic",vnode_target_topic,"vnode_target");
      vnode_state_pub_ = ros::NodeHandle().advertise<velvet_msgs::VNodeState>(vnode_state_topic, 10); 
      vnode_target_sub_ = n_.subscribe(vnode_target_topic, 1, &VelvetGripperNode::setVel, this);
  }
  doVelocityControl = false;
  target_vel =0;
  t_prev_read = getDoubleTime();
  t_prev_send = getDoubleTime();
  heartbeat_setpos_ = nh_.createTimer(ros::Duration(2), &VelvetGripperNode::sendSetPosForVelControl, this);

  /////global matricse for FT calculation
  double lx=0.037,ly=0.0035,lz=0.019;
  Wlc.setIdentity();
  Wlc(4,0) = -lz;
  Wlc(5,0) = ly;
  Wlc(3,1) = lz;
  Wlc(5,1) = -lx;
  Wlc(3,2) = -ly;
  Wlc(4,2) = lx;

  double rx=0,ry=0,rz=0;
  Wrc.setIdentity();
  Wrc(4,0) = -rz;
  Wrc(5,0) = ry;
  Wrc(3,1) = rz;
  Wrc(5,1) = -rx;
  Wrc(3,2) = -ry;
  Wrc(4,2) = rx;
  
  double px=0.0,py=0.0035,pz=0.033;
  Wp.setIdentity();
  Wp(4,0) = -pz;
  Wp(5,0) = py;
  Wp(3,1) = pz;
  Wp(5,1) = -px;
  Wp(3,2) = -py;
  Wp(4,2) = px;

  std::cerr<<"Wp = "<<Wp<<std::endl;
}

VelvetGripperNode::~VelvetGripperNode()
{
}


void VelvetGripperNode::setVel( const velvet_msgs::VNodeTarget& msg) {
    //target vel is in msg->target_vel
    data_mutex.lock();
    target_vel = msg.target_vel;
    doVelocityControl = true;
    data_mutex.unlock();
}

void VelvetGripperNode::publishStatus() {

    if(!bHWIfce) {
	sensor_msgs::JointState js;
	js.header.stamp = ros::Time::now();
	js.name.push_back("velvet_fingers_joint_1");
	js.position.push_back(my_angle);
	js.name.push_back("velvet_fingers_joint_2");
	js.position.push_back(my_angle);

	gripper_status_publisher_.publish(js);
    } else {
	velvet_msgs::VNodeState msg;
	msg.joint_pos = my_angle;
	msg.joint_vel = my_vel;
	msg.joint_eff = open_curr;
	vnode_state_pub_.publish(msg);

    }
    return;
}

  
void VelvetGripperNode::stateCallback( const velvet_msgs::GripperStatePtr& msg) {
    data_mutex.lock();
    //update gripper state variables
    my_angle = msg->oc.val / 1000.; //from mRad
    finger1_angle = msg->pl.val ;
    finger2_angle = msg->pr.val ;
    belt1_pos = msg->blb.val ;
    belt2_pos = msg->brb.val ;
    belt3_pos = msg->blf.val ;
    belt4_pos = msg->brf.val ;

    open_curr = msg->c_oc.val ;
    belt1_curr = msg->c_blb.val ;
    belt2_curr = msg->c_brb.val ;
    belt3_curr = msg->c_blf.val ;
    belt4_curr = msg->c_brf.val;

    double tnow = getDoubleTime();
    double dt = tnow - t_prev_read;
    my_vel = (my_angle - prev_oangle) / dt;
    prev_oangle = my_angle;
    t_prev_read=tnow;
    
    //publish joint states
    publishStatus();
    data_mutex.unlock();

}

void VelvetGripperNode::ftCallback( const ft_msgs::FTArrayPtr& msg) {
    velvet_msgs::ContactPoint cp;
    velvet_msgs::ContactPointArray cps;
    Eigen::Vector3d cpt;
    data_mutex.lock();
    //update ft sensor states
    for(int i=0; i<msg->sensor_data.size(); ++i) {
	Eigen::Matrix<double,6,1> targ;
	targ(0,0) = msg->sensor_data[i].fx;
	targ(1,0) = msg->sensor_data[i].fy;
	targ(2,0) = msg->sensor_data[i].fz;
	targ(3,0) = msg->sensor_data[i].tx;
	targ(4,0) = msg->sensor_data[i].ty;
	targ(5,0) = msg->sensor_data[i].tz;
	cp.id = msg->sensor_data[i].id;    
	switch(msg->sensor_data[i].id) {
	    case 0:
		ftb1=targ;
		cpb1.setZero();
		cp.valid = (calculateFT(ftb1,cpb1,fnb1,tb1));
		cp.x = cpb1(0);
		cp.y = cpb1(1);
		cp.z = cpb1(2);
		cps.points.push_back(cp);
		break;
	    case 1:
		ftb2=targ;
		cpb2.setZero();
		cp.valid = calculateFT(ftb2,cpb2,fnb2,tb2);
		cp.x = cpb2(0);
		cp.y = cpb2(1);
		cp.z = cpb2(2);
		cps.points.push_back(cp);
		break;
	    case 2:
		ftb3=targ;
		cpb3.setZero();
		cp.valid = calculateFT(ftb3,cpb3,fnb3,tb3);
		cp.x = cpb3(0);
		cp.y = cpb3(1);
		cp.z = cpb3(2);
		cps.points.push_back(cp);
		break;
	    case 3:
		ftb4=targ;
		cpb4.setZero();
		cp.valid = (calculateFT(ftb4,cpb4,fnb4,tb4));
		cp.x = cpb4(0);
		cp.y = cpb4(1);
		cp.z = cpb4(2);
		cps.points.push_back(cp);
		break;
	    default:
		break;
	};
    }
    cp_pub_.publish(cps);
    data_mutex.unlock();
}

bool VelvetGripperNode::calculateFT(Eigen::Matrix<double,6,1> &orig, Eigen::Vector3d &contact_point, Eigen::Vector3d &contact_force, double &t_norm) {

    double min_contact_force = 1.8;
    Eigen::Matrix<double,6,1> w;
    w = Wp*orig;
    
    contact_force(0) = w(0);
    contact_force(1) = w(1);
    contact_force(2) = w(2);

    if(fabs(contact_force(2)) < min_contact_force) return false; 

    contact_point(0) = w(4)/w(2);
    contact_point(1) = -w(3)/w(2);
    contact_point(2) = 0.033; 

    t_norm = w(5) - w(1)*contact_point(0) + w(0)*contact_point(1);

    return true;
#if 0
    //left cylinder
    bool left_valid = true;
    double Rleft = 0.028;
    double min_contact_force = 1;
    Eigen::Matrix<double,6,1> wlc;
    wlc = Wlc*orig;

    //std::cerr<<"w: "<<orig.transpose()<<"\n Wlc: "<<Wlc<<"\nw transformed: "<<wlc.transpose()<<std::endl;
    contact_force(0) = wlc(4) / Rleft;
    contact_force(1) = wlc(1);
    contact_force(2) = wlc(0)*wlc(0)+wlc(2)*wlc(2) - contact_force(1)*contact_force(1);

    if(contact_force(2) > min_contact_force) {
	contact_force(2) = sqrt(contact_force(2));
	std::cerr<<"contact force: "<<contact_force.transpose()<<std::endl;
    } else {
	//std::cerr<<"contact force z zero\n";
	left_valid = false;
    }
    //left_valid &= (contact_force(0) >= 0 && contact_force(1) >= 0);

    double omega=0;
    double eps = 1e-2;
    double y;
    if(left_valid) {
	double omega_1, omega_2;
	omega_1 = 2*atan2(-contact_force(2) + wlc(2), wlc(0) + contact_force(0));
	omega_2 = 2*atan2(-contact_force(2) - wlc(2), wlc(0) + contact_force(0));
	if( fabs(wlc(2) + (contact_force(0)*sin(omega_1) + contact_force(2)*cos(omega_1))) < fabs(wlc(2) + (contact_force(0)*sin(omega_2) + contact_force(2)*cos(omega_2))) ) {
	    omega = omega_1;
	} else { 
	    omega = omega_2;
	}
	if(omega < -M_PI) omega +=2*M_PI;
	if(omega > M_PI) omega -=2*M_PI;
    } 
    //left_valid &= ((omega<M_PI/2 && omega >-M_PI/2) || omega > 3*M_PI/2);

    if(left_valid) {
	t_norm = (wlc(2)*wlc(5)+ wlc(0)*wlc(3) + wlc(1)*Rleft*(wlc(0)*cos(omega) - wlc(2)*sin(omega))) / (wlc(0)*sin(omega)+wlc(2)*cos(omega));
	y = (sin(omega)*t_norm - wlc(1)*Rleft*cos(omega) - wlc(3))/ (-wlc(2));

	contact_point(0) = y;
	contact_point(1) = wlc(2);
	contact_point(2) = wlc(3);

	//contact_point(0) = omega; //sin(omega)*Rleft + Wlc(4,2);
	//contact_point(1) = y; // + Wlc(5,0);
	//contact_point(2) = cos(omega)*Rleft + Wlc(3,1);

	velvet_msgs::ContactPoint cp;
	cp.x = contact_point(0);
	cp.y = contact_point(1);
	cp.z = contact_point(2);
	cp_pub_.publish(cp);
	std::cerr<<"contact point "<<contact_point.transpose()<<std::endl;
	return true;
    }
#endif
}
//checks if a grasp is stable
bool VelvetGripperNode::isValidGrasp(float open_angle, float open_angle_thresh, float p1, float p2, float min_phalange_delta, bool check_phalanges) {
    if(!use_ft) {
	if(open_angle > open_angle_thresh) return false;
	if(check_phalanges) {
	    if(p1 < min_phalange_delta && p2 < min_phalange_delta) return false;
	} 
    } else {
	return isForceClosure();
    }
    return true;
}
  
bool VelvetGripperNode::isForceClosure() {

}
  
void VelvetGripperNode::finishGrasp(velvet_interface_node::SmartGrasp::Response &res, float init_angle, bool success) {

    // swich off belts
    velvet_msgs::SetCur curcall;
    curcall.request.id = 1;
    curcall.request.curr = 0;
    curcall.request.time = 100; //0.1 sec
    set_cur_.call(curcall);

    //if a failed grasp, switch off open/close and go back to initial angle
    if(!success) {
	//set OC current to 0
	curcall.request.id = 0;
	curcall.request.curr = 0;
	curcall.request.time = 100; //0.1 sec
	set_cur_.call(curcall);

	//set OC pos to initial angle
	if(init_angle > 0 && init_angle < M_PI/2) {
	    velvet_msgs::SetPos poscall;
	    poscall.request.id = 0;
	    poscall.request.pos = init_angle * 1000.; //mRad
	    poscall.request.time = 3000; //3 sec
	    set_pos_.call(poscall);
	}
    }
    
    //fill in results
    res.success = success;
    res.opening_current = open_curr;
    res.opening_angle = my_angle;
    res.p1_angle = finger1_angle;
    res.p2_angle = finger2_angle;
    
}

bool VelvetGripperNode::request_pos(velvet_interface_node::VelvetToPos::Request  &req,
 	velvet_interface_node::VelvetToPos::Response &res ) {

    //check if req.angle is inside bounds
    if(req.angle < 0 || req.angle > M_PI/2) {
	return false;
    }
    ROS_INFO("Sending gripper to pos %f",req.angle);
    float my_angle_here;
    if(limit_closing_speed) {
	data_mutex.lock();
	my_angle_here = my_angle;
	data_mutex.unlock();
    }
    //send request on set_pos
    velvet_msgs::SetPos poscall;
    poscall.request.id = 0;
    poscall.request.pos = req.angle * 1000.; //mRad
    
    if(limit_closing_speed) {
	poscall.request.time = fabsf(my_angle_here-req.angle) / closing_speed;
	if(poscall.request.time < min_ramp_time) poscall.request.time = min_ramp_time;
    } else {
	poscall.request.time = 3000; //3 sec
    }
    set_pos_.call(poscall);
    return true;
}

bool VelvetGripperNode::request_grasp(velvet_interface_node::SmartGrasp::Request  &req,
    velvet_interface_node::SmartGrasp::Response &res ) {

    float curr_offset = 10;
    float current_threshold_contact = req.current_threshold_contact;
    float current_threshold_final = req.current_threshold_final;
    float ANGLE_CLOSED = req.gripper_closed_thresh;
    float MIN_PHALANGE_DELTA = req.phalange_delta_rad;
    float MAX_BELT_TRAVEL = req.max_belt_travel_mm; 
    bool CHECK_PHALANGES = req.check_phalanges;

    std::cerr<<"starting smart grasp, thresholds are "<<current_threshold_contact<<" "<<current_threshold_final<<std::endl;

    bool success_grasp = false;
    double ZERO_MOVEMENT_BELT = 0.01;
    int N_ZERO_BELTS = 10;
    int N_ZERO_OPEN = 5;
    double T_MAX_SEC = 25;
    float T_MIN_BELTS = 3000; //3 secs 
    double t_start = getDoubleTime();
    double tnow = 0, t_start_belts;
   
    data_mutex.lock(); 
    double initial_angle = my_angle;
    data_mutex.unlock(); 

    velvet_msgs::SetCur curcall;
    velvet_msgs::SetPos poscall;	
#ifdef BE_HACKED
    poscall.request.id = 1;
    poscall.request.pos = 2*MAX_BELT_TRAVEL;
    poscall.request.time = 2*T_MIN_BELTS; // 1/5 of remaining time
    ROS_INFO("belts should move %f mm in %f sec", poscall.request.pos, poscall.request.time);
    if(!set_pos_.call(poscall)) {
	    std::cerr<<"couldn't call pos service\n";
	    this->finishGrasp(res, initial_angle, false);
	    return true;
    }
#endif
    curcall.request.id = 0;
    curcall.request.curr = 5;
    curcall.request.time = 1000; //2 seconds
    if(!set_cur_.call(curcall)) {
	std::cerr<<"Could not call set_cur service\n";
	this->finishGrasp(res, initial_angle, success_grasp);
	return true;
    }
    //sleep for ramp to start
#ifdef BE_HACKED
    usleep(500000);
#else
    usleep(1500000);
#endif
    curcall.request.id = 0;
    curcall.request.curr = current_threshold_contact;
    curcall.request.time = 500; //2 seconds
    if(!set_cur_.call(curcall)) {
	std::cerr<<"Could not call set_cur service\n";
	this->finishGrasp(res, initial_angle, success_grasp);
	return true;
    }
    usleep(800000);
    float my_angle_last, belt1_pos_last, belt2_pos_last;
    float d_angle, d_belt1, d_belt2, d_finger1, d_finger2;
    int n_zero_belt1=0, n_zero_belt2=0, n_zero_open=0; 
    
    data_mutex.lock();
    my_angle_last = my_angle;
    belt1_pos_last = belt1_pos;
    belt2_pos_last = belt2_pos;
    data_mutex.unlock();
   
    if(!use_ft) {
	//monitor when opening angle stops changing 
	while(n_zero_open < N_ZERO_OPEN) {
	    data_mutex.lock();
	    d_angle = fabsf(my_angle - my_angle_last);
	    my_angle_last = my_angle;
	    data_mutex.unlock();

	    if(d_angle < 0.01) {
		n_zero_open++;
		std::cerr<<n_zero_open<<" "<<d_angle<<std::endl;
	    } else {
		n_zero_open = 0;
	    }
	    usleep(200000); //FIXME: this needs to be checked
	    tnow = getDoubleTime();
	    if(tnow - t_start > T_MAX_SEC) {
		std::cerr<<"TIMED OUT on approach\n";
		this->finishGrasp(res, my_angle_last, false);
		return true;
	    }
	}
    } else {
	//monitor the force vectors on belts 3 and 4
	Eigen::Vector3f fb3, fb4;
	data_mutex.lock();
	fb3(0)=ftb3(0,0);
	fb3(1)=ftb3(1,0);
	fb3(2)=ftb3(2,0);
	fb4(0)=ftb4(0,0);
	fb4(1)=ftb4(1,0);
	fb4(2)=ftb4(2,0);
	my_angle_last = my_angle;
	data_mutex.unlock();
	//wait for force magnitude > thresh 
	while(fb3.norm() < contact_force_t || fb4.norm() < contact_force_t) {

	    data_mutex.lock();
	    fb3(0)=ftb3(0,0);
	    fb3(1)=ftb3(1,0);
	    fb3(2)=ftb3(2,0);
	    fb4(0)=ftb4(0,0);
	    fb4(1)=ftb4(1,0);
	    fb4(2)=ftb4(2,0);
	    my_angle_last = my_angle;
	    data_mutex.unlock();

	    usleep(200000); //FIXME: this needs to be checked
	    tnow = getDoubleTime();
	    if(tnow - t_start > T_MAX_SEC) {
		std::cerr<<"TIMED OUT on approach\n";
		this->finishGrasp(res, my_angle_last, false);
		return true;
	    }

	}
    }

    data_mutex.lock();
    my_angle_last = my_angle;
    data_mutex.unlock();

    if(my_angle_last < ANGLE_CLOSED) {
	std::cerr<<"CONTACT!!\n Switch on belts!\n";
#ifdef BE_HACKED    
	curcall.request.id = 0;
	curcall.request.curr = current_threshold_contact+curr_offset;
	curcall.request.time = 500; //2 seconds
	if(!set_cur_.call(curcall)) {
		std::cerr<<"Could not call set_cur service\n";
		this->finishGrasp(res, initial_angle, success_grasp);
		return true;
	}
	usleep(800000);
	if(my_angle_last < one_beer) {
		t_start_belts = getDoubleTime();
		poscall.request.id = 2;
		poscall.request.pos = 2*MAX_BELT_TRAVEL/3;
		poscall.request.time = (T_MAX_SEC - (t_start_belts-t_start))*200; // 1/5 of remaining time
		if(poscall.request.time < T_MIN_BELTS) poscall.request.time = T_MIN_BELTS; //give it at least T_MIN seconds to avoid crazy jumps
		ROS_INFO("MOVING OPPOSITE DIRECTION: belts should move %f mm in %f sec", poscall.request.pos, poscall.request.time);
		if(!set_pos_.call(poscall)) {
			std::cerr<<"couldn't call pos service\n";
			this->finishGrasp(res, initial_angle, false);
			return true;
		}
		usleep(poscall.request.time*1000);
	}
#endif
	t_start_belts = getDoubleTime();
	poscall.request.id = 1;
	poscall.request.pos = MAX_BELT_TRAVEL;
	poscall.request.time = (T_MAX_SEC - (t_start_belts-t_start))*200; // 1/5 of remaining time
	if(poscall.request.time < T_MIN_BELTS) poscall.request.time = T_MIN_BELTS; //give it at least T_MIN seconds to avoid crazy jumps
	ROS_INFO("belts should move %f mm in %f sec", poscall.request.pos, poscall.request.time);
	if(!set_pos_.call(poscall)) {
	    std::cerr<<"couldn't call pos service\n";
	    this->finishGrasp(res, initial_angle, false);
	    return true;
	}
    } else {
	std::cerr<<"Probably NOTHING INSIDE GRIPPER, FAIL\n";
	this->finishGrasp(res, initial_angle, false);
	return true;
    }


    while(!success_grasp) {  
        data_mutex.lock();	
	d_belt1 = fabsf(belt1_pos_last - belt1_pos);
	d_belt2 = fabsf(belt2_pos_last - belt2_pos);
	belt1_pos_last = belt1_pos;
	belt2_pos_last = belt2_pos;
        data_mutex.unlock();	

	n_zero_belt1 = d_belt1 < ZERO_MOVEMENT_BELT ? n_zero_belt1+1 : 0; 
	n_zero_belt2 = d_belt2 < ZERO_MOVEMENT_BELT ? n_zero_belt2+2 : 0; 

	//give the belts 1 second to start moving	
	tnow = getDoubleTime();	
	if(tnow - t_start_belts < 1) {
	    n_zero_belt1 = 0;
	    n_zero_belt2 = 0;
	}
	
	//when grasp success - > done
        //conditions for a likely successful grasp:
	//	belts not moving anymore
	//	phalanges have angles above some threshold
	//	all current thresholds achieved	

	tnow = getDoubleTime();
	if((n_zero_belt1 > N_ZERO_BELTS && n_zero_belt2 > N_ZERO_BELTS) || tnow - t_start > T_MAX_SEC) {
	    std::cerr<<"BELTS HAVE BLOCKED (or time is up)!\n";
	    std::cerr<<"ENABLE AND SET CURRENT\n";
	    
	    //set oc current to final threshold
	    curcall.request.id = 0;
	    curcall.request.curr = current_threshold_final;
	    curcall.request.time = 2000; //2 seconds
	    if(!set_cur_.call(curcall)) {
		std::cerr<<"Could not call set_cur service\n";
		this->finishGrasp(res, initial_angle, success_grasp);
		return true;
	    }

	    //set belt current to 0
	    curcall.request.id = 1;
	    curcall.request.curr = 0;
	    curcall.request.time = 1000; //1 seconds
	    if(!set_cur_.call(curcall)) {
		std::cerr<<"Could not call set_cur service\n";
		this->finishGrasp(res, initial_angle, success_grasp);
		return true;
	    }
	    sleep(1);

	    data_mutex.lock();
	    my_angle_last = my_angle;
	    d_finger1 = fabsf(finger1_angle);
	    d_finger2 = fabsf(finger2_angle);
	    ROS_INFO("my angle is %f",my_angle);
	    data_mutex.unlock();

	    success_grasp = isValidGrasp(my_angle_last, ANGLE_CLOSED, d_finger1, d_finger2, MIN_PHALANGE_DELTA, CHECK_PHALANGES);
	    this->finishGrasp(res, initial_angle, success_grasp);
	    ROS_INFO("Grasping finished, success? %d",success_grasp); 
	    return true;
	}
	usleep(10000);
    }

    std::cerr<<"THIS SHOULD NOT HAPPEN\n";
    this->finishGrasp(res, initial_angle, success_grasp);
    return true;
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "velvet_node");
    VelvetGripperNode gripperNode;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    //ros::spin();
    return 0;
}

 
