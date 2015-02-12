#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/JointState.h>
#include <velvet_interface_node/SmartGrasp.h>
#include <velvet_interface_node/VelvetToPos.h>
#include <velvet_msgs/GripperState.h>
#include <velvet_msgs/SetPos.h>
#include <velvet_msgs/SetCur.h>
#include <velvet_msgs/SetPID.h>
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

class VelvetGripperNode
{
public:
  VelvetGripperNode();
  virtual ~VelvetGripperNode();

private:
  ros::NodeHandle nh_, n_;

  ros::Publisher gripper_status_publisher_;
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
  float ftb1[6], ftb2[6], ftb3[6], ftb4[6];
  bool use_ft;
  bool limit_closing_speed;
  double closing_speed;
  double min_ramp_time;
  double contact_force_t;
  double my_vel;
  double prev_oangle;
  double t_prev_read,t_prev_send;
  double target_vel;
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
    data_mutex.lock();
    //update ft sensor states
    for(int i=0; i<msg->sensor_data.size(); ++i) {
	float *targ=NULL;
	switch(msg->sensor_data[i].id) {
	    case 0:
		targ = ftb1;
		break;
	    case 1:
		targ = ftb2;
		break;
	    case 2:
		targ = ftb3;
		break;
	    case 3:
		targ = ftb4;
		break;
	    default:
		break;
	};
	if(targ!=NULL) {
	    targ[0] = msg->sensor_data[i].fx;
	    targ[1] = msg->sensor_data[i].fy;
	    targ[2] = msg->sensor_data[i].fz;
	    targ[3] = msg->sensor_data[i].tx;
	    targ[4] = msg->sensor_data[i].ty;
	    targ[5] = msg->sensor_data[i].tz;
	}
    }
    data_mutex.unlock();
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
    curcall.request.id = 0;
    curcall.request.curr = 5;
    curcall.request.time = 1000; //2 seconds
    if(!set_cur_.call(curcall)) {
	std::cerr<<"Could not call set_cur service\n";
	this->finishGrasp(res, initial_angle, success_grasp);
	return true;
    }
    //sleep for ramp to start
    usleep(1500000);
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
	fb3(0)=ftb3[0];
	fb3(1)=ftb3[1];
	fb3(2)=ftb3[2];
	fb4(0)=ftb4[0];
	fb4(1)=ftb4[1];
	fb4(2)=ftb4[2];
	my_angle_last = my_angle;
	data_mutex.unlock();
	//wait for force magnitude > thresh 
	while(fb3.norm() < contact_force_t || fb4.norm() < contact_force_t) {

	    data_mutex.lock();
	    fb3(0)=ftb3[0];
	    fb3(1)=ftb3[1];
	    fb3(2)=ftb3[2];
	    fb4(0)=ftb4[0];
	    fb4(1)=ftb4[1];
	    fb4(2)=ftb4[2];
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

 
