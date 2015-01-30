#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <velvet_msgs/GripperState.h>
#include <ft_msgs/FTArray.h>
#include <experiment_console/ExperimentResult.h>
#include <velvet_interface_node/VelvetToPos.h>
#include <velvet_interface_node/SmartGrasp.h>

#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>



class ExpConsoleNode {

    private:
	// Our NodeHandle, points to home
	ros::NodeHandle nh_;
	//global node handle
	ros::NodeHandle n_;

	ros::Publisher result_pub;
	ros::Subscriber js_sub, vs_sub, ft_sub;

	ros::ServiceClient request_pos_, request_grasp_;

	velvet_interface_node::SmartGrasp graspcall;
	velvet_interface_node::VelvetToPos poscall;

	boost::mutex data_mutex;
	boost::thread exp_thread;

	experiment_console::ExperimentResult result;

    public:
	ExpConsoleNode() {
	    nh_ = ros::NodeHandle("~");
	    n_ = ros::NodeHandle();
	    
	    nh_.param("contact_thresh", graspcall.request.current_threshold_contact, 100);
	    nh_.param("final_thresh", graspcall.request.current_threshold_final, 200);
	    nh_.param("belt_travel", graspcall.request.max_belt_travel_mm, 200);
	    double tmp;
	    nh_.param("phalange_gelta", tmp, 0.01);
	    graspcall.request.phalange_delta_rad = tmp;
	    nh_.param("gripper_closed_thresh", tmp, 1.6);
	    graspcall.request.gripper_closed_thresh = tmp;
	    bool tmp2;
	    nh_.param("check_phalanges", tmp2, true);
	    graspcall.request.check_phalanges = tmp2;

	    result_pub = ros::NodeHandle().advertise<experiment_console::ExperimentResult>("exp_results", 1);
	    js_sub = n_.subscribe("joint_states", 1, &ExpConsoleNode::jsCallback, this);
	    vs_sub = n_.subscribe("velvet_state", 1, &ExpConsoleNode::vsCallback, this);
	    ft_sub = n_.subscribe("ft_topic", 1, &ExpConsoleNode::ftCallback, this);
	    
	    request_grasp_ = n_.serviceClient<velvet_interface_node::SmartGrasp>("/velvet_node/velvet_grasp");
	    request_pos_ = n_.serviceClient<velvet_interface_node::VelvetToPos>("/velvet_node/gripper_pos");

	    exp_thread = boost::thread(boost::bind(&ExpConsoleNode::run,this));
	}
	~ExpConsoleNode() {
	    //join thread...
	}

	void jsCallback( const sensor_msgs::JointStatePtr& msg) {
	        data_mutex.lock();
		result.joints = *msg;
	        data_mutex.unlock();
	}
	void vsCallback( const velvet_msgs::GripperStatePtr& msg) {
	        data_mutex.lock();
		result.velvet_state = *msg;
	        data_mutex.unlock();
	}
	void ftCallback( const ft_msgs::FTArrayPtr& msg) {
	        data_mutex.lock();
		result.ft_sensors = *msg;
	        data_mutex.unlock();
	}
	void run() {
	    std::cout<<"boost thread starting up...\n";
	    char c = 'a';
	    poscall.request.angle = 0;
	    while (c!='q' && c!='Q') {
		//call set zero service

		std::cout<<"calling pos service now!\n";
		if(!request_pos_.call(poscall)) {
		    std::cerr<<"could not call pos service\n";
		    return;
		}
		std::cout<<"wait for input...\n";

		//wait for input
		std::cin>>c;
		std::cout<<"calling grasp service now!\n";
		if(c == 'q') break;
		//call grasp service
		if(!request_grasp_.call(graspcall)) {
		    std::cerr<<"could not call grasp service\n";
		    return;
		}

		//wait for success 
		std::cin>>c;
		data_mutex.lock();
		switch(c) {
		    case 's':
			result.success = true;
			break;
		    default:
			result.success = false;
			break;
		};

		result_pub.publish(result);
		data_mutex.unlock();
	
			
	    }

	}
};

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "experiment_console");
    ExpConsoleNode consoleNode;
    ros::spin();
    return 0;
}



