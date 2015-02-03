#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <velvet_msgs/GripperState.h>
#include <ft_msgs/FTArray.h>
#include <geometry_msgs/Transform.h>
#include <experiment_console/ExperimentResult.h>
#include <velvet_interface_node/VelvetToPos.h>
#include <velvet_interface_node/SmartGrasp.h>

#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>


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
        tf::TransformListener tl;

	boost::mutex data_mutex;
	boost::thread exp_thread;

	experiment_console::ExperimentResult result;
	std::string object_frame_name, gripper_frame_name, outfile_name;

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
	    nh_.param<std::string>("gripper_frame_name",gripper_frame_name,"velvet_fingers_palm");
	    nh_.param<std::string>("object_frame_name",object_frame_name,"object_frame");
	    nh_.param<std::string>("outfile_name",outfile_name,"results.txt");

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
	    std::ofstream fst (outfile_name.c_str(), std::ofstream::out);
	    fst <<"# gripper experiments result file. format is:\n";
	    fst<<"Success(0/1), open_enc, blb, blf, brb, brf, pl, pr, curr_open, curr_blb, curr_blf, curr_brb, curr_brf, ft_blb [fx,fy,fz,tx,ty,tz], ft_blf, ft_brb, ft_brf, tx, ty, tz, roll, pitch, yaw\n";
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

		//get the transform of the gripper palm relative to the object frame
		tf::StampedTransform palm2obj_tf;
		try {
		    tl.waitForTransform(gripper_frame_name, object_frame_name, ros::Time(0), ros::Duration(1.0) );
		    tl.lookupTransform(gripper_frame_name, object_frame_name, ros::Time(0), palm2obj_tf);
		} catch (tf::TransformException ex) {
		    ROS_ERROR("%s",ex.what());
		    return;
		}
		Eigen::Affine3d palm2obj_eig;
		tf::transformTFToEigen(palm2obj_tf, palm2obj_eig);
		tf::transformEigenToMsg(palm2obj_eig, result.t_world);
		
		std::cout<<"grasp service done. Was it succesful? (y=yes, n=no)\n";
		//wait for success 
		std::cin>>c;
		data_mutex.lock();
		switch(c) {
		    case 'y':
			result.success = true;
			break;
		    default:
			result.success = false;
			break;
		};
		fst<<(int)result.success<<","<<result.velvet_state.oc.val<<","<<result.velvet_state.blb.val<<","<<result.velvet_state.blf.val<<","<<
		    result.velvet_state.brb.val<<","<<result.velvet_state.brf.val<<","<<result.velvet_state.pl.val<<","<<result.velvet_state.pr.val<<","<<
		    result.velvet_state.c_oc.val<<","<<result.velvet_state.c_blb.val<<","<<result.velvet_state.c_blf.val<<","<<result.velvet_state.c_brb.val<<","<<
		    result.velvet_state.c_brf.val<<",";
		for(int i=0; i<result.ft_sensors.sensor_data.size(); ++i) {
		    fst<<result.ft_sensors.sensor_data[i].fx<<","<<result.ft_sensors.sensor_data[i].fy<<","<<result.ft_sensors.sensor_data[i].fz<<","
			<<result.ft_sensors.sensor_data[i].tx<<","<<result.ft_sensors.sensor_data[i].ty<<","<<result.ft_sensors.sensor_data[i].tz<<",";
		}
		Eigen::Vector3d t,r;
		t = palm2obj_eig.translation();
		r = palm2obj_eig.rotation().eulerAngles(0,1,2);
		fst<<t(0)<<","<<t(1)<<","<<t(2)<<","<<r(0)<<","<<r(1)<<","<<r(2)<<"\n";

		result_pub.publish(result);
		data_mutex.unlock();
	
			
	    }
	    fst.close();

	}
};

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "experiment_console");
    ExpConsoleNode consoleNode;
    ros::spin();
    return 0;
}



