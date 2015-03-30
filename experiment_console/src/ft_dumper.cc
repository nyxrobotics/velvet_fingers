#include <ros/ros.h>

#include <velvet_msgs/GripperState.h>
#include <velvet_msgs/ContactPointArray.h>
#include <ft_msgs/FTArray.h>
#include <experiment_console/SetOutfile.h>

#include <fstream>
#include <boost/thread.hpp>

struct FTDumpResult {
    velvet_msgs::ContactPointArray contact_point;
    velvet_msgs::GripperState velvet_state;
    ft_msgs::FTArray ft_sensors;
};

class FTDumper {

    private:
	// Our NodeHandle, points to home
	ros::NodeHandle nh_;
	//global node handle
	ros::NodeHandle n_;

	ros::Subscriber cp_sub;
	ros::Subscriber vs_sub;
	ros::Subscriber ft_sub;
	ros::ServiceServer out_srv;
	boost::mutex data_mutex;

	std::ofstream fst;
	std::string outfile_name;
	FTDumpResult result;
	bool newcp, newvs, newft;
	bool isOpen;
    public:
	FTDumper() {
	    nh_ = ros::NodeHandle("~");
	    n_ = ros::NodeHandle();
	    
	    nh_.param<std::string>("outfile_name",outfile_name,"results.txt");

	    cp_sub = n_.subscribe("contact_points", 1, &FTDumper::cpCallback, this);
	    vs_sub = n_.subscribe("velvet_state", 1, &FTDumper::vsCallback, this);
	    ft_sub = n_.subscribe("ft_topic", 1, &FTDumper::ftCallback, this);
	    
	    out_srv = nh_.advertiseService("set_output", &FTDumper::setOut, this);
	    
	    newcp=false;
	    newvs=false;
	    newft=false;
	    bool isOpen = false;
	}
	bool setOut(experiment_console::SetOutfile::Request &req,
		experiment_console::SetOutfile::Response &res) {
    	    if(isOpen) fst.close();

	    fst.open(req.outname.c_str(), std::ofstream::out);
	    isOpen=true;
	    newcp=false;
	    newvs=false;
	    newft=false;
	    fst <<"# gripper experiments result file. format is:\n";
	    fst <<"# open_enc, blb, blf, brb, brf, pl, pr, curr_open, curr_blb, curr_blf, curr_brb, curr_brf, ft_blb [fx,fy,fz,tx,ty,tz,cpx,cpy], ft_blf, ft_brb, ft_brf\n";

	    return true;
	}
	void cpCallback( const velvet_msgs::ContactPointArrayPtr& msg) {
	        data_mutex.lock();
		result.contact_point = *msg;
		newcp=true;
		if(newvs && newft && newcp) dumpData();
	        data_mutex.unlock();
	}
	void vsCallback( const velvet_msgs::GripperStatePtr& msg) {
	        data_mutex.lock();
		result.velvet_state = *msg;
		newvs=true;
		if(newvs && newft && newcp) dumpData();
	        data_mutex.unlock();
	}
	void ftCallback( const ft_msgs::FTArrayPtr& msg) {
	        data_mutex.lock();
		result.ft_sensors = *msg;
		newft=true;
		if(newvs && newft && newcp) dumpData();
	        data_mutex.unlock();
	}

	void dumpData() {
	    if(!isOpen) return;
	    newcp=false;
	    //newvs=false;
	    newft=false;
	    
	    fst<<result.velvet_state.oc.val<<","<<result.velvet_state.blb.val<<","<<result.velvet_state.blf.val<<","<<
		result.velvet_state.brb.val<<","<<result.velvet_state.brf.val<<","<<result.velvet_state.pl.val<<","<<result.velvet_state.pr.val<<","<<
		result.velvet_state.c_oc.val<<","<<result.velvet_state.c_blb.val<<","<<result.velvet_state.c_blf.val<<","<<result.velvet_state.c_brb.val<<","<<
		result.velvet_state.c_brf.val;
	    for(int i=0; i<result.ft_sensors.sensor_data.size(); ++i) {
		fst<<","<<result.ft_sensors.sensor_data[i].fx<<","<<result.ft_sensors.sensor_data[i].fy<<","<<result.ft_sensors.sensor_data[i].fz<<","
		    <<result.ft_sensors.sensor_data[i].tx<<","<<result.ft_sensors.sensor_data[i].ty<<","<<result.ft_sensors.sensor_data[i].tz<<","<<result.contact_point.points[i].x<<","<<result.contact_point.points[i].y;
	    }
	    fst<<std::endl;
	}

	~FTDumper() {
	}

#if 0
	void run() {
	    std::cout<<"boost thread starting up...\n";
	    ROS_WARN("waiting for service...");
	    ros::service::waitForService("/velvet_node/velvet_grasp");

	    fst <<"# gripper experiments result file. format is:\n";
	    fst<<"Success(0/1), open_enc, blb, blf, brb, brf, pl, pr, curr_open, curr_blb, curr_blf, curr_brb, curr_brf, ft_blb [fx,fy,fz,tx,ty,tz], ft_blf, ft_brb, ft_brf, tx, ty, tz, roll, pitch, yaw\n";
	    char c = 'a';
	    poscall.request.angle = 0.1;
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
		    tl.waitForTransform(object_frame_name, gripper_frame_name, ros::Time(0), ros::Duration(1.0) );
		    tl.lookupTransform(object_frame_name, gripper_frame_name, ros::Time(0), palm2obj_tf);
		} catch (tf::TransformException ex) {
		    ROS_ERROR("%s",ex.what());
		    return;
		}
		data_mutex.lock();
		Eigen::Affine3d palm2obj_eig;
		tf::transformTFToEigen(palm2obj_tf, palm2obj_eig);
		tf::transformEigenToMsg(palm2obj_eig, result.t_world);
		
		std::cout<<"grasp service done. Was it succesful? (y=yes, n=no)\n";
		//wait for success 
		std::cin>>c;
		bool meaningful = true;
		switch(c) {
		    case 'y':
			result.success = true;
			break;
		    case 'n':
			result.success = false;
			meaningful = true;
			break;
		    default:
			result.success = false;
			meaningful = false;
			break;
		};
		if(!meaningful) {
		    fst<<"3";
		} else {
		fst<<(int)result.success;
		}
		fst<<","<<result.velvet_state.oc.val<<","<<result.velvet_state.blb.val<<","<<result.velvet_state.blf.val<<","<<
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
#endif
};

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "dump");
    FTDumper consoleNode;
    ros::spin();
    return 0;
}



