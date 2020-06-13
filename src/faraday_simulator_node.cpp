#include <ros/ros.h>
#include <sensor_msgs/JointState.h>//for publishing robot current position
#include <trajectory_msgs/JointTrajectory.h>

#include "faraday_simulator/faraday_simulator.h"

void publishCurrentState(const ros::TimerEvent& timer,
						 ros::Publisher& pub,
						 faraday_simulator::FaradaySimulator& sim)
{
	sensor_msgs::JointState joint_state;
	joint_state.header.frame_id="base_link";
	joint_state.header.stamp=ros::Time::now();
	joint_state.name=sim.getJointNames();
	//compute current position
	sim.computeTrajectoryPosition(timer.current_real,joint_state.position);
	sim.pollAction();

	// std::reverse(joint_state.position.begin(),joint_state.position.end());
	// for(unsigned i=0;i<joint_state.position.size();++i)
	// 	joint_state.position[i]*=-1.0;

	pub.publish(joint_state);
}						

void setCurrentTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& traj,
						  faraday_simulator::FaradaySimulator& sim)
{
	ROS_INFO("Setting new trajctory");
	sim.setTrajectory(*traj);
}						  

int main(int argc,char** argv)
{
	const static double default_position[]={227.139, 225.652, 225.652, 227.139, 0.0};

	ros::init(argc,argv,"faraday_simulator_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~"); //using this style handlenode is able to get private name

	//nh loads joint names if possible
	std::vector<std::string> joint_names;
	if(!nh.getParam("controller_joint_names",joint_names))
	{
		//otherwise, it loads defaults
		joint_names.push_back("joint_1");
		joint_names.push_back("joint_2");
		joint_names.push_back("joint_3");
		joint_names.push_back("joint_4");
		joint_names.push_back("joint_5");
	}

	//pnh loads configuration parameters
	std::vector<double> seed_position;
	if(!pnh.getParam("initial_position",seed_position))
		seed_position.assign(default_position,default_position+5);

	// ROS_INFO_STREAM("initial_position: "<<seed_position[0]);

	double publish_rate;
	pnh.param<double>("rate",publish_rate,30.0);

	// ROS_INFO_STREAM("publish_rate: "<<publish_rate);

	//instantiate simulation
	faraday_simulator::FaradaySimulator sim(seed_position,joint_names,nh);

	//create pub/subscribers and wire them up 
	ros::Publisher current_state_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);
	ros::Subscriber command_state_sub=
		nh.subscribe<trajectory_msgs::JointTrajectory>("joint_path_command",
													  1,
													  boost::bind(setCurrentTrajectory,
													  			   _1,
													  			   boost::ref(sim)));
	ros::Timer state_publish_timer=
		nh.createTimer(ros::Duration(1.0/publish_rate),boost::bind(publishCurrentState,
													    _1,
													    boost::ref(current_state_pub),
													    boost::ref(sim)));

	ROS_INFO("Simulator service spinning");

	ros::spin();

	return 0;
}
