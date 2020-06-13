#ifndef FARADAY_SIMULATOR_H
#define FARADAY_SIMULATOR_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/action_server.h>

namespace faraday_simulator
{
class FaradaySimulator
{
public:
	FaradaySimulator(const std::vector<double>& seed_pose,
					 const std::vector<std::string>& joint_names,
					 ros::NodeHandle& nh);

	const std::vector<std::string>& getJointNames() const {return joint_names_;}

	//Initializes trajectory,start time, and position fields
	bool setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory);
	//Compute the robot position at a given time based on the currently active
	//trajectory
	bool computeTrajectoryPosition(const ros::Time& tm,std::vector<double>& output) const;

	void pollAction();
	~FaradaySimulator() {}

private:
	//Configuration
	std::vector<std::string> joint_names_;

	//Action server
	typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTrajectoryActionServer;

	void goalCB(JointTrajectoryActionServer::GoalHandle& gh);
	void cancelCB(JointTrajectoryActionServer::GoalHandle& gh);

	JointTrajectoryActionServer action_server_;
	JointTrajectoryActionServer::GoalHandle active_goal_;
	bool has_active_goal_;

	//State
	trajectory_msgs::JointTrajectory traj_;
	std::vector<double> traj_start_position_;
	ros::Time traj_start_time_;
};

}


#endif //FARADAY_SIMULATOR_H