#include "faraday_simulator/faraday_simulator.h"
#include <ros/console.h>
#include <ros/assert.h>

faraday_simulator::FaradaySimulator::FaradaySimulator(const std::vector<double>& seed_pose,
													  const std::vector<std::string>& joint_names,
													  ros::NodeHandle& nh)
	:joint_names_(joint_names)
	,traj_start_position_(seed_pose)						
	,traj_start_time_(ros::Time::now())
	,action_server_(nh,"joint_trajectory_action",boost::bind(&FaradaySimulator::goalCB,this,_1),
					boost::bind(&FaradaySimulator::cancelCB,this,_1),false)
	,has_active_goal_(false)
{
	ROS_ASSERT(seed_pose.size()==5);
	ROS_ASSERT(joint_names.size()==5);
	action_server_.start();
}


bool faraday_simulator::FaradaySimulator::setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory)
{
	ROS_INFO("Setting new active trajectory");
	//Compute current state
	ros::Time now=ros::Time::now();
	std::vector<double> position;
	computeTrajectoryPosition(now,position);

	//Rollover to the new trajectory
	traj_start_position_=position;
	traj_=new_trajectory;
	traj_start_time_=now;

	return true;
}


static double linearInterpolate(double start,double stop,double ratio)
{
	return start+(stop-start)*ratio;
}

//Compute the robot position at a given time based on the currently active
//trajectory
bool faraday_simulator::FaradaySimulator::computeTrajectoryPosition(const ros::Time& tm,
																   std::vector<double>& output) const
{
	//Check to see if time is in past of traj
	if(tm<traj_start_time_||traj_.points.empty())
	{
		output=traj_start_position_;
		return true;
	}
	//check to see if time is past end of traj
	else if(tm>traj_start_time_+traj_.points.back().time_from_start)
	{
		output=traj_.points.back().positions;
		return true;
	}

	//Otherwise the traj must be within the trajectory
	ros::Duration dt=tm-traj_start_time_;

	size_t idx=0;
	for(size_t i=0;i<traj_.points.size();++i)
	{
		if(dt<traj_.points[i].time_from_start)
		{
			idx=i;
			break;
		}
	}

	//Grab the two points and interpolate
	const trajectory_msgs::JointTrajectoryPoint& end_pt=traj_.points[idx];

	//output container
	std::vector<double> point;
	point.reserve(traj_start_position_.size());

	if(0==idx)
	{
		//interpolate from start position
		double ratio=dt.toSec()/end_pt.time_from_start.toSec();

		for (int i = 0; i < 5; ++i)
		{
			point.push_back(linearInterpolate(traj_start_position_[i],end_pt.positions[i],ratio));
		}
	}
	else
	{
		const trajectory_msgs::JointTrajectoryPoint& start_pt=traj_.points[idx-1];
		//interpolate between two points
		double ratio=(dt-start_pt.time_from_start).toSec()/(end_pt.time_from_start-start_pt.time_from_start).toSec();

		for (int i = 0; i < 5; ++i)
		{
			point.push_back(linearInterpolate(start_pt.positions[i],end_pt.positions[i],ratio));
		}
	}

	output=point;

	return true;
}				

void faraday_simulator::FaradaySimulator::pollAction()												
{
	if(has_active_goal_&&ros::Time::now()>(traj_start_time_+traj_.points.back().time_from_start))
	{
		active_goal_.setSucceeded();
		has_active_goal_=false;
	}
}

void faraday_simulator::FaradaySimulator::goalCB(JointTrajectoryActionServer::GoalHandle& gh)
{
	ROS_INFO("Receive new goal request");
	if(has_active_goal_)
	{
		ROS_WARN("Receive new goal,canceling current one");
		active_goal_.setAborted();
		has_active_goal_=false;
	}

	gh.setAccepted();
	active_goal_=gh;
	has_active_goal_=true;

	const trajectory_msgs::JointTrajectory& traj=active_goal_.getGoal()->trajectory;
	setTrajectory(traj);
}


void faraday_simulator::FaradaySimulator::cancelCB(JointTrajectoryActionServer::GoalHandle &gh)
{
	ROS_INFO("cancelling goal");
	if(active_goal_==gh)
	{
		//stop the controller
		//mark the goal as canceled
		active_goal_.setCanceled();
		has_active_goal_=false;
	}
}