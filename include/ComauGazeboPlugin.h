
#ifndef COMAUGAZEBOPLUGIN_H_
#define COMAUGAZEBOPLUGIN_H_

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <controller_manager/controller_manager.h>
#include <sdf/sdf_config.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <urdf/model.h>

#include <stdio.h>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <set>
#include <vector>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//action server
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#ifdef COMAU
#include "RobotMovementComauLib.h"
#else
#include "RobotMovementPID.h"
#endif


namespace gazebo {

/**
 * This plugin manages the Comau's robot movement inside the Gazebo simulator
 */
class ComauGazeboPlugin : public ModelPlugin {

	enum ComauJoints {
		joint_1 = 0, // 0
		joint_2, // 1
		joint_3, // 2
		joint_4, // 3
		joint_5, // 4
		joint_6, // 5

		firstComauJoint = joint_1,
		lastComauJoint = joint_6,
	};

public:

	ComauGazeboPlugin() : pgain_(100), dgain_(0)
,as_(nh_, "comau_controller/follow_joint_trajectory", boost::bind(&ComauGazeboPlugin::goalCB, this, _1), false),
	action_name_ ("comau_controller/follow_joint_trajectory")
{
		//as_.registerPreemptCallback(boost::bind(&preemptCB, this));
	as_.start();
	            // check PID values
	        };

	/*ComauGazeboPlugin() : pgain_(2), dgain_(0){

	};*/

	/**
	 * Gazebo's plugin Destructor
	 */
	virtual ~ComauGazeboPlugin();

	/**
	 * This method loads the Comau's robot model and the Gazebo's plugin
	 * @param _parent Pointer to the model
	 * @param _sdf Pointer to the sdf world file
	 */
	virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	/**
	 * This method updates the model position inside Gazebo
	 */
	virtual void OnUpdate();


private:
	ros::Publisher feedback_states_pub, joint_states_pub, robot_status_pub, goal_pub;
	control_msgs::FollowJointTrajectoryFeedback feedback;
	control_msgs::FollowJointTrajectoryGoal goal_r;
	int pointc;
	trajectory_msgs::JointTrajectoryPoint desired, actual;
	ros::Subscriber cmd_angles_sub_, trajectory_sub;
	ros::WallTime startMovement, prevUpdateTime;
	double pgain_, dgain_;
	ros::WallDuration updateRate;
	std::vector<double> jointsAngles, jointsVelocities;
	unsigned int jointsNumber;
#ifdef COMAU
RobotMovementComauLib* comau;
#else   
RobotMovementPID* pid;
#endif

//\brief Gazebo's plugin variables
physics::ModelPtr parent_model_; // Pointer to the model
urdf::Model urdf_model_; // urdf model
std::vector<gazebo::physics::JointPtr> joints_;

//action server
ros::NodeHandle nh_;
// NodeHandle instance must be created before this line. Otherwise strange error may occur.
actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
control_msgs::FollowJointTrajectoryResult result;
std::string action_name_;
int goal;
int progress;

void trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj);
/**
 * This callback sets new target angles for the robot joints
 * @param jointState The jointState message containing the new target angles for the robot
 */
void cmdCallback(const sensor_msgs::JointState::ConstPtr& jointState);

void nextInterpolationStep(const ros::TimerEvent& event);

// \brief Service Call Name
std::string setModelsJointsStatesServiceName;

// \brief pointer to ros node
ros::NodeHandle* rosnode_;

// \brief ros service
ros::ServiceServer setModelsJointsStatesService;

// \brief ros service callback
// \brief tmp vars for performance checking
double wall_start, sim_start;

// \brief set topic name of robot description parameter
std::string robotNamespace;

// Pointer to the update event connection
event::ConnectionPtr updateConnection;

void goalCB(const control_msgs::FollowJointTrajectoryGoal::ConstPtr& goal);
void preemptCB();
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ComauGazeboPlugin)

} /* namespace gazebo */

#endif /* COMAUGAZEBOPLUGIN_H_ */
