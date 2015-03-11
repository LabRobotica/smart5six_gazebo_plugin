
#ifndef NOGAZEBO_H
#define	NOGAZEBO_H

#include <ros/ros.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#ifdef COMAU
#include "RobotMovementComauLib.h"
#else
#include "RobotMovementPID.h"
#endif


/**
 * This class simulates the Comau's robot movements without visualizing them on Gazebo
 */
class NoGazebo {
public:
    /**
     * NoGazebo class constructor
     * @param jointsNumber Is the number of the robot joints
     */
    NoGazebo(const ros::Duration d, const unsigned int jointsNumber);

    /**
     * NoGazebo class destructor
     */
    ~NoGazebo();

    /**
     * This callback sets new target angles for the robot joints
     * @param jointState The jointState message containing the new target angles for the robot
     */
    void cmdCallback(const sensor_msgs::JointState::ConstPtr& jointState);

    /**
     * This method consists in a loop that keeps the robot position updated and it prints actual and target joints angle values
     */
    void update(const ros::TimerEvent& event);

    ros::Publisher feedback_states_pub, joint_states_pub, robot_status_pub;
    ros::Subscriber cmd_angles_sub;

private:
    ros::Time startMovement, prevUpdateTime;
    std::vector<double> actualJointsAngles, targetJointsAngles, actualJointsVelocities, tempJointsAngles;
    unsigned int jointsNumber;

    control_msgs::FollowJointTrajectoryFeedback feedback;
    trajectory_msgs::JointTrajectoryPoint desired, actual;

#ifdef COMAU
    RobotMovementComauLib* comau;
#else   
    RobotMovementPID* pid;
#endif
    ros::NodeHandle n;
};

#endif	/* NOGAZEBO_H */

