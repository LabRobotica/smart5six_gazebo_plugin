
#ifndef ROBOTMOVEMENTCOMAULIB_H
#define	ROBOTMOVEMENTCOMAULIB_H

#include <unistd.h>
#include <stdio.h>
#include <ros/package.h>
#include <vector>
#include <boost/thread/pthread/mutex.hpp>

#include "smart5six_gazebo_plugin/endEffectorPosition.h"
#include "RobotMovementInterface.h"
#include "eORL.h"

class RobotMovementComauLib : public RobotMovementInterface {
public:
    /**
     * The RobotMovementComauLib class constructor initializes the Comau's ORL library. 
     * @param jointsNumber The number of non fixed joints of the robot.
     */
    RobotMovementComauLib(const unsigned int jointsNumber);

    /**
     * The RobotMovementComauLib destructor. It terminates Comau's ORL library.
     */
    ~RobotMovementComauLib();

    /**
     * This method updates Comau's robot joints angles using ORL library.
     * @param jointsAngles The array of the robot joints angles. Once called this method, the jointsAngles array is updated with new interpolation values. JointsAngles array dimension must be equal to jointsNumber.
     */
    void getNextInterpolationAngles(std::vector<double>& actualJointsAngles);

    /**
     * This method sets new Comau's robot joints angles. The new target angles are set only if they are different from the previous target angles.
     * @param jointState The jointState message containing the target angles.
     */
    void setMove(const sensor_msgs::JointState::ConstPtr& jointState);
    void setMove(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj);

    void updateEndEffectorPosition(const smart5six_gazebo_plugin::endEffectorPosition::ConstPtr &pos);

private:

    void getNextInterpolationVelocities(const std::vector<double>& actualJointsAngles, std::vector<double>& actualJointsVelocities) {
    };

    ORL_joint_value targetJointsPosition, actualJointsPosition;
    ORL_cartesian_position car_pos;
    int status;
    std::vector<double> targetJointsAngles;
    unsigned int jointsNumber;
    std::string absolutePath;
    bool modified;
    std::string modalityString;

    int si_k;
    bool print;
    boost::mutex mutex;

    ros::NodeHandle n;
    ros::Subscriber sub;
};

#endif	/* ROBOTMOVEMENTCOMAULIB_H */

