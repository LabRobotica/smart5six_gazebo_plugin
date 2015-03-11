
#ifndef ROBOTMOVEMENTINTERFACE_H
#define	ROBOTMOVEMENTINTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>
#include <vector>

#define DEG2RAD 0.017453293
#define RAD2DEG 57.295779513
#define EPSILON 0.005

class RobotMovementInterface {
    
public:
    virtual ~RobotMovementInterface(){};
    
    /**
     * This method must calculate the new joints angles values.
     * @param actualJointsAngles This joints array contains the actual joint model values. The method updates this array with new calculated values.
     */
    virtual void getNextInterpolationAngles(std::vector<double>& actualJointsAngles) = 0;
    
    /**
     * This method must calculate the new joints velocity values.
     * @param actualJointsAngles The current joints angles array.
     * @param actualJointsVelocities The new joints velocity computed. This method saves the results in this variable.
     */
    virtual void getNextInterpolationVelocities(std::vector<double>& actualJointsAngles, std::vector<double>& actualJointsVelocities) = 0;
    
    /**
     * This method must set up the new requested joints values.
     * @param jointState The message that contains the new joints values.
     */
    virtual void setMove(const sensor_msgs::JointState::ConstPtr& jointState) = 0;

   // virtual void setMove(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj) = 0;
};

#endif	/* ROBOTMOVEMENTINTERFACE_H */

