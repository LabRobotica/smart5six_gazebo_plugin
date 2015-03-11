
#ifndef ROBOTMOVEMENTPID_H
#define	ROBOTMOVEMENTPID_H

#include "RobotMovementInterface.h"
#include <vector>

class RobotMovementPID : public RobotMovementInterface {
public:
    /**
     * RobotMovementPID class constructor. 
     * By default, proportional and derivative gain of the PID controller are set to 1 and 0 respectively.
     * @param jointsNumber The number of the robot joints
     * @param interpolationPeriod The period of time that elapses for each interpolation step
     */
    RobotMovementPID(const unsigned int jointsNumber, const double interpolationPeriod);

    /**
     * RobotMovementPID class destructor.
     */
    ~RobotMovementPID();

    /**
     * This method updates the speed of the robot joints by taking the actual joints angles and velocities. 
     * The new velocities are saved into the actualJointsVelocities array.
     * @param actualJointsAngles The array of the robot current angles. The dimension of this array must be equal to jointsNumber.
     * @param actualJointsVelocities The array of the robot current velocities. The dimension of this array must be equal to jointsNumber.
     */
    void getNextInterpolationVelocities(std::vector<double>& actualJointsAngles, std::vector<double>& actualJointsVelocities);

    /**
     * This method sets new robot joints angles. 
     * The new target angles are set only if they are different from the previous target angles.
     * @param jointState The jointState message containing the target angles.
     */
    void setMove(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj);
    void setMove(const sensor_msgs::JointState::ConstPtr& jointState);

    /**
     * Changes the proportional gain of the PID controller of the robot.
     * @param pgain The new value of the proportional gain.
     */
    void setPGain(const double pgain);

    /**
     * Changes the derivative gain of the PID controller of the robot.
     * @param dgain The new value of the derivative gain.
     */
    void setDGain(const double dgain);
    
    /**
     * This method updates the joints angles by considering the interpolation time.
     * The new velocities are saved into the actualJointsVelocities array.
     * @param actualJointsAngles The array of the robot current angles. The dimension of this array must be equal to jointsNumber.
     */
    void getNextInterpolationAngles(std::vector<double>& actualJointsAngles);


private:
    bool jointsModified;
    std::vector<double> targetJointsAngles;
    unsigned int jointsNumber;
    double interpolationPeriod, dgain, pgain;
    double damping_force, diff_force;
};

#endif	/* ROBOTMOVEMENTPID_H */

