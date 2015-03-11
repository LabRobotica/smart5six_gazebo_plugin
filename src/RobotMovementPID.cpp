
#include "RobotMovementPID.h"

RobotMovementPID::RobotMovementPID(const unsigned int jointsNumber, const double interpolationPeriod) {

    targetJointsAngles.resize(jointsNumber);
    
    this->interpolationPeriod = interpolationPeriod;
    this->jointsNumber = jointsNumber;
    jointsModified = false;
    dgain = 0.0;
    pgain = 1.0;
}

RobotMovementPID::~RobotMovementPID() {
}

void RobotMovementPID::setDGain(const double dgain) {

    this->dgain = dgain;
}

void RobotMovementPID::setPGain(const double pgain) {

    this->pgain = pgain;
}

void RobotMovementPID::getNextInterpolationAngles(std::vector<double>& actualJointsAngles) {

    for (unsigned int i = 0; i < jointsNumber; i++) {
        
        diff_force = pgain * (targetJointsAngles[i] - actualJointsAngles[i]);
        
        actualJointsAngles[i] += diff_force * interpolationPeriod;
    }
}

void RobotMovementPID::getNextInterpolationVelocities(std::vector<double>& actualJointsAngles, std::vector<double>& actualJointsVelocities)
{
    for (unsigned int i = 0; i < jointsNumber; i++)
    {
        damping_force = dgain * (0 - actualJointsVelocities[i]);
        diff_force = pgain * (targetJointsAngles[i] - actualJointsAngles[i]);

        //aggiunto
        actualJointsAngles[i] += diff_force * interpolationPeriod;
        actualJointsVelocities[i] = diff_force + damping_force;
    }
}

void RobotMovementPID::setMove(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj) {

	for (unsigned int i = 0; i < jointTraj->points[0].positions.size() ; i++) {
        if (fabs(targetJointsAngles[i] - jointTraj->points[0].positions[i]) > EPSILON) {
            jointsModified = true;
            break;
        }
    }
    if (!jointsModified)
        return;

    for (unsigned int i = 0; i < jointTraj->points[0].positions.size(); i++) {
    	targetJointsAngles[i] = jointTraj->points[0].positions[i];
    }
    jointsModified = false;
}

void RobotMovementPID::setMove(const sensor_msgs::JointState::ConstPtr& jointState) {

    for (unsigned int i = 0; i < jointState->position.size(); i++) {
        if (fabs(targetJointsAngles[i] - jointState->position[i]) > EPSILON) {
            jointsModified = true;
            break;
        }
    }
    if (!jointsModified)
        return;

    for (unsigned int i = 0; i < jointState->position.size(); i++) {
        targetJointsAngles[i] = jointState->position[i];
    }
    jointsModified = false;
}
