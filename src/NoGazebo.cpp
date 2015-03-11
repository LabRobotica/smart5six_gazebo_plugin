
#include "NoGazebo.h"

NoGazebo::NoGazebo(const ros::Duration d, const unsigned int jointsNumber) {

    this->prevUpdateTime = ros::Time::now();

    this->jointsNumber = jointsNumber;
    actualJointsAngles.resize(jointsNumber);
    targetJointsAngles.resize(jointsNumber);
    actualJointsVelocities.resize(jointsNumber);
    tempJointsAngles.resize(jointsNumber);

    //resize of the actual and desired msg with Comau's joints number
    actual.positions.resize(jointsNumber);
    desired.positions.resize(jointsNumber);
    for (int i = 1; i <= jointsNumber; i++) {
        std::stringstream name;
        name << "joint_" << i;
        feedback.joint_names.push_back(name.str());
    }
    startMovement = ros::Time::now();

#ifdef COMAU
    comau = new RobotMovementComauLib(jointsNumber);
#else
    pid = new RobotMovementPID(jointsNumber, d.toSec());
    pid->setPGain(2.0);
    pid->setDGain(0.0);
#endif

    feedback_states_pub = n.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
    cmd_angles_sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1, &NoGazebo::cmdCallback, this);

    feedback.desired = desired;
}

NoGazebo::~NoGazebo() {

#ifdef COMAU
    delete comau;
#else
    delete pid;
#endif
}

void NoGazebo::cmdCallback(const sensor_msgs::JointState::ConstPtr& jointState) {

    //this for cycle is only for printing the target joints values
    for (int i = 0; i < jointState->position.size(); i++) {
        if (fabs(targetJointsAngles[i] - jointState->position[i]) > EPSILON) {
            targetJointsAngles[i] = jointState->position[i];
        }
    }
#ifdef COMAU
    comau->setMove(jointState);
#else
    pid->setMove(jointState);
#endif

    for (unsigned int i = 0; i < jointState->position.size(); i++) {
        desired.positions[i] = jointState->position[i];
    }

    feedback.desired = desired;
    startMovement = ros::Time::now();
}

void NoGazebo::update(const ros::TimerEvent& event) {

#ifdef COMAU
    comau->getNextInterpolationAngles(actualJointsAngles);
#else
    pid->getNextInterpolationAngles(actualJointsAngles);
#endif

    //        for (unsigned int i = 0; i < jointsNumber; i++) {
    //            ROS_INFO("joint_%d: actual angle %.3f | desired angle %.3f", i + 1, actualJointsAngles[i], targetJointsAngles[i]);
    //        }
    //        ROS_INFO("");

    for (unsigned int i = 0; i < jointsNumber; i++) {
        actual.positions[i] = actualJointsAngles[i];
    }
    actual.time_from_start = ros::Time::now() - startMovement;
    feedback.actual = actual;
    feedback_states_pub.publish(feedback);

    prevUpdateTime = ros::Time::now();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "no_gazebo");

    int jointsNumber = 6;

    ros::Duration d(0.002);
    NoGazebo nogazebo = NoGazebo(d, jointsNumber);

    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(d, &NoGazebo::update, &nogazebo);

    ros::spin();

    return 0;
}