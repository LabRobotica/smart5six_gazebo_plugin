
#include "RobotMovementComauLib.h"

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

//#define USE_EORL

#define STRING_IP_CNTRL        "172.22.178.102" //robot ip (for the download of the c5g file from controller)
#define STRING_SYS_ID          "CNTRLC5G_102"


#define CYCLE_TIME_IN_SECONDS                   0.002
#define NUMBER_OF_DOFS                          6
ReflexxesAPI *RML = NULL;
RMLPositionInputParameters *IP = NULL;
RMLPositionOutputParameters *OP = NULL;
RMLPositionFlags Flags;
int ResultValue = 0;

RobotMovementComauLib::RobotMovementComauLib(const unsigned int jointsNumber) {

    std::string path = ros::package::getPath("smart5six_gazebo_plugin") + "/";
    if ((ORLOPEN_initialize_controller(STRING_IP_CNTRL, STRING_SYS_ID, ORL_SILENT, ORL_CNTRL01)) != 0) {
        if (ORL_initialize_controller("CNTRLC5G_61.c5g", path.c_str(), ORL_SILENT, ORL_CNTRL01) != 0) {
            printf("ERROR! ORL controller cannot be initilized!\n");
            ros::shutdown();
        }
    } else
        printf("%s.c5g OK\n", STRING_SYS_ID);

    // read modality from .launch file
    ros::param::get(("modality"), modalityString);


    //initialize variables
    this->jointsNumber = jointsNumber;
    targetJointsAngles.resize(jointsNumber);

    si_k = 0;
    print = false;

    car_pos.unit_type = ORL_CART_POSITION;
    car_pos.x = 0;
    car_pos.y = 0;
    car_pos.z = 0;
    car_pos.a = 0;
    car_pos.e = 0;
    car_pos.r = 0;
    strcpy(car_pos.config_flags, " ");

    actualJointsPosition.unit_type = ORL_POSITION_LINK_DEGREE;
    targetJointsPosition.unit_type = ORL_POSITION_LINK_DEGREE;
    for (unsigned int i = 0; i < jointsNumber; i++) {
        actualJointsPosition.value[i] = 0;
        targetJointsPosition.value[i] = 0;
    }

#ifdef USE_EORL
    ORL_set_position(NULL, &targetJointsPosition, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM2);
    ORLOPEN_set_period(ORL_2_0_MILLIS, ORL_VERBOSE, ORL_CNTRL01);
#else    
    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    IP->MaxVelocityVector->VecData[0] = 2.443; // rad/s
    IP->MaxVelocityVector->VecData[1] = 2.792; // rad/s
    IP->MaxVelocityVector->VecData[2] = 2.967; // rad/s
    IP->MaxVelocityVector->VecData[3] = 7.854; // rad/s
    IP->MaxVelocityVector->VecData[4] = 6.545; // rad/s
    IP->MaxVelocityVector->VecData[5] = 9.6; // rad/s


    for (unsigned int i = 0; i < NUMBER_OF_DOFS; i++) {

        IP->MaxAccelerationVector->VecData[i] = IP->MaxVelocityVector->VecData[i] * 2; // from 0 to max speed in 500ms
        IP->SelectionVector->VecData[i] = true;
    }
#endif

    status = 2;
    modified = false;

    if (modalityString.compare("inverseKinematics") == 0) {
        sub = n.subscribe("endEffectorPos", 1, &RobotMovementComauLib::updateEndEffectorPosition, this);
    }
    printf("RobotMovement modality: \"%s\"\n", modalityString.c_str());
}

RobotMovementComauLib::~RobotMovementComauLib() {

    ORL_terminate_controller(ORL_VERBOSE, ORL_CNTRL01);

    delete RML;
    delete IP;
    delete OP;
}

void RobotMovementComauLib::updateEndEffectorPosition(const smart5six_gazebo_plugin::endEffectorPosition::ConstPtr &pos) {

    car_pos.x = pos->x;
    car_pos.y = pos->y;
    car_pos.z = pos->z;
    car_pos.a = pos->a;
    car_pos.e = pos->e;
    car_pos.r = pos->r;

    //printf("car_pos: x %f, y %f, z %f, a %f, e %f, r %f\n", car_pos.x, car_pos.y, car_pos.z, car_pos.a, car_pos.e, car_pos.r);
    ORL_inverse_kinematics(&car_pos, &targetJointsPosition, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);

    for (unsigned int i = 0; i < NUMBER_OF_DOFS; i++) {

        targetJointsAngles[i] = targetJointsPosition.value[i] * DEG2RAD;
        IP->TargetPositionVector->VecData [i] = targetJointsAngles[i]; // first waypoint
    }
    ResultValue = 0;

    modified = false;
}

void RobotMovementComauLib::getNextInterpolationAngles(std::vector<double>& jointsAngles) {

#ifdef USE_EORL
    if (status == ORLOPEN_RES_OK) {
        boost::mutex::scoped_lock lock(mutex);
        status = ORL_get_next_interpolation_step(&actualJointsPosition, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);
        si_k++;
    }
#else
    if (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED) {

        boost::mutex::scoped_lock lock(mutex);
        // Calling the Reflexxes OTG algorithm
        ResultValue = RML->RMLPosition(*IP, OP, Flags);

        *IP->CurrentPositionVector = *OP->NewPositionVector;
        *IP->CurrentVelocityVector = *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
        si_k++;
    }
#endif

    for (unsigned int i = 0; i < jointsNumber; i++) {
#ifdef USE_EORL
        jointsAngles[i] = actualJointsPosition.value[i] * DEG2RAD;
#else
        jointsAngles[i] = IP->CurrentPositionVector->VecData[i];
#endif
    }
}

void RobotMovementComauLib::setMove(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj)
{
	ROS_INFO("set move traj");

    if (modalityString.compare("inverseKinematics") != 0) { //if the modality is inverse kinematics, skip jointState angle setting

        for (unsigned int i = 0; i < jointTraj->points[0].positions.size() ; i++)
        {
            if (fabs(targetJointsAngles[i] - jointTraj->points[0].positions[i]) > 3 * DEG2RAD)
            { //EPSILON

                boost::mutex::scoped_lock lock(mutex);
#ifdef USE_EORL
                status = ORL_cancel_motion(ORL_SILENT, ORL_CNTRL01, ORL_ARM2); //status = 2;
#endif
                modified = true;
                print = true;
                break;
            }
        }
        if (!modified) //exit from the method if joints sliders have not been modified
            return;

        si_k = 0;
#ifdef USE_EORL
        ORL_set_position(NULL, &actualJointsPosition, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);
        ORL_set_move_parameters(ORL_NO_FLY, ORL_WAIT, ORL_FLY_NORMAL, ORL_TRJNT, NULL, &targetJointsPosition, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);
#endif

        for (unsigned int i = 0; i < jointTraj->points[0].positions.size(); i++)
        {
            targetJointsAngles[i] = jointTraj->points[0].positions[i];
            targetJointsPosition.value[i] = jointTraj->points[0].positions[i] * RAD2DEG;
            ROS_INFO("received joint_%d trajectory value: %f", i + 1, jointTraj->points[0].positions[i]);
        }

        boost::mutex::scoped_lock lock(mutex);

#ifdef USE_EORL
        status = ORLOPEN_RES_OK;
#else
        for (int i = 0; i < NUMBER_OF_DOFS; ++i) {
            IP->TargetPositionVector->VecData [i] = targetJointsAngles[i];
        }
        ResultValue = 0;
#endif

        modified = false;
    }
}

void RobotMovementComauLib::setMove(const sensor_msgs::JointState::ConstPtr& jointState) {

    if (modalityString.compare("inverseKinematics") != 0) { //if the modality is inverse kinematics, skip jointState angle setting

        for (unsigned int i = 0; i < jointState->position.size(); i++) {
            if (fabs(targetJointsAngles[i] - jointState->position[i]) > 3 * DEG2RAD) { //EPSILON

                boost::mutex::scoped_lock lock(mutex);
#ifdef USE_EORL
                status = ORL_cancel_motion(ORL_SILENT, ORL_CNTRL01, ORL_ARM2); //status = 2;
#endif
                modified = true;
                print = true;
                break;
            }
        }
        if (!modified) //exit from the method if joints sliders have not been modified
            return;

        si_k = 0;
#ifdef USE_EORL
        ORL_set_position(NULL, &actualJointsPosition, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);
        ORL_set_move_parameters(ORL_NO_FLY, ORL_WAIT, ORL_FLY_NORMAL, ORL_TRJNT, NULL, &targetJointsPosition, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);
#endif

        for (unsigned int i = 0; i < jointState->position.size(); i++) {
            targetJointsAngles[i] = jointState->position[i];
            targetJointsPosition.value[i] = jointState->position[i] * RAD2DEG;
        }

        boost::mutex::scoped_lock lock(mutex);

#ifdef USE_EORL
        status = ORLOPEN_RES_OK;
#else
        for (int i = 0; i < NUMBER_OF_DOFS; ++i) {
            IP->TargetPositionVector->VecData [i] = targetJointsAngles[i]; 
        }
        ResultValue = 0;
#endif

        modified = false;
    }
}
