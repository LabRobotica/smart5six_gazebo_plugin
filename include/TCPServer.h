
#ifndef TCPSERVER_H
#define	TCPSERVER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "smart5six_gazebo_plugin/endEffectorPosition.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>
#include <stdint.h>


#define pack754_32(f) (pack754((f), 32, 8))
#define pack754_64(f) (pack754((f), 64, 11))

#define RAD2DEG 57.295779513


/**
 * This class receives ROS messages and forwards their values through a TCP packet
 */
class TCPServer {
public:
    /**
     * TCPServer class constructor
     */
    TCPServer();
    
    /**
     * TCPServer class destructor
     */
    ~TCPServer();

    /**
     * This callback method receives a jointState message and sends its values through a TCP packet
     * @param jointState is the JointState message containing the angle value of each joint (in radians)
     */
    void jointsAnglesCB(const sensor_msgs::JointState::ConstPtr &jointState);
    
    /**
     * This callback method receives a endEffectorPosition message and sends its values through a TCP packet
     * @param positionMsg is the endEffectorPosition custom message containing the x, y and z position and the three Euler angles
     */
    void endEffectorPositionCB(const smart5six_gazebo_plugin::endEffectorPosition::ConstPtr &positionMsg);

private:
    long double fnorm;
    int shift, listenfd, connfd;
    long long sign, exp, significand;
    unsigned significandbits;
    double values[6];
    uint64_t number, packetNum;
    char modalityChar;

    struct sockaddr_in serv_addr, client_addr;
    char sendBuff[sizeof (number) * 7 + sizeof (modalityChar)]; //1 long for packet number + 6 double for joints angles/end-effector position + 1 char for modality + 
    
    
    uint64_t pack754(long double f, unsigned bits, unsigned expbits) {

        significandbits = bits - expbits - 1; // -1 for sign bit

        if (f == 0.0) return 0; // get this special case out of the way

        // check sign and begin normalization
        if (f < 0) {
            sign = 1;
            fnorm = -f;
        } else {
            sign = 0;
            fnorm = f;
        }

        // get the normalized form of f and track the exponent
        shift = 0;
        while (fnorm >= 2.0) {
            fnorm /= 2.0;
            shift++;
        }
        while (fnorm < 1.0) {
            fnorm *= 2.0;
            shift--;
        }
        fnorm = fnorm - 1.0;

        // calculate the binary form (non-float) of the significand data
        significand = fnorm * ((1LL << significandbits) + 0.5f);

        // get the biased exponent
        exp = shift + ((1 << (expbits - 1)) - 1); // shift + bias

        // return the final answer
        return (sign << (bits - 1)) | (exp << (bits - expbits - 1)) | significand;
    }
};

#endif	/* TCPSERVER_H */

