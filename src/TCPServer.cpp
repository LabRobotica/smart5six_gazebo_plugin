
#include "TCPServer.h"

std::string modalityString = "";

TCPServer::TCPServer() {

    packetNum = 0;
    listenfd = 0;
    connfd = 0;

    for (int i = 0; i < 6; i++)
        values[i] = 0.0;

    listenfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    printf("socket retrieve success\n");

    memset(&serv_addr, '0', sizeof (serv_addr));
    memset(sendBuff, '0', sizeof (sendBuff));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5555);

    bind(listenfd, (struct sockaddr*) &serv_addr, sizeof (serv_addr));

    if (listen(listenfd, 10) == -1) {
        printf("Failed to listen\n");
        ros::shutdown();
    }

    //get modality from launcher
    ros::param::get(("modality"), modalityString);
    if (modalityString.compare("jointsRemapping") == 0 || modalityString.compare("") == 0)
        modalityChar = 'j';
    else if (modalityString.compare("inverseKinematics") == 0)
        modalityChar = 'i';

    printf("TCP server modality: \"%s\"\n", modalityString.c_str());

    //start accepting connection
    socklen_t clientLen = sizeof(client_addr); 
    connfd = accept(listenfd, (struct sockaddr*) &client_addr, &clientLen);
}

TCPServer::~TCPServer() {

    close(connfd);
    close(listenfd);
}

void TCPServer::endEffectorPositionCB(const smart5six_gazebo_plugin::endEffectorPosition::ConstPtr &positionMsg) {

    values[0] = positionMsg->x;
    values[1] = positionMsg->y;
    values[2] = positionMsg->z;
    values[3] = positionMsg->a;
    values[4] = positionMsg->e;
    values[5] = positionMsg->r;

    /////////////////////////////////////////////////////////////
    //        send the packet containing the new values        //
    /////////////////////////////////////////////////////////////

    send(connfd, sendBuff, sizeof (sendBuff), 0);
    memcpy(sendBuff, &packetNum, sizeof (packetNum)); //copy the packet number on the buffer
    packetNum++;

    memcpy(sendBuff + 7 * sizeof (number), &modalityChar, sizeof (modalityChar)); //copy the modality char at the end of the packet
    //printf("buff: %c\n",sendBuff[7 * sizeof (number)]);

    for (unsigned int i = 0; i < 6; i++) {
        number = pack754_64(values[i]);
        memcpy(sendBuff + (i + 1) * sizeof (number), &number, sizeof (number)); //copy the 6 values on the buffer
    }
}

void TCPServer::jointsAnglesCB(const sensor_msgs::JointState::ConstPtr &jointState) {

    for (unsigned int i = 0; i < jointState->position.size(); i++) {
        values[i] = jointState->position[i];
    }

    /////////////////////////////////////////////////////////////
    //        send the packet containing the new values        //
    /////////////////////////////////////////////////////////////

    send(connfd, sendBuff, sizeof (sendBuff), 0);
    memcpy(sendBuff, &packetNum, sizeof (packetNum)); //copy the packet number on the buffer
    packetNum++;

    memcpy(sendBuff + 7 * sizeof (number), &modalityChar, sizeof (modalityChar)); //copy the modality char
    //printf("buff: %c\n",sendBuff[7 * sizeof (number)]);

    for (unsigned int i = 0; i < 6; i++) {
        number = pack754_64(values[i] * RAD2DEG);
        memcpy(sendBuff + (i + 1) * sizeof (number), &number, sizeof (number)); //copy the 6 values on the buffer
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "tcp_server");
    ros::NodeHandle n;

    TCPServer server;

    ros::Subscriber sub;
    if (modalityString.compare("jointsRemapping") == 0 || modalityString.compare("") == 0)
        sub = n.subscribe("joint_states", 1, &TCPServer::jointsAnglesCB, &server);
    else if (modalityString.compare("inverseKinematics") == 0)
        sub = n.subscribe("endEffectorPos", 1, &TCPServer::endEffectorPositionCB, &server);

    ros::spin();

    return 0;
}
