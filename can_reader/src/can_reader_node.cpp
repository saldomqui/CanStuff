/************************************************************************
 * by Salvador Dominguez
 *
 ************************************************************************/

/**
  \file can_reader.cpp
  \brief ROS node to get and publish CAN msgs
  \author Salvador Dominguez
  \date 7/11/2023
  */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>

// Semaphores and threads
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

// ROS
#include "ros/ros.h"

#include "thorvald_base/CANFrame.h"

// Namespaces
using namespace std;

int soc;
int read_can_port;
string can_port;

bool getCanData(thorvald_base::CANFrame &msg_can);

void canToSendCallback(thorvald_base::CANFrame msg_can);

int open_port(const char *port);
int send_port(struct can_frame *frame);

bool extendedCanId = false;

int open_port(const char *port)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (soc < 0)
    {
        return (-1);
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        return (-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        return (-1);
    }

    return 0;
}

int send_port(struct can_frame *frame)
{
    int retval;

    retval = write(soc, frame, sizeof(struct can_frame));
    // cout << "can_reader: retval:" << retval << endl;
    if (retval != sizeof(struct can_frame))
    {
        return (-1);
    }
    else
    {
        return (0);
    }
}

/* this is just an example, run in a thread */

struct can_frame read_port()
{
    struct can_frame frame_rd;
    int recvbytes = 0;
    struct timeval timeout = {1, 0};
    fd_set readSet;

    FD_ZERO(&readSet);
    FD_SET(soc, &readSet);

    // timeout.tv_sec=0;
    // timeout.tv_usec=200000;

    frame_rd.can_dlc = 0;

    if (select((soc + 1), &readSet, NULL, NULL, &timeout) > 0)
    {
        if (FD_ISSET(soc, &readSet))
        {
            recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
            // cout << "can_reader: read_port(). recvbytes:" << recvbytes << endl;
            // printf("0x%03X [%d] ", frame_rd.can_id, frame_rd.can_dlc);
            // for (int i = 0; i < frame_rd.can_dlc; i++)
            //    printf("%02X ", frame_rd.data[i]);
            // printf("\r\n");
        }
    }
    return frame_rd;
}

int close_port()
{
    close(soc);
    return 0;
}

// Callback to catch the status of other devices
void canToSendCallback(const thorvald_base::CANFrameConstPtr &msg_can)
{
    struct can_frame frame_wr;

    frame_wr.can_id = msg_can->id;
    frame_wr.can_dlc = msg_can->length;
    for (int i = 0; i < msg_can->length; i++)
    {
        frame_wr.data[i] = msg_can->data[i];
    }
    printf("can_reader: SENDING CAN MSG: ID: %2X, DLC: %d, DAT: %2X %2X %2X %2X %2X %2X %2X %2X\n", frame_wr.can_id, frame_wr.can_dlc, frame_wr.data[0], frame_wr.data[1], frame_wr.data[2], frame_wr.data[3], frame_wr.data[4], frame_wr.data[5], frame_wr.data[6], frame_wr.data[7]);

    while (send_port(&frame_wr) < 0)
        ;
    //    cout << "can_reader: Error sending can data" << endl;
}

bool getCanData(thorvald_base::CANFrame &msg_can)
{
    struct can_frame frame_rd;

    frame_rd = read_port();
    if (frame_rd.can_dlc)
    {
        if (frame_rd.can_dlc != msg_can.length)
        {
            msg_can.data.resize(frame_rd.can_dlc);
        }

        msg_can.id = frame_rd.can_id;
        msg_can.length = frame_rd.can_dlc;
        for (int i = 0; i < frame_rd.can_dlc; i++)
            msg_can.data[i] = frame_rd.data[i];

        return true;
    }

    return false;
}

int main(int argc, char **argv)
{
    // Connect to ROS
    ros::init(argc, argv, "ros_can_monitor");
    ROS_INFO("Node ros_can_monitor Connected to roscore");

    ros::NodeHandle local_nh_("~"); // Local ROS Handler
    ros::NodeHandle global_nh_;     // Local ROS Handler

    local_nh_.getParam("can_port", can_port);
    cout << "can_reader: can_port:" << can_port << endl;

    ros::Publisher can_pub;

    // Publishers
    can_pub = global_nh_.advertise<thorvald_base::CANFrame>("can/data_read", 1000);

    // Subscribers
    ROS_INFO("Subscribing to topics\n");
    ros::Subscriber can_sub;

    can_sub = global_nh_.subscribe<thorvald_base::CANFrame>("can/data_to_send", 33, &canToSendCallback);

    if (open_port(can_port.c_str()) < 0)
    {
        cout << "can_reader: Unable to open: " << can_port << " port!!!" << endl;
        exit(-1);
    }

    while (global_nh_.ok())
    {
        bool got_can_msg = false;

        thorvald_base::CANFrame msg_can;

        got_can_msg = getCanData(msg_can);
        if (got_can_msg)
            can_pub.publish(msg_can);

        ros::spinOnce();
    }

    close_port();
    ROS_INFO("ROS-Node Terminated\n");
}
