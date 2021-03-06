/************************************************************************************
 * BSD 3-Clause License
 * Copyright (c) 2021, Ameya konkar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    talker.cpp
 *  @author  ameyakonk (AmeyaKonkar)
 *  @date    11/08/2021
 *  @version 1.0
 *
 *  @brief Source file to implement a simple ROS publisher node and a service
 *         server node
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement a simple ROS pubslisher node publishing a custom
 *  message and facilitate change in message content upon a request
 *
 */

#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <stdlib.h>
#include <sstream>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "beginner_tutorials/message_srv.h"

std::string outputMessage =         // default output message
    "Hello Terps";

/**
 *   @brief  the ros service callback function that modifies the string to
 * publish
 *
 *   @param  req is the data member of string type in Request object of
 *           changeMessage service
 *           resp is the data member of string type in Response object of
 *           changeMessage service
 *   @return boolean value. true to indicate succesful service, false to
 *           indicate failure
 */
bool changeMessage(beginner_tutorials::message_srv::Request &req,
                  beginner_tutorials::message_srv::Response &resp) {
  if (!req.a.empty()) {
    ROS_WARN_STREAM("New Message "
                    << req.a);
    outputMessage = req.a;
    resp.message = outputMessage;
    return true;
  } else {
    ROS_ERROR_STREAM(
        "Publishing default message!");
    return false;
  }
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::ServiceServer server =
    n.advertiseService("change_message", &changeMessage);

    int frequency;
    if (argc == 2) {
    frequency = std::atoi(argv[1]);
    if (frequency <= 0) {
    ROS_ERROR_STREAM("The user defined frequency is a non positive number");
    ROS_WARN_STREAM("Frequency is set to 10 Hz");
    frequency = 10;
    }
    } else if (argc == 1) {
        ROS_WARN_STREAM("No frequency specified. Frequency is set to 10 Hz");
        frequency = 10;
    } else {
        ROS_FATAL_STREAM(
        "Multiple frequencies specified by the user! Shutting down publisher"
        "node!");
        ros::shutdown();
    }
    ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << outputMessage << count;
    msg.data = ss.str();
    ROS_INFO_STREAM("Frequency: " << frequency);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
        chatter_pub.publish(msg);
    transform.setOrigin(tf::Vector3(cos(ros::Time::now().toSec()),
                                    sin(ros::Time::now().toSec()), 0.0));
    tf::Quaternion q;
    // Set yaw to 1 radian and 0 roll and pitch values
    q.setRPY(0, 0, 1.0);
    transform.setRotation(q);
    // Broadcast the transform at current time with world as parent frame and
    // talk as child frame
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
return 0;
}
