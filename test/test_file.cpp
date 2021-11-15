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
 *  @file    test_file.cpp
 *  @author  ameyakonk (AmeyaKonkar)
 *  @date    11/15/2021
 *  @version 1.0
 *
 *  @brief Source file to implement a simple ROS test
 *
 *  @section DESCRIPTION
 *
 *  Source file to implement a simple ROS test.  
 * 
 */

#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>
#include <ros/service_client.h>

// modifyOutput service
#include "beginner_tutorials/message_srv.h"

// ROS Standard message
#include "std_msgs/String.h"

/**
 * @brief Test case to check the existence of the modifyOutput service
 * @param none
 * @return none
 */
TEST(TestTalkerNode, testInitializationOfROSService) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::message_srv>("change_message");
  // Check if the client exists
  bool exists(client.waitForExistence(ros::Duration(5.0)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Test case to check the succesful call of the modifyOutput service
 * @param none
 * @return none
 */
TEST(TestTalkerNode, testROSServiceCall) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::message_srv>("change_message");

  beginner_tutorials::message_srv::Request req;
  beginner_tutorials::message_srv::Response resp;

  req.a = "Hello Terps";
  std::string expectedString = req.a;

  bool success = client.call(req, resp);
  EXPECT_TRUE(success);
  EXPECT_EQ(expectedString, resp.message);
}
