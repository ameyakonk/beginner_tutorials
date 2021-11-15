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

  beginner_tutorials::modifyOutput::Request req;
  beginner_tutorials::modifyOutput::Response resp;

  req.desiredOutput = "Hello Terps";
  std::string expectedString = req.a;

  bool success = client.call(req, resp);
  EXPECT_TRUE(success);
  EXPECT_EQ(expectedString, resp.message);
}
