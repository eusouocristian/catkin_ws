#include "ros/ros.h"
#include "SubscribeAndPublish.hpp"
#include <string>
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
ros::ServiceServer service;
ros::ServiceClient client;


void SubscribeAndPublish::SAPMethod() {
    ros::NodeHandle n; 

    //Topic you want to publish
    this->pub1 = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    this->pub2 = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    //Topic you want to subscribe
    this->sub1 = n.subscribe("/simple_arm/joint_states", 10, &SubscribeAndPublish::joint_states_callback, this);
    this->sub2 = n.subscribe("rgb_camera/image_raw", 10, &SubscribeAndPublish::look_away_callback, this);
    
  }


void SubscribeAndPublish::joint_states_callback(const sensor_msgs::JointState js)
{
    // Get joints current position
    std::vector<double> joints_current_position = js.position;

    // Define a tolerance threshold to compare double values
    double tolerance = 0.0005;

    // Check if the arm is moving by comparing its current joints position to its latest
    if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position[1]) < tolerance)
        this->moving_state = false;
    else {
        this->moving_state = true;
        this->joints_last_position = joints_current_position;
    }
}

void SubscribeAndPublish::look_away_callback(const sensor_msgs::Image img)
{
    bool uniform_image = true;

    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] - img.data[0] != 0) {
            uniform_image = false;
            break;
        }
    }

    // If the image is uniform and the arm is not moving, move the arm to the center
    if (uniform_image == true && moving_state == false)
        SubscribeAndPublish::move_arm_center();
}

void SubscribeAndPublish::move_arm_center()
{
    ros::NodeHandle n; 
    // Define a client service capable of requesting services from safe_move

    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles [1.57, 1.57]
    simple_arm::GoToPosition srv;

    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;
        
    client.call(srv);
    ROS_INFO_STREAM("SERVICE CALLED");

    // //Call the safe_move service and pass the requested joint angles
    // if (!client.call(srv)) 
    //     ROS_ERROR("Failed to call service safe_move");
}



// This function checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
    // Define clamped joint angles and assign them to the requested ones
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    // Get min and max joint parameters, and assigning them to their respective variables
    float min_j1, max_j1, min_j2, max_j2;
    // Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;
    // Get node name
    std::string node_name = ros::this_node::getName();
    // Get joints min and max parameters
    n2.getParam(node_name + "/min_joint_1_angle", min_j1);
    n2.getParam(node_name + "/max_joint_1_angle", max_j1);
    n2.getParam(node_name + "/min_joint_2_angle", min_j2);
    n2.getParam(node_name + "/max_joint_2_angle", max_j2);

    // Check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1) {
        clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }
    // Check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2) {
        clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    // Store clamped joint angles in a clamped_data vector
    std::vector<float> clamped_data = { clamped_j1, clamped_j2 };

    return clamped_data;
}



bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
                            simple_arm::GoToPosition::Response& res)
{

    ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);
    
    SubscribeAndPublish SAPObj;
    
    // Check if requested joint angles are in the safe zone, otherwise clamp them
    std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

    // Create Float64 variables to publish
    std_msgs::Float64 joint1_angle, joint2_angle;
    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    // Publish clamped joint angles to the arm
    SAPObj.pub1.publish(joint1_angle);
    SAPObj.pub2.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}



int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "unique_arm_mover");
    ros::NodeHandle n;

    // Define a safe_move service with a handle_safe_move_request callback function
    service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands");

    SubscribeAndPublish SAPObj;
    SAPObj.SAPMethod();

    // Handle ROS communication events
    ros::spin();

    return 0;
}