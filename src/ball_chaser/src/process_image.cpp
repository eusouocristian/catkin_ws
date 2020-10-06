#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>
#include <optional>

// Define a global client that can request services
ros::ServiceClient client;
// Define global vectors for calculating the Integrative component for a PI controller
std::vector<float> linear_error_vector;
std::vector<float> angular_error_vector;


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service");
}



// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    std::vector<int> pixel_pose;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // img.width = 800;
    // img.step = 2400;
    // Loop through each pixel in the image and check if its equal to the first one
    int pose_min = img.height * img.step;
    int pose_max = 0;
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            // Save the pixel position into a vector to avaliate later
            pixel_pose.push_back(i);
        }
    }

    // Iterate through the vector to discover the max value of the position
    for (unsigned int i = 0; i < pixel_pose.size(); i++) {
        if (pixel_pose[i] > pose_max) {
             pose_max = pixel_pose[i];
        }
    }
    // Iterate through the vector to discover the min value of the position
    for (unsigned int i = 0; i < pixel_pose.size(); i++) {
        if (pixel_pose[i] < pose_min) {
             pose_min = pixel_pose[i];
        }
    }

    // Calculate the index of the min column and line numbers;
    int col_min = pose_min % img.step;
    int lin_min = (int)(pose_min/img.step) + 1 ;

    // Calculate the index of the max column and line numbers;
    int col_max = pose_max % img.step;
    int lin_max = (int)(pose_max/img.step) + 1 ;
    
    // Adjust the code to fix wrong index at the right border
    if (col_min == 0) {
    col_min = img.step;
    lin_min = lin_min - 1;
    }

    // Adjust the code to fix wrong index at the right border
    if (col_max == 0) {
    col_max = img.step;
    lin_max = lin_max - 1;
    }

    //Calculating the center of the ball (max_col - min_com)
    int ball_center_X = ((col_max - col_min) / 2 ) + col_min;
    int ball_center_Y = ((lin_max - lin_min) / 2 ) + lin_min;

    // Calculate an index of distance from the ball based on the ball width on the frame
    int distance = lin_max - lin_min;

    // Calculate the percentage of the pixel positions in relation to the frame 
    float X_perc = ball_center_X / (float)img.step;
    float Y_perc = ball_center_Y / (float)img.height;

    // Correct a bug when the X_perc and Y_perc is too small
    if (X_perc < 0.02) {
        X_perc = 1;
    }
    if (Y_perc < 0.02) {
        Y_perc = 1;
    }

    // Set parameters for Feedback control
    float angular_ref = 0.5;
    float angular_error = angular_ref - X_perc;
    int linear_ref = 400;
    int linear_error = linear_ref - distance;

    // Set parameter for Proportional & Integral controller - PI
    float Kp_linear = 0.001;
    float Ki_linear = 0.00005;
    float Kp_angular = 10;
    float Ki_angular = 0.01;

    // Fix error values when the ball is out of the frame
    if (linear_error == 1200) {
        linear_error = 0;
    }
    if (angular_error == -0.5) {
        angular_error = 0.5;
    }

    // Update the values of error in the vector 
    linear_error_vector.push_back(linear_error); 
    angular_error_vector.push_back(angular_error);   
  
    // erase the positions of the vector with older values | Keep the vector with defined length
    int vector_linear_length = 20; // It corresponds to the Integration Time 
    for (int k = 1; k < linear_error_vector.size(); k++){
        if (k > vector_linear_length) {	
            linear_error_vector.erase(linear_error_vector.begin());
        }
    }
    // Calculate the sum of the values inside the vector
    // It corresponds to the Integral of errors related to defined Integration time
    float sum_linear_errors = 0; 
    for (int j = 1; j < linear_error_vector.size(); j++) {
        sum_linear_errors = sum_linear_errors + linear_error_vector[j];
    }

    // erase the positions of the vector with older values | Keep the vector with defined length
    int vector_angular_length = 20;// It corresponds to the Integration Time 
    for (int m = 1; m < angular_error_vector.size(); m++){
        if (m > vector_angular_length) {	
            angular_error_vector.erase(angular_error_vector.begin());
        }
    }
    // Calculate the sum of the values inside the vector
    // It corresponds to the Integral of errors related to defined Integration time
    float sum_angular_errors = 0; 
    for (int q = 1; q < angular_error_vector.size(); q++) {
        sum_angular_errors = sum_angular_errors + angular_error_vector[q];
    }

    // Calculate the control_action for linear and angular actions based on Parallel PID structure
    float control_action_linear = (Kp_linear * linear_error) + (Ki_linear * sum_linear_errors);
    float control_action_angular = (Kp_angular * angular_error) + (Ki_angular * sum_angular_errors);

    // The control actions when the ball is out of the frame 
    // It makes the robot turn around to look for the ball somewhere
    if (X_perc == 1) {
        //if the ball is out of the frame, robot will turn around
        drive_robot(0, 2);
    } else {
        //When the ball is detected, the robot starts moving towards its direction
        drive_robot(control_action_linear ,  control_action_angular);
        }
    
    
    ROS_INFO("linear: %.2f  sum_errors: %.2f || Angular: %.2f  sum_errors: %.2f", (float)control_action_linear, (float)sum_linear_errors, (float)control_action_angular, (float)sum_angular_errors );   

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}