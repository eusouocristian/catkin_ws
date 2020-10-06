#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>

// Define a global client that can request services
ros::ServiceClient client;

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

    // ROS_INFO("X= %.2f    Y= %.2f    Distance: %d", X_perc, Y_perc, distance);

    // Set parameters for Proportional controller P
    float X_ref = 0.5;
    float X_error = X_ref - X_perc;
    int distance_ref = 400;
    int distance_error = distance_ref - distance;
    //Action of 5 when the error is 0.4
    float P_X = 5/0.4;
    //Action of 0.5 when the error is 400
    float P_distance = 0.5/400;

    
    // int test;

    // if (not test) {
    //     teste = 1;
    // }
    // ROS_INFO_STREAM( "Test: " << teste);




    // The control actions 
    if (X_perc == 1) {
        //if the ball is out of the frame, robot will turn around
        drive_robot(0, 2);
    } else {
        //When the ball is detected, the robot starts movinto on its direction
        drive_robot(P_distance * distance_error, P_X * X_error);

    }


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