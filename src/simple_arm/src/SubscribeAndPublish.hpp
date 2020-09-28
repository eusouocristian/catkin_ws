#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include "simple_arm/GoToPosition.h"

class SubscribeAndPublish
{
public:
  void SAPMethod();
  void joint_states_callback(const sensor_msgs::JointState js);
  void look_away_callback(const sensor_msgs::Image img);
  void move_arm_center();
  std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2);
private:
  ros::Subscriber sub1, sub2;
  ros::Publisher pub1, pub2;
  std::vector<double> joints_last_position{ 0, 0 };
  bool moving_state;
};
