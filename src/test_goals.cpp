#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <redundancy_solver/GetGoals.h>
#include <redundancy_solver/GetGoalsJoints.h>
#include <redundancy_solver/SimulateGoals.h>
#include <redundancy_solver/SimulateGoalsJoints.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_goals");

  ros::NodeHandle nh;

  ros::ServiceClient goals_client = nh.serviceClient<redundancy_solver::SimulateGoalsJoints>("CBF_simulate_goals_joint");
  
  sensor_msgs::JointState goal1;
  goal1.position.resize(6);

  goal1.position[0] = -0.6;
  goal1.position[1] = -1.59;
  goal1.position[2] = -1.54;
  goal1.position[3] = -1.07;
  goal1.position[4] = 1.59;
  goal1.position[5] = 0.24;
  
  sensor_msgs::JointState goal2;
  goal2.position.resize(6);

  goal2.position[0] = -1.66;
  goal2.position[1] = -1.35;
  goal2.position[2] = -1.71;
  goal2.position[3] = -1.08;
  goal2.position[4] = 0.93;
  goal2.position[5] = -1.5;

  sensor_msgs::JointState goal3;
  goal3.position.resize(6);

  goal3.position[0] = -1.57;
  goal3.position[1] = -1.41;
  goal3.position[2] = -1.99;
  goal3.position[3] = -1.31;
  goal3.position[4] = 1.57;
  goal3.position[5] = -1.0;

  sensor_msgs::JointState goal4;
  goal4.position.resize(6);

  goal4.position[0] = -0.87;
  goal4.position[1] = -1.69;
  goal4.position[2] = -1.95;
  goal4.position[3] = -0.69;
  goal4.position[4] = 1.57;
  goal4.position[5] = -0.5;

  std::vector<sensor_msgs::JointState> pose_vec{goal1, goal2, goal3, goal4};

  redundancy_solver::SimulateGoalsJoints srv;

  srv.request.goals = pose_vec;

  goals_client.call(srv);


  return 0;
}

