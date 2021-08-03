#ifndef REDUNDANCY_SOLVER_H_
#define REDUNDANCY_SOLVER_H_
#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <fstream>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>

extern "C" {
    #include "solver.h"
}

//#include <rossini_msg/planned_trajectory_msg.h>

class RedundancySolver {

		public:

			RedundancySolver();
			~RedundancySolver();
			void spin();

			// Callbacks
			void jointPosCb(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
			void basePosCb(const nav_msgs::OdometryConstPtr& msg);
			void joyCallback(const sensor_msgs::Joy::ConstPtr&);
			void forceCallback(const geometry_msgs::WrenchStamped& msg);

		private:

			// NodeHandle
        	ros::NodeHandle nh_;

			// Subscribers
			ros::Subscriber joints_pos_sub_;
			ros::Subscriber base_pose_sub_;
			ros::Subscriber joy_sub_;
			ros::Subscriber wrench_sub_;

			// Publisher
			ros::Publisher base_vel_pub_;
			ros::Publisher arm_pose_pub_;
			ros::Publisher ee_tip_pub_;

			// Clients
			//ros::ServiceClient get_manipulator_info_client;
			//prbt_planner::GetManipulatorInfo get_manipulator_info_srv;

			// Initial variables
			control_msgs::JointTrajectoryControllerState initial_pos_joints_;
			double initial_pos_array[6];
			double initial_vel_array[6]; 
			nav_msgs::Odometry initial_base_odom_;
			geometry_msgs::Pose initial_base_pose_;
			geometry_msgs::Twist initial_vel_base_;

			// Cycle control variables
			bool first_cycle_;
			bool disable_cbf_;
			bool enable_force_1_;
			bool enable_force_2_;
			double current_time_;
			double start_;
			double cycle_time;
			bool interaction_;

			// TF variables
			tf2_ros::Buffer* tfBuffer;
			tf2_ros::TransformListener* tf2_listener;

			// MoveIt! model loaders
			//robot_model_loader::RobotModelLoader* robot_model_loader;

			// CBF variables
			std::vector<float> joint_limits;
			std::vector<double> velocity_commands;
			std::vector<double> position_commands;

			// Admittance parameters
			Eigen::Matrix<double, 6, 6> M_a_,M_p_,D_a_,D_p_;
			Eigen::Matrix<double, 6, 1> desired_twist_; // given by the Master
			Eigen::Matrix<double, 6, 1> F_ext_;
			Eigen::Matrix<double, 6, 6> M_tot_, D_tot_;

			// Tank parameters
    		double ext_b_;
    		float tank_state_;
    		float tank_energy_;
    		double xt_dot_;
			const float TANK_INITIAL_VALUE = 20;
    		const float TANK_MAX_VALUE = 500;
    		const float TANK_MIN_VALUE = 5;
    		double sum_y2_;

			// Tank Based Admittance Control Problem Variables
			double sum_of_delta_;

			// Output to file
			std::ofstream tank_file_;
			std::ofstream cbf_file_;
			std::ofstream force_file_;
			double start_time_;


};

#endif /* REDUNDANCY_SOLVER_H */
