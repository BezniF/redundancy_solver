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
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

// custom srvs
#include <redundancy_solver/GetGoals.h>
#include <redundancy_solver/GetGoalsJoints.h>
#include <redundancy_solver/ChangeSafetyDistance.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

extern "C" {
    #include "solver.h"
}

//#include <rossini_msg/planned_trajectory_msg.h>

class RedundancySolverUR {

		public:

			RedundancySolverUR();
			~RedundancySolverUR();
			void spin();

			// Callbacks
			//void jointPosCb(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
			void jointPosSimCb(const std_msgs::Float64MultiArrayConstPtr& msg);
			void jointPosCb(const sensor_msgs::JointStateConstPtr& msg);
			void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
			void forceCallback(const geometry_msgs::WrenchStamped& msg);
			void obsCallback(const geometry_msgs::PoseStampedConstPtr& msg);
			void obs2Callback(const geometry_msgs::PoseStampedConstPtr& msg);
			void obsSimCallback(const geometry_msgs::PointConstPtr& msg);
			void urBaseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
			bool setGoalsCallback(redundancy_solver::GetGoalsRequest& req, redundancy_solver::GetGoalsResponse& res);
			bool setGoalsJointsCallback(redundancy_solver::GetGoalsJointsRequest& req, redundancy_solver::GetGoalsJointsResponse& res);
			bool stopRobotCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
			bool resumeRobotCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
			bool changeSafetyCallback(redundancy_solver::ChangeSafetyDistanceRequest& req, redundancy_solver::ChangeSafetyDistanceResponse& res);
			void updateGoal();

		private:

			// NodeHandle
        	ros::NodeHandle nh_;

			// Subscribers
			ros::Subscriber joints_pos_sub_;
			ros::Subscriber joints_pos_sim_sub_;
			ros::Subscriber joy_sub_;
			ros::Subscriber wrench_sub_;
			ros::Subscriber obs_sub_;
			ros::Subscriber obs2_sub_;
			ros::Subscriber obs_sim_sub_;
			ros::Subscriber ur_base_sub_;

			// Publisher
			ros::Publisher base_vel_pub_;
			ros::Publisher arm_pose_pub_;
			ros::Publisher ee_tip_pub_;
			ros::Publisher tank_pub_;
			ros::Publisher safety_pub_;

			// ROS Services
			ros::ServiceServer setGoalsServer;
			ros::ServiceServer setGoalsJointsServer;
			ros::ServiceServer stopRobotServer;
			ros::ServiceServer resumeRobotServer;
			ros::ServiceServer changeSafetyServer;
			ros::ServiceClient goalReachedClient;
			ros::ServiceClient taskFinishedClient;
			ros::ServiceClient warningMotionClient;

			// Initial variables
			// control_msgs::JointTrajectoryControllerState initial_pos_joints_;
			sensor_msgs::JointState initial_pos_joints_;
			double initial_pos_array[6];
			double initial_vel_array[6]; 
			nav_msgs::Odometry initial_base_odom_;
			geometry_msgs::Pose initial_base_pose_;
			geometry_msgs::Twist initial_vel_base_;
			geometry_msgs::Point obs_pose;
			geometry_msgs::Point obs2_pose;
			geometry_msgs::Point ur_base_pose;
			geometry_msgs::PoseStamped obs_pose_stamped;
			geometry_msgs::PoseStamped obs2_pose_stamped;
			double roll_ee, pitch_ee, yaw_ee;
			Eigen::Vector3d ee_pose_linear;

			// Cycle control variables
			bool first_cycle_;
			bool disable_cbf_;
			bool enable_force_1_;
			bool enable_force_2_;
			double current_time_;
			double ur_base_update_;
			double start_;
			double cycle_time;
			bool interaction_;
			std::vector<double> dotq_vec;
			double goal_distance_;
			double GOAL_TOLERANCE_;
			
			// Mode selectors
			bool REAL_ROBOT_;
			bool ROBOT_STOPPED_;
			bool JOINT_SPACE_;

			// TF variables
			tf2_ros::Buffer* tfBuffer;
			tf2_ros::TransformListener* tf2_listener;

			// MoveIt! model loaders
			robot_model_loader::RobotModelLoader* robot_model_loader;

			// CBF variables
			std::vector<float> joint_limits;
			std::vector<double> velocity_commands;
			std::vector<double> position_commands;
			std::vector<double> acceleration_max_;
			std::vector<double> dotq_max_;
			std::vector<double> dotq_prev_;
			std::vector<double> goal_pose_rpy_;
			std::vector<double> goal_joint_space_;
			double D;
			std::vector<geometry_msgs::Pose> goals_vector_;
			std::vector<sensor_msgs::JointState> goals_joints_vector_;

			// Admittance parameters
			Eigen::Matrix<double, 6, 6> M_a_,M_p_,D_a_,D_p_;
			Eigen::Matrix<double, 6, 1> desired_twist_; // given by the Master
			Eigen::Matrix<double, 6, 1> F_ext_;
			Eigen::Matrix<double, 6, 6> M_tot_, D_tot_;
			Eigen::Matrix<double, 6, 6> M_prev_, dot_M_;

			// Tank parameters
    		double ext_b_;
    		float tank_state_;
    		float tank_energy_;
    		double xt_dot_;
			const float TANK_INITIAL_VALUE = 5.2; // was 20
    		const float TANK_MAX_VALUE = 500;
    		const float TANK_MIN_VALUE = 5;
    		double sum_y2_;

			// Tank Based Admittance Control Problem Variables
			double sum_of_delta_;

			// Output to file
			std::ofstream tank_file_;
			std::ofstream cbf_file_;
			std::ofstream force_file_;
			std::ofstream potential_file_;
			std::ofstream dotq_file_;
			std::ofstream dist_file_;
			std::ofstream pose_file_;
			double start_time_;

			// KDL stuff
			KDL::Chain ur10e;
			KDL::ChainFkSolverPos_recursive* fksolver;//Forward position solver
			KDL::ChainJntToJacSolver* jsolver;//Jacobian solver

			KDL::JntArray q; //Future position of the robot
			KDL::Frame ee_pose; //Desired cartesian position
			KDL::Jacobian j; //Jacobian
			std::vector<bool> lock_joints; //Locked joints to compute jacobian

};

#endif /* REDUNDANCY_SOLVER_H */
