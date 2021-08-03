//#include "redundancy_solver.h"
#include "redundancy_solver_ur.h"
#include <signal.h>

void mySigintHandler(int sig){
	ros::NodeHandle nh;

	ros::Publisher arm_pose_pub;
	arm_pose_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

	std_msgs::Float64MultiArray stop_msg;
	std::vector<double> stop_vector;
	stop_vector.resize(6, 0.0);

	stop_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	stop_msg.layout.dim[0].size = stop_vector.size();
	stop_msg.layout.dim[0].stride = 1;
	stop_msg.layout.dim[0].label = "stop_vel";

	stop_msg.data.clear();
	stop_msg.data.insert(stop_msg.data.end(), stop_vector.begin(), stop_vector.end());

	arm_pose_pub.publish(stop_msg);

	ros::shutdown();
}

int main(int argc, char **argv){

		ros::init(argc, argv, "redundancy_solver",ros::init_options::NoSigintHandler);
		RedundancySolverUR* solver = new RedundancySolverUR();

		signal(SIGINT, mySigintHandler);

		ros::Rate r(1000); // Was 500
		while(ros::ok()) {

			ros::spinOnce();
			solver->spin();
			r.sleep();
		}
		
		delete solver;

//return 0;

}
