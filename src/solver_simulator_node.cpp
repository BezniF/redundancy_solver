#include "solver_simulator.h"

int main(int argc, char **argv){

		ros::init(argc, argv, "solver_simulator");
		SolverSimulator* sim = new SolverSimulator();

		ros::Rate r(1000); // Was 500
		while(ros::ok()) {

			ros::spinOnce();
			r.sleep();
		}
		
		delete sim;

}
