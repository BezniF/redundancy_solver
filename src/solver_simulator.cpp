#include "solver_simulator.h"
#include "solver.h"
#include "solver.c"
#include "ldl.c"
#include "matrix_support.c"
#include "util.c"

SolverSimulator::SolverSimulator() {

    // Subscribers
    joints_pos_sub_= nh_.subscribe("/joint_states", 1, &SolverSimulator::jointPosCb, this);
    joints_pos_sim_sub_= nh_.subscribe("joint_values", 1, &SolverSimulator::jointPosSimCb, this);
    joy_sub_ = nh_.subscribe("/joy", 1, &SolverSimulator::joyCallback, this);
    obs_sub_ = nh_.subscribe("/vrpn_client_node/supporto/pose", 1, &SolverSimulator::obsCallback, this);
    obs2_sub_ = nh_.subscribe("/vrpn_client_node/ostacolo_cbf/pose", 1, &SolverSimulator::obs2Callback, this);
    obs_sim_sub_ = nh_.subscribe("/obs_pose", 1, &SolverSimulator::obsSimCallback, this);
    ur_base_sub_ = nh_.subscribe("/vrpn_client_node/ur_base/pose", 1 , &SolverSimulator::urBaseCallback, this);

    //Publishers
    arm_pose_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    // Service servers
    simulateGoalsServer = nh_.advertiseService("/CBF_simulate_goals", &SolverSimulator::setGoalsCallback, this);
    simulateGoalsJointsServer = nh_.advertiseService("/CBF_simulate_goals_joint", &SolverSimulator::setGoalsJointsCallback, this);
    changeSafetyServer = nh_.advertiseService("/CBF_change_safety_sim", &SolverSimulator::changeSafetyCallback, this);

    // Service clients
    setGoalsClient = nh_.serviceClient<redundancy_solver::GetGoalsJoints>("/CBF_set_goals_joint");
    changeSafetyRealClient = nh_.serviceClient<redundancy_solver::ChangeSafetyDistance>("/CBF_change_safety_real");
    
    first_cycle_ = true;
    ur_base_update_ = true;

    joint_limits.resize(6);
    joint_limits = {2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI};

    acceleration_max_.resize(6);
    dotq_max_.resize(6);
    dotq_prev_.resize(6);
    dotq_vec.resize(6);
    acceleration_max_ = {2.61, 2.61, 2.61, 2.61, 2.61, 2.61};
    dotq_max_ = {1.05, 1.05, 1.57, 1.57, 1.57, 1.57};
    dotq_prev_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    cycle_time = 0.002;
    D = 0.30; //was 0.30

    //Bools
    sum_of_delta_ =  0.0;
    disable_cbf_ = false;

    // TF structs
    tfBuffer = new tf2_ros::Buffer;
	tf2_listener = new tf2_ros::TransformListener(*tfBuffer);

    // MoveIt structs
    robot_model_loader = new robot_model_loader::RobotModelLoader ("robot_description");

    start_time_ = ros::Time::now().toSec();

    //! Change according to which robot is in use
    REAL_ROBOT_ = true;

    //! Change according to the threshold for considering the goal reached
    GOAL_TOLERANCE_ = 0.01; // was 0.03

}

SolverSimulator::~SolverSimulator() {

    // Close the Ostream
    tank_file_.close();
    cbf_file_.close();
    force_file_.close();
    dotq_file_.close();
    dist_file_.close();
    pose_file_.close();
    potential_file_.close();

}

//* Callback for debugging stuff using the joystick
void SolverSimulator::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    //TODO
}

//* Callbacks for setting initial values
void SolverSimulator::jointPosCb(const sensor_msgs::JointStateConstPtr& msg){
    if(REAL_ROBOT_){
        initial_pos_joints_ =  *msg;

        for(int i=0; i < 6; i++){
            initial_pos_array[i] = initial_pos_joints_.position[i];
            initial_vel_array[i] = initial_pos_joints_.velocity[i];
        }
        double tmp_pos = initial_pos_array[2];
        initial_pos_array[2] = initial_pos_array[0];
        initial_pos_array[0] = tmp_pos;

        double tmp_vel = initial_vel_array[2];
        initial_vel_array[2] = initial_vel_array[0];
        initial_vel_array[0] = tmp_vel;
    }

}

void SolverSimulator::jointPosSimCb(const std_msgs::Float64MultiArrayConstPtr& msg){
    if(!REAL_ROBOT_){
        std_msgs::Float64MultiArray  array_msg = *msg;

        for(int i=0; i < 6; i++){
            initial_pos_array[i] = array_msg.data[i];
        }
    }
}

void SolverSimulator::obsCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    if(REAL_ROBOT_){
        geometry_msgs::PoseStamped  pose_msg = *msg;

        obs_pose.x = pose_msg.pose.position.x;
        obs_pose.y = pose_msg.pose.position.y;
        obs_pose.z = pose_msg.pose.position.z;

        obs_pose_stamped = *msg;
    }

}

void SolverSimulator::obs2Callback(const geometry_msgs::PoseStampedConstPtr& msg){
    if(REAL_ROBOT_){
        geometry_msgs::PoseStamped  pose_msg = *msg;

        obs2_pose.x = pose_msg.pose.position.x;
        obs2_pose.y = pose_msg.pose.position.y;
        obs2_pose.z = pose_msg.pose.position.z;

        obs2_pose_stamped = *msg;
    }

}

void SolverSimulator::obsSimCallback(const geometry_msgs::PointConstPtr& msg){
    if(!REAL_ROBOT_){
        geometry_msgs::Point  point_msg = *msg;

        obs_pose.x = -point_msg.x;
        obs_pose.y = -point_msg.y;
        obs_pose.z = point_msg.z;
    }

}

void SolverSimulator::urBaseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    if(REAL_ROBOT_){
        if(ur_base_update_){
            geometry_msgs::PoseStamped  base_pose_msg = *msg;

            ur_base_pose.x = base_pose_msg.pose.position.x;
            ur_base_pose.y = base_pose_msg.pose.position.y;
            ur_base_pose.z = base_pose_msg.pose.position.z;

            ur_base_update_ = false;
        }
    }

}

// ****************** SERVICE HANDLERS*************************//

bool SolverSimulator::setGoalsCallback(redundancy_solver::SimulateGoalsRequest &req, redundancy_solver::SimulateGoalsResponse &res){
    // Simulate the trajectory composed by the requested goals
    // bool result = simulateTrajectory(req.goals);

    // res.success = result;

    // if(!result)
    //     res.distance = 0.5 * D_goal_obs_; // TODO: tune this parameter
    // else
    //     res.distance = 0.0;

    // if(result){
    //     redundancy_solver::GetGoals srv;
    //     srv.request.goals = req.goals;
    //     setGoalsClient.call(srv);
    // }

    // return result;
}

bool SolverSimulator::setGoalsJointsCallback(redundancy_solver::SimulateGoalsJointsRequest& req, redundancy_solver::SimulateGoalsJointsResponse& res){
    // Simulate the trajectory composed by goals in the joint space
    bool result = simulateTrajectory(req.goals);

    res.success = result;

    if(!result)
        res.distance = 0.5 * D_goal_obs_; // TODO: tune this parameter
    else
        res.distance = 0.0;

    if(result){
        redundancy_solver::GetGoalsJoints srv;
        srv.request.goals = req.goals;
        setGoalsClient.call(srv);
    }

    return result;
}

bool SolverSimulator::changeSafetyCallback(redundancy_solver::ChangeSafetyDistanceRequest& req, redundancy_solver::ChangeSafetyDistanceResponse& res){
    D = req.distance;

    redundancy_solver::ChangeSafetyDistance srv;
    srv.request.distance = req.distance;
    changeSafetyRealClient.call(srv);

    return true;
}

bool SolverSimulator::simulateTrajectory(std::vector<sensor_msgs::JointState> vec){

    std::vector<sensor_msgs::JointState> goals_vec =  vec;

    double sim_pos_array[6];
    std::copy(std::begin(initial_pos_array), std::end(initial_pos_array), std::begin(sim_pos_array));

    bool timeout = false;

    for(int i=0; i < goals_vec.size(); i++){

        //geometry_msgs::Pose current_goal = goals_vec[i];
        sensor_msgs::JointState current_goal = goals_vec[i];
        bool goal_reached = false;

        // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        // tf::Quaternion quat;
        // tf::quaternionMsgToTF(current_goal.orientation, quat);

        // // Access roll pitch and yaw
        // double roll, pitch, yaw;
        // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // // the found angles are written in a Eigen::Vector3d
        // Eigen::Vector3d rpy_goal;
        // rpy_goal[0] = roll;
        // rpy_goal[1] = pitch;
        // rpy_goal[2] = yaw;

        double timeout_time = 5.0;
        double current_time = ros::Time::now().toSec();

        while (ros::Time::now().toSec() - current_time <= timeout_time){
            // ********* INIALIZE MODEL AND READ POSE HERE ********* //

            // Instantiating the robot model
            robot_model::RobotModelPtr kinematic_model = robot_model_loader->getModel();

            // Creating the robot kinematic state and setting it to the current joints positions
            robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
            kinematic_state->setVariablePositions(sim_pos_array);
            const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
            const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

            // Pasting the kinematic model to the Joint Groups in MoveIt!
            std::vector<double> joint_values;
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

            // Computing the Jacobian of the arm
            Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
            Eigen::MatrixXd jacobian_arm;
            kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                        reference_point_position,
                                        jacobian_arm);

            const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");

            // Compute the pose of the end-effector with respect to the /base frame //? are we sure?
            Eigen::Vector3d ee_pose_linear = end_effector_state.translation();
            Eigen::Matrix3d ee_pose_angular = end_effector_state.rotation();

            ROS_INFO_STREAM("EE pose: \n" << ee_pose_linear);

            // Computing current RPY of the EE using tf (n.b. Eigen sucks at this)
            double roll_ee, pitch_ee, yaw_ee;

            geometry_msgs::TransformStamped rot_msg = tf2::eigenToTransform(end_effector_state);

            tf2::Quaternion quat;
            tf2::convert(rot_msg.transform.rotation, quat);
            tf2::Matrix3x3(quat).getRPY(roll_ee, pitch_ee, yaw_ee);

            // Correct the Jacobian for a pose control application
            double R,P,Y;

            R = roll_ee;
            P = pitch_ee;
            Y = yaw_ee;

            Eigen::Matrix3d T_rpy;
            T_rpy << cos(Y)*cos(P),  -sin(Y), 0, 
                     sin(Y)*cos(P),   cos(Y), 0,
                           -sin(P),        0, 1;

            Eigen::MatrixXd transform_RPY_arm(6,6);
            transform_RPY_arm <<Eigen::Matrix3d::Identity(3,3), 
                            Eigen::Matrix3d::Zero(3,3),
                            Eigen::Matrix3d::Zero(3,3),
                            T_rpy.inverse();

            jacobian_arm = transform_RPY_arm * jacobian_arm;

            // **************** TRANSFORMS HERE **************** //
            //* Jacobian's Mapping and Inverse
            Eigen::MatrixXd J_inv = jacobian_arm.inverse();

            /***************************************************************************************/
            /*************************************PENALTY MATRIX************************************/
            /***************************************************************************************/
            Eigen::VectorXd m(6);
            for(int i = 0; i < 6; i++)
                m[i] = 1.0;

            if(interaction_){
                for(int i= 0; i < 6; i++){
                    m[i] = 50000.0; // was 50.0
                }
            }

            for(int i = 0; i < 6; i++)
                params.M[i] = m[i];

            /***************************************************************************************/
            /**********************LOADING THE DATA FOR THE OPTIMIZATION PROBLEM********************/
            /***************************************************************************************/
            set_defaults();  // Set basic algorithm parameters.
            setup_indexing();
            settings.max_iters = 20; //* SOLVER PARAMETERS
            //settings.eps = 1e-9;
            //settings.resid_tol= 1e-2;

            // LOAD THE CURRENT JOINT STATE OF THE END-EFFECTOR
            params.sigma[0] = sim_pos_array[0];
            params.sigma[1] = sim_pos_array[1];
            params.sigma[2] = sim_pos_array[2];
            params.sigma[3] = sim_pos_array[3];
            params.sigma[4] = sim_pos_array[4];
            params.sigma[5] = sim_pos_array[5];

            // LOAD THE CURRENT POSE OF THE END EFFECTOR
            params.x[0] = ee_pose_linear[0];
            params.x[1] = ee_pose_linear[1];
            params.x[2] = ee_pose_linear[2];
            params.x[3] = roll_ee;
            params.x[4] = pitch_ee;
            params.x[5] = yaw_ee;

            // SELECT HERE THE DESIRED FINAL END-EFFECTOR POSITION
            params.sigma_0[0] = current_goal.position[0];
            params.sigma_0[1] = current_goal.position[1];
            params.sigma_0[2] = current_goal.position[2];
            params.sigma_0[3] = current_goal.position[3];
            params.sigma_0[4] = current_goal.position[4];
            params.sigma_0[5] = current_goal.position[5];

            // SELECT HERE THE DESIRED FINAL END-EFFECTOR VELOCITY
            params.Sigma[0] = 0.0;
            params.Sigma[1] = 0.0;
            params.Sigma[2] = 0.0;
            params.Sigma[3] = 0.0;
            params.Sigma[4] = 0.0;
            params.Sigma[5] = 0.0;

            // SELECT HERE THE POSE OF THE OBSTACLE AND THE DESIRED DISTANCE TO BE KEPT //!
            if(REAL_ROBOT_){
                //* Transform the obstacle pose into the real ur_base reference frame
                geometry_msgs::TransformStamped transform = tfBuffer->lookupTransform("ur_base", "supporto",
                                    ros::Time(0), ros::Duration(3.0));

                params.sigma_obs[0]= transform.transform.translation.x;
                params.sigma_obs[1]= transform.transform.translation.y;
                params.sigma_obs[2]= transform.transform.translation.z;
                params.sigma_obs[3]= roll_ee;
                params.sigma_obs[4]= pitch_ee;
                params.sigma_obs[5]= yaw_ee;

                geometry_msgs::TransformStamped transform2 = tfBuffer->lookupTransform("ur_base", "ostacolo_cbf",
                                                ros::Time(0), ros::Duration(3.0));

                params.sigma_obs2[0]= transform2.transform.translation.x;
                params.sigma_obs2[1]= transform2.transform.translation.y;
                params.sigma_obs2[2]= transform2.transform.translation.z;
                params.sigma_obs2[3]= roll_ee;
                params.sigma_obs2[4]= pitch_ee;
                params.sigma_obs2[5]= yaw_ee;

            }
            else{
                params.sigma_obs[0]= obs_pose.x;
                params.sigma_obs[1]= obs_pose.y;
                params.sigma_obs[2]= obs_pose.z;
                params.sigma_obs[3]= roll_ee;
                params.sigma_obs[4]= pitch_ee;
                params.sigma_obs[5]= yaw_ee;

                // Randomly initialize the second obstacle since it is not present in the simulation
                params.sigma_obs2[0]=2.5;
                params.sigma_obs2[1]=2.5;
                params.sigma_obs2[2]=2.5;
                params.sigma_obs2[3]=roll_ee;
                params.sigma_obs2[4]=pitch_ee;
                params.sigma_obs2[5]=yaw_ee;
            }

            
            /***************************************************************************************/
            /*************************************EXECUTION CBFs************************************/
            /***************************************************************************************/

            //* SAFETY CBF FOR OBSTACLE AVOIDANCE
            double d_2 = 0.0;

            for(int i= 0; i< 3; i++){
                d_2 += pow(params.x[i] - params.sigma_obs[i] , 2);
            }

            double d = sqrt(d_2);
            double gamma_safe = 50.0; // was 50.0

            // Definition of h_safe
            params.h_safe[0] =  - gamma_safe * (d_2 - pow(D,2));

            if(params.h_safe[0] > 0.0){

                ROS_INFO_STREAM("STOP RIGHT THERE CRIMINAL SCUM!");
                ROS_INFO_STREAM("d: " << d);
            }

            //* SAFETY CBF FOR SECOND OBSTACLE AVOIDANCE
            double d_tb2 = 0.0;

            for(int i= 0; i< 3; i++){
                d_tb2 += pow(params.x[i] - params.sigma_obs2[i] , 2);
            }

            double d_tb = sqrt(d_tb2);

            params.h_safe_2[0] =  - gamma_safe * (d_tb2 - pow(2*D,2));

            if(params.h_safe_2[0] > 0.0){

                ROS_INFO_STREAM("STOP RIGHT THERE CRIMINAL TURTLEBOT!");
                ROS_INFO_STREAM("d: " << d_tb);
            }

            //* GOAL CBF h1 = - 1/2 * square(norm(sigma-sigma_0)) FOR TASK COMPLETION
            double quad_norm = 0.0;
            double gamma_goal = 1.0; //was 5.0

            for(int i=0; i<6; i++)
                quad_norm += pow(params.sigma[i] - params.sigma_0[i], 2);

            // for(int i=3; i<6; i++){
            //     quad_norm += pow(atan2(sin(params.sigma_0[i] - params.sigma[i]), cos(params.sigma_0[i] - params.sigma[i])),2);
            // }

            double goal_d;
            goal_d = sqrt(quad_norm);

            // Parameters to activate or deactivate the slack variables' influence
            params.theta[0] = 0.0;
            params.theta[1] = 0.0;

            params.h_goal[0] = - gamma_goal * quad_norm;
            // ROS_INFO_STREAM("Goal CBF: " << params.h_goal[0]);

            //* LIMIT CBF h_lim TO AVOID JOINT LIMITS
            double h2 = 0.0;
            double h_lim_array[6];

            for(int i=0; i<6; i++){
                params.Q_lim[i] = ( (joint_limits[i] - 2 * sim_pos_array[i] + (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
            }

            for(int i=0; i<6; i++){
                params.h_lim[i] =  -0.5 * ( (joint_limits[i] - sim_pos_array[i]) * (sim_pos_array[i] - (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
                h_lim_array[i] = -0.5 * ( (joint_limits[i] - sim_pos_array[i]) * (sim_pos_array[i] - (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
            }

            // ROS_INFO_STREAM("Limits CBF: " << params.h_lim[0]);

            // LOAD THE CURRENT CONFIGURATION'S JACOBIAN
            // *NdR: CVXgen stores matrices as flat-arrays in column major form (namely Aij = params.A[(i-1) + (j-1)*m)])
            int n = 0;
            for(int j=0; j < jacobian_arm.cols(); j++){
                for(int i=0; i < jacobian_arm.rows(); i++){

                    params.J[n] = jacobian_arm(i,j);
                    n++;
                }
            }

            // HANDLE THE RELAXATION OF TASKS VIA SLACK VARIABLES
            params.l[0] = 60.0; // was 60 // TODO: CONSIDERARE L'IDEA DI USARE UNA MATRICE QUADRATA PER PESARE I SINGOLI DELTA E NON UN GUADAGNO PROPORZIONALE

            // IMPOSING SPEED AND ACCELERATION LIMITS
            params.dotq_max[0] = 0.15;
            params.dotq_min[0] = -0.15;

            for(int i = 0; i< 6; i++)
                params.a_max[i] = 0.25 * cycle_time;

            for(int i=0; i<6 ; i++)
                params.dotq_prev[i] = dotq_prev_[i];

            //************ADMITTANCE AND TANK PARAMS WHICH ARE NOT NEEDED FOR THE SIMULATION**************//
            for(int i = 0; i< 6; i++)
                params.dotq_adm[i] = 0.0;
            // for(int i = 0; i < 6; i++)
            //     params.A[i] = 0.0;
            // params.B[0] = 50;

            /***************************************************************************************/
            /*******************PUBLISH THE SOLUTION OF THE OPTIMIZATION PROBLEM********************/
            /***************************************************************************************/
            settings.verbose = 0;
            settings.max_iters = 50;
            long num_iters = solve();

            for(int i = 0; i < 6; i++){
                if(isnan(vars.dotq[i])){
                    vars.dotq[i] = 0.0;
                    ROS_ERROR_STREAM("OUTPUT NON VALIDO!");
                }
            }

            for(int i = 0; i < 2; i++){
                if(isnan(vars.delta[i])){
                    vars.delta[i] = 0.0;
                    ROS_ERROR_STREAM("SLACK VARIABLES NON VALIDE!");
                }
            }

            // ROS_INFO_STREAM("Delta 1: " << vars.delta[0]);
            // ROS_INFO_STREAM("Delta 2: " << vars.delta[1]);

            //* UPDATE THE TANK VARIABLES
            Eigen::VectorXd q_dot(6);
            Eigen::VectorXd x_dot(6);

            for(int i = 0; i< q_dot.size(); i++){
                q_dot[i] = vars.dotq[i];
                dotq_prev_[i] = vars.dotq[i];
            }

            for(int i = 0; i<6; i++){
                //ROS_INFO_STREAM("DOTQ" << i  <<": " << vars.dotq[i]);
                if((vars.dotq[i] > 1.58 ) || (vars.dotq[i] < -1.58))
                    ROS_ERROR_STREAM("VAI TROPPO VELOCE OHHH CALMA!!!!!!!");
            }

            x_dot = jacobian_arm * q_dot;

            // ROS_WARN_STREAM("X POST OPT: " << x_dot[0]);
            // ROS_WARN_STREAM("Y POST OPT: " << x_dot[1]);
            // ROS_WARN_STREAM("Z POST OPT: " << x_dot[2]);

            cycle_time = 0.002;

            //* Create the messages and publishing them
            trajectory_msgs::JointTrajectory arm_pose;
            arm_pose.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            arm_pose.points.resize(1);
            arm_pose.points[0].time_from_start = ros::Duration(0.001);

            //Update the simulated joints position
            double delta_q[6] = {q_dot[0] * cycle_time, 
                                q_dot[1] * cycle_time, 
                                q_dot[2] * cycle_time,
                                q_dot[3] * cycle_time,
                                q_dot[4] * cycle_time,
                                q_dot[5] * cycle_time};

            for(int i = 0; i < 6; i++)
                sim_pos_array[i] += delta_q[i];

            // Computing the distance goal-obstacle
            double d_obs_goal_sqr = 0.0;
            double d_obs2_goal_sqr = 0.0;

            // Compute the cartesian position of the obstacle
            kinematic_state->setJointGroupPositions(joint_model_group, params.sigma_0);
            const Eigen::Affine3d& obs_pose_cartesian= kinematic_state->getGlobalLinkTransform("ee_link");
            auto obstacle_xyz = obs_pose_cartesian.translation();

            for(int i = 0; i < 3; i++)
                d_obs_goal_sqr += pow(params.sigma_obs[i] - obstacle_xyz[i],2);

            for(int i = 0; i < 3; i++)
                d_obs2_goal_sqr += pow(params.sigma_obs2[i] - obstacle_xyz[i],2);

            D_goal_obs_ = std::min(sqrt(d_obs_goal_sqr),sqrt(d_obs2_goal_sqr));

            // Computing the distance from the goal and using it as a break condition
            ROS_WARN_STREAM("Distance from goal: " << goal_d);

            if(goal_d <= GOAL_TOLERANCE_){
                goal_reached = true;
                break;
            }
            /**********************************************/
            /******************** DEBUG *******************/
            /**********************************************/

        }
        if(goal_reached)
            ROS_INFO_STREAM("Hooray, the goal number " << i + 1 << " can be reached!");
        else{
            ROS_ERROR_STREAM("TIMEOUT: the goal number " << i + 1 << " cannot be reached in the given time!");
            timeout = true;
            break;
        }
    }

    ROS_INFO_STREAM("timeout: " << timeout);

    bool result = !(timeout);

    ROS_INFO_STREAM("result: " << result);

    return result;
}

//* Spinner
void SolverSimulator::spin(){
    //TODO
}