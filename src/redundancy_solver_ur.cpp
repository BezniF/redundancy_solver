#include "redundancy_solver_ur.h"
#include "solver.h"
#include "solver.c"
#include "ldl.c"
#include "matrix_support.c"
#include "util.c"

RedundancySolverUR::RedundancySolverUR() {

    joints_pos_sub_= nh_.subscribe("/joint_states", 1, &RedundancySolverUR::jointPosCb, this);
    joints_pos_sim_sub_= nh_.subscribe("joint_values", 1, &RedundancySolverUR::jointPosSimCb, this);
    joy_sub_ = nh_.subscribe("/joy", 1, &RedundancySolverUR::joyCallback, this);
    wrench_sub_ = nh_.subscribe("wrench", 1, &RedundancySolverUR::forceCallback, this);
    obs_sub_ = nh_.subscribe("/vrpn_client_node/supporto/pose", 1, &RedundancySolverUR::obsCallback, this);
    obs2_sub_ = nh_.subscribe("/vrpn_client_node/ostacolo_cbf/pose", 1, &RedundancySolverUR::obs2Callback, this);
    obs_sim_sub_ = nh_.subscribe("/obs_pose", 1, &RedundancySolverUR::obsSimCallback, this);
    ur_base_sub_ = nh_.subscribe("/vrpn_client_node/ur_base/pose", 1 , &RedundancySolverUR::urBaseCallback, this);

    //arm_pose_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_traj_controller/command", 1);
    arm_pose_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);
    tank_pub_ = nh_.advertise<std_msgs::Float64>("/tank_value_debug", 1);
    safety_pub_ = nh_.advertise<std_msgs::Bool>("/CBF_warning_motion_area", 1);

    // Service servers
    setGoalsServer = nh_.advertiseService("/CBF_set_goals", &RedundancySolverUR::setGoalsCallback, this);
    setGoalsJointsServer = nh_.advertiseService("/CBF_set_goals_joint", &RedundancySolverUR::setGoalsJointsCallback, this);
    stopRobotServer = nh_.advertiseService("/CBF_stop_robot", &RedundancySolverUR::stopRobotCallback, this);
    resumeRobotServer = nh_.advertiseService("/CBF_resume_robot", &RedundancySolverUR::resumeRobotCallback, this);
    changeSafetyServer = nh_.advertiseService("/CBF_change_safety_real", &RedundancySolverUR::changeSafetyCallback, this);

    // Service clients
    goalReachedClient = nh_.serviceClient<std_srvs::Trigger>("/CBF_goal_reached");
    taskFinishedClient = nh_.serviceClient<std_srvs::Trigger>("/CBF_task_finished");
    // warningMotionClient = nh_.serviceClient<std_srvs::Trigger>("/CBF_warning_motion_area");

    first_cycle_ = true;
    ur_base_update_ = true;

    joint_limits.resize(6);
    joint_limits = {2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI};

    // Vectors init
    acceleration_max_.resize(6);
    dotq_max_.resize(6);
    dotq_prev_.resize(6);
    dotq_vec.resize(6);
    goal_pose_rpy_.resize(6);
    goal_joint_space_.resize(6);
    acceleration_max_ = {2.61, 2.61, 2.61, 2.61, 2.61, 2.61};
    dotq_max_ = {1.05, 1.05, 1.57, 1.57, 1.57, 1.57};
    dotq_prev_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    cycle_time = 0.002;

    // Admittance parameters init
    std::vector<double> M_a;
    std::vector<double> D_a;
    std::vector<double> des_twist_init (6, 0.0);

    if (!nh_.getParam("mass_arm", M_a)) {
        ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    }

    if (!nh_.getParam("damping_arm", D_a)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    }

    M_a_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_a.data());
    D_a_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_a.data());

    M_tot_ = M_a_;
    D_tot_ = D_a_;

    M_prev_ = M_tot_;
    dot_M_.setZero();
        
    desired_twist_= Eigen::Map<Eigen::Matrix<double, 6, 1> >(des_twist_init.data());
    F_ext_.setZero(); // Callback from Force / Torque sensor

    tank_energy_ = TANK_INITIAL_VALUE;
    tank_state_ = sqrt(2 * tank_energy_);

    //Bools
    sum_of_delta_ =  0.0;
    disable_cbf_ = false;

    // TF structs
    tfBuffer = new tf2_ros::Buffer;
	tf2_listener = new tf2_ros::TransformListener(*tfBuffer);

    // MoveIt structs
    robot_model_loader = new robot_model_loader::RobotModelLoader ("robot_description");

    // Output to file stream
    std::stringstream file_path;
    file_path << "/home/federico/MATLAB_ws/IROS21-Alexa/src/tank.txt";
    tank_file_.open(file_path.str());

    std::stringstream file_path2;
    file_path2 << "/home/federico/MATLAB_ws/IROS21-Alexa/src/cbf.txt";
    cbf_file_.open(file_path2.str());

    std::stringstream file_path3;
    file_path3 << "/home/federico/MATLAB_ws/IROS21-Alexa/src/force.txt";
    force_file_.open(file_path3.str());

    std::stringstream file_path4;
    file_path4 << "/home/federico/MATLAB_ws/IROS21-Alexa/src/dotq.txt";
    dotq_file_.open(file_path4.str());

    std::stringstream file_path5;
    file_path5 << "/home/federico/MATLAB_ws/IROS21-Alexa/src/distance.txt";
    dist_file_.open(file_path5.str());

    std::stringstream file_path6;
    file_path6 << "/home/federico/MATLAB_ws/IROS21-Alexa/src/pose.txt";
    pose_file_.open(file_path6.str());
    
    std::stringstream file_path7;
    file_path7 << "/home/federico/MATLAB_ws/IROS21-Alexa/src/potential.txt";
    potential_file_.open(file_path7.str());

    start_time_ = ros::Time::now().toSec();

    interaction_ = false;

    //Geometry parameters DH(a, alpha, d, theta)
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,M_PI_2,0.1807,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(-0.6127,0.0,0.0,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(-0.57155,0.0,0.0,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,M_PI_2,0.17415,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,-M_PI_2,0.11985,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,0.0,0.11655,0.0)));

    fksolver = new KDL::ChainFkSolverPos_recursive(ur10e);
    jsolver = new KDL::ChainJntToJacSolver(ur10e);

    q.resize((ur10e.getNrOfJoints()));

    lock_joints.resize(ur10e.getNrOfJoints());
	for(int i = 0; i < ur10e.getNrOfJoints(); i++){
		lock_joints[i] = false;
	}

    j.resize(6);
	jsolver->setLockedJoints(lock_joints);

    //!SELECT ACCORDING TO WHICH ROBOT IS IN USE
    REAL_ROBOT_ = true;

    //!SELECT ACCORDING TO THE DESIRED GOAL TOLERANCE
    GOAL_TOLERANCE_ = 0.02;

    //! FLAG TO SIGNAL THE ROBOT IS IN A STOP STATE
    ROBOT_STOPPED_ = true;

    //! FLAG TO USE GOALS COMPUTED IN THE JOINT SPACE
    JOINT_SPACE_ = true;

    D = 0.30;

    goal_distance_ = 5.0;
}

RedundancySolverUR::~RedundancySolverUR() {

    // Close the Ostream
    tank_file_.close();
    cbf_file_.close();
    force_file_.close();
    dotq_file_.close();
    dist_file_.close();
    pose_file_.close();
    potential_file_.close();

}

//* Callback for the wrench coming from the Ati Mini 45 F/T sensor
void RedundancySolverUR::forceCallback(const geometry_msgs::WrenchStamped& msg){
    if(REAL_ROBOT_){
    //     F_ext_[0] = msg.wrench.force.x;
    //     F_ext_[1] = msg.wrench.force.y;
    //     F_ext_[2] = msg.wrench.force.z;
    //     F_ext_[3] = msg.wrench.torque.x;
    //     F_ext_[4] = msg.wrench.torque.y;
    //     F_ext_[5] = msg.wrench.torque.z;

    //     for(int i = 0; i < 3; i++){
    //         if((F_ext_[i] > 0.0) && (F_ext_[i] < 4.0))
    //             F_ext_[i] = 0.0;

    //         else if((F_ext_[i] < 0.0) && (F_ext_[i] > - 4.0))
    //             F_ext_[i] = 0.0;
    //     }

    //     for(int i = 3; i < F_ext_.size(); i++){
    //         if((F_ext_[i] > 0.0) && (F_ext_[i] < 5.0))
    //             F_ext_[i] = 0.0;

    //         else if((F_ext_[i] < 0.0) && (F_ext_[i] > - 5.0))
    //             F_ext_[i] = 0.0;
    //     }
        F_ext_[0] = 0.0;
        F_ext_[1] = 0.0;
        F_ext_[2] = 0.0;
        F_ext_[3] = 0.0;
        F_ext_[4] = 0.0;
        F_ext_[5] = 0.0;
    }

    
}

//* Callback for debugging stuff using the joystick
void RedundancySolverUR::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
    
    enable_force_1_ = (bool)msg->buttons[4];
    enable_force_2_ = (bool)msg->buttons[5];

}

//* Callbacks for setting initial values
void RedundancySolverUR::jointPosCb(const sensor_msgs::JointStateConstPtr& msg){
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
        
        for(int i=0; i < 6; i++){
            q(i) = initial_pos_array[i];
            //ROS_INFO_STREAM("JOINT" << i << ": " << initial_pos_array[i]);
        }
        // std::cout << "***********************************************" << std::endl;
    }

}

void RedundancySolverUR::jointPosSimCb(const std_msgs::Float64MultiArrayConstPtr& msg){
    if(!REAL_ROBOT_){
        std_msgs::Float64MultiArray  array_msg = *msg;

        for(int i=0; i < 6; i++){
            initial_pos_array[i] = array_msg.data[i];
            //ROS_INFO_STREAM("JOINT" << i << ": " << initial_pos_array[i]);
        }
    }

}

void RedundancySolverUR::obsCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    if(REAL_ROBOT_){
        geometry_msgs::PoseStamped  pose_msg = *msg;

        obs_pose.x = pose_msg.pose.position.x;
        obs_pose.y = pose_msg.pose.position.y;
        obs_pose.z = pose_msg.pose.position.z;

        obs_pose_stamped = *msg;
    }

}

void RedundancySolverUR::obs2Callback(const geometry_msgs::PoseStampedConstPtr& msg){
    if(REAL_ROBOT_){
        geometry_msgs::PoseStamped  pose_msg = *msg;

        obs2_pose.x = pose_msg.pose.position.x;
        obs2_pose.y = pose_msg.pose.position.y;
        obs2_pose.z = pose_msg.pose.position.z;

        obs2_pose_stamped = *msg;
    }

}

void RedundancySolverUR::obsSimCallback(const geometry_msgs::PointConstPtr& msg){
    if(!REAL_ROBOT_){
        geometry_msgs::Point  point_msg = *msg;

        obs_pose.x = -point_msg.x;
        obs_pose.y = -point_msg.y;
        obs_pose.z = point_msg.z;
    }

}


void RedundancySolverUR::urBaseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
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

bool RedundancySolverUR::setGoalsCallback(redundancy_solver::GetGoalsRequest& req, redundancy_solver::GetGoalsResponse& res){
    goals_vector_ = req.goals;

    geometry_msgs::Pose current_goal = req.goals[0];

    // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(current_goal.orientation, quat);

    // Access roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROBOT_STOPPED_ = false;
    
    goal_pose_rpy_[0] = current_goal.position.x;
    goal_pose_rpy_[1] = current_goal.position.y;
    goal_pose_rpy_[2] = current_goal.position.z;
    goal_pose_rpy_[3] = roll;
    goal_pose_rpy_[4] = pitch;
    goal_pose_rpy_[5] = yaw;

    res.success = true;
    return true;
}

bool RedundancySolverUR::setGoalsJointsCallback(redundancy_solver::GetGoalsJointsRequest& req, redundancy_solver::GetGoalsJointsResponse& res){
    goals_joints_vector_ = req.goals;
    sensor_msgs::JointState current_goal = goals_joints_vector_[0];
    ROS_INFO_STREAM ("Point: " << current_goal.position[0] << "" << current_goal.position[1] << "" << current_goal.position[2] << "" << current_goal.position[3] << "" << current_goal.position[4] << "" << current_goal.position[5]);

    for(int i = 0; i < 6; i++)
        goal_joint_space_[i] = current_goal.position[i];
    
    ROBOT_STOPPED_ = false;

    res.success = true;
    return true;
}

bool RedundancySolverUR::stopRobotCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res){
    //Raise the flag signing that the robot is in stop state
    ROBOT_STOPPED_ = true;

    res.success = true;
    res.message = "Robot stopped!";
}

bool RedundancySolverUR::resumeRobotCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res){
    //Raise the flag signing that the robot is in stop state
    ROBOT_STOPPED_ = false;

    res.success = true;
    res.message = "Robot resumed!";
}

bool RedundancySolverUR::changeSafetyCallback(redundancy_solver::ChangeSafetyDistanceRequest& req, redundancy_solver::ChangeSafetyDistanceResponse& res){
    D = req.distance;

    return true;
}

void RedundancySolverUR::updateGoal(){
    if(!JOINT_SPACE_){
        if((goal_distance_ <= GOAL_TOLERANCE_) && (goals_vector_.size() > 1) && (!ROBOT_STOPPED_)){
            ROS_ERROR_STREAM("GOAL REACHED! ONTO THE NEW ONE!");
            goals_vector_.erase(goals_vector_.begin());

            // Select the new goal from the vector
            geometry_msgs::Pose current_goal = goals_vector_[0];

            // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
            tf::Quaternion quat;
            tf::quaternionMsgToTF(current_goal.orientation, quat);

            // Access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            goal_pose_rpy_[0] = current_goal.position.x;
            goal_pose_rpy_[1] = current_goal.position.y;
            goal_pose_rpy_[2] = current_goal.position.z;
            goal_pose_rpy_[3] = roll;
            goal_pose_rpy_[4] = pitch;
            goal_pose_rpy_[5] = yaw;

            std_srvs::Trigger srv;
            goalReachedClient.call(srv);

        }
        else if((goal_distance_ <= GOAL_TOLERANCE_) && (goals_vector_.size() == 1) && (!ROBOT_STOPPED_)){
            ROS_ERROR_STREAM("FINAL GOAL REACHED! TASK COMPLEATED!");
            ROBOT_STOPPED_ = true;

            std_srvs::Trigger srv;
            taskFinishedClient.call(srv);
            
        }
    }
    else{
        if((goal_distance_ <= GOAL_TOLERANCE_) && (goals_joints_vector_.size() > 1) && (!ROBOT_STOPPED_)){
            ROS_ERROR_STREAM("GOAL REACHED! ONTO THE NEW ONE!");
            for(int i=0; i < 6; i++)
                ROS_INFO_STREAM("JOINT " << i << ": " << initial_pos_array[i]);
            goals_joints_vector_.erase(goals_joints_vector_.begin());

            // Select the new goal from the vector
            sensor_msgs::JointState current_goal = goals_joints_vector_[0];

            for(int i = 0; i < 6; i++)
                goal_joint_space_[i] = current_goal.position[i];
            
            ros::Duration(0.5).sleep();
            
            std_srvs::Trigger srv;
            goalReachedClient.call(srv);

        }
        else if((goal_distance_ <= GOAL_TOLERANCE_) && (goals_joints_vector_.size() == 1) && (!ROBOT_STOPPED_)){
            ROS_ERROR_STREAM("FINAL GOAL REACHED! TASK COMPLEATED!");
            for(int i=0; i < 6; i++)
                ROS_INFO_STREAM("JOINT " << i << ": " << initial_pos_array[i]);
            ROBOT_STOPPED_ = true;

            std_srvs::Trigger srv;
            taskFinishedClient.call(srv);
            
        }
    }
}

//* Spinner

void RedundancySolverUR::spin(){

// ********* INIALIZE MODEL AND READ POSE HERE ********* //

double current_time = ros::Time::now().toSec();

// Instantiating the robot model
robot_model::RobotModelPtr kinematic_model = robot_model_loader->getModel();

// Creating the robot kinematic state and setting it to the current joints positions
robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setVariablePositions(initial_pos_array);
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

// Correct the forces using the reference frame of the tool
const Eigen::Affine3d& end_effector_state_tool = kinematic_state->getGlobalLinkTransform("tool0");

Eigen::Matrix3d ee_pose_angular_tool = end_effector_state_tool.rotation();

Eigen::MatrixXd force_mapping(6,6);
Eigen::MatrixXd force_mapping_inverse(6,6);
force_mapping << ee_pose_angular_tool,
                 Eigen::Matrix3d::Zero(3,3),
                 Eigen::Matrix3d::Zero(3,3),
                ee_pose_angular_tool;    

F_ext_ = force_mapping * F_ext_;

force_mapping_inverse << ee_pose_angular_tool.inverse(),
                 Eigen::Matrix3d::Zero(3,3),
                 Eigen::Matrix3d::Zero(3,3),
                 ee_pose_angular_tool.inverse();

const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");

// Compute the pose of the end-effector with respect to the /base frame
ee_pose_linear = end_effector_state.translation();
Eigen::Matrix3d ee_pose_angular = end_effector_state.rotation();

// Compute RPY by converting through tf
geometry_msgs::TransformStamped rot_msg = tf2::eigenToTransform(end_effector_state);

tf2::Quaternion quat;
tf2::convert(rot_msg.transform.rotation, quat);
tf2::Matrix3x3(quat).getRPY(roll_ee, pitch_ee, yaw_ee);

// ROS_INFO_STREAM("X: " << ee_pose_linear[0]);
// ROS_INFO_STREAM("Y: " << ee_pose_linear[1]);
// ROS_INFO_STREAM("Z: " << ee_pose_linear[2]);
// ROS_INFO_STREAM("R: " << roll_ee);
// ROS_INFO_STREAM("P: " << pitch_ee);
// ROS_INFO_STREAM("Y: " << yaw_ee);

// Update the current goal, given the distance from the previous one
// 

//! Correct the Jacobian for a pose control application
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

jacobian_arm = transform_RPY_arm * jacobian_arm; //!!!!!!!!!!!!!!!!!!!!

//* Uncomment to use the kinematics computed by KDL instead of MoveIt!
/*jsolver->JntToJac(q, j, ur10e.getNrOfJoints());
fksolver->JntToCart(q, ee_pose);

jacobian_arm = j.data;

ee_pose_linear[0] = ee_pose.p.x();
ee_pose_linear[1] = ee_pose.p.y();
ee_pose_linear[2] = ee_pose.p.z();

ee_pose.M.GetRPY(roll_ee, pitch_ee, yaw_ee);

ROS_INFO_STREAM("X KDL: " << ee_pose_linear[0]);
ROS_INFO_STREAM("Y KDL: " << ee_pose_linear[1]);
ROS_INFO_STREAM("Z KDL: " << ee_pose_linear[2]);
ROS_INFO_STREAM("R KDL: " << roll_ee);
ROS_INFO_STREAM("P KDL: " << pitch_ee);
ROS_INFO_STREAM("Y KDL: " << yaw_ee);*/


// **************** TRANSFORMS HERE **************** //
geometry_msgs::PoseStamped ee_pose_msg;
ee_pose_msg.header.frame_id = "ee_link";
ee_pose_msg.header.stamp = ros::Time::now();
ee_pose_msg.pose.position.x = ee_pose_linear[0];
ee_pose_msg.pose.position.y = ee_pose_linear[1];
ee_pose_msg.pose.position.z = ee_pose_linear[2];

//ROS_ERROR_STREAM("TRANSFORM MATRIX: \n" << transform.transform.rotation);

Eigen::Quaterniond q_ee(end_effector_state.rotation());
geometry_msgs::Quaternion q_msg;
q_msg.x = q_ee.x();
q_msg.y = q_ee.y();
q_msg.z = q_ee.z();
q_msg.w = q_ee.w();

ee_pose_msg.pose.orientation = q_msg;

// PUBLISH THE EE POSE WRT ROBOT BASE HERE FOR ATI MINI 45
ee_tip_pub_.publish(ee_pose_msg);


Eigen::Matrix3d frame_correction;
frame_correction <<  0.0, -1.0,  0.0,
                     0.0,  0.0, -1.0,
                     1.0,  0.0,  0.0;

Eigen::MatrixXd jacobian_mapping(6,6);
jacobian_mapping << frame_correction,
                 Eigen::Matrix3d::Zero(3,3),
                 Eigen::Matrix3d::Zero(3,3),
                frame_correction;    

//* Jacobian's Inverse
Eigen::MatrixXd J_inv = jacobian_arm.inverse();

//* CHECK INTERACTION
interaction_ = false;

for(int i = 0; i < F_ext_.size(); i++){
    if(F_ext_[i] != 0.0)
        interaction_ = true;
}

/***************************************************************************************/
/*************************************PENALTY MATRIX************************************/
/***************************************************************************************/
Eigen::VectorXd m(6);
for(int i = 0; i < 6; i++)
    m[i] = 1.0;

// Decomment for using admittance
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

// LOAD THE CURRENT POSE OF THE END-EFFECTOR
if(!JOINT_SPACE_){
    params.sigma[0] = ee_pose_linear[0];
    params.sigma[1] = ee_pose_linear[1];
    params.sigma[2] = ee_pose_linear[2];
    params.sigma[3] = roll_ee;
    params.sigma[4] = pitch_ee;
    params.sigma[5] = yaw_ee;
}
else{
    params.sigma[0] = initial_pos_array[0];
    params.sigma[1] = initial_pos_array[1];
    params.sigma[2] = initial_pos_array[2];
    params.sigma[3] = initial_pos_array[3];
    params.sigma[4] = initial_pos_array[4];
    params.sigma[5] = initial_pos_array[5];
}


// SELECT HERE THE DESIRED FINAL END-EFFECTOR POSITION
if(!JOINT_SPACE_){
    if(!ROBOT_STOPPED_){
        params.sigma_0[0] = goal_pose_rpy_[0];
        params.sigma_0[1] = goal_pose_rpy_[1];
        params.sigma_0[2] = goal_pose_rpy_[2];
        params.sigma_0[3] = goal_pose_rpy_[3];
        params.sigma_0[4] = goal_pose_rpy_[4];
        params.sigma_0[5] = goal_pose_rpy_[5];
    }
    else{
        params.sigma_0[0] = ee_pose_linear[0];
        params.sigma_0[1] = ee_pose_linear[1];
        params.sigma_0[2] = ee_pose_linear[2];
        params.sigma_0[3] = roll_ee;
        params.sigma_0[4] = pitch_ee;
        params.sigma_0[5] = yaw_ee;
    }
}
else{
    if(!ROBOT_STOPPED_){
        params.sigma_0[0] = goal_joint_space_[0];
        params.sigma_0[1] = goal_joint_space_[1];
        params.sigma_0[2] = goal_joint_space_[2];
        params.sigma_0[3] = goal_joint_space_[3];
        params.sigma_0[4] = goal_joint_space_[4];
        params.sigma_0[5] = goal_joint_space_[5];
    }
    else{
        params.sigma_0[0] = initial_pos_array[0];
        params.sigma_0[1] = initial_pos_array[1];
        params.sigma_0[2] = initial_pos_array[2];
        params.sigma_0[3] = initial_pos_array[3];
        params.sigma_0[4] = initial_pos_array[4];
        params.sigma_0[5] = initial_pos_array[5];
    }
}

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

    // geometry_msgs::PoseStamped obs_pose_transformed_msg; 
    // tf2::doTransform(obs_pose_stamped, obs_pose_transformed_msg, transform);

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

    // ROS_WARN_STREAM("OBS X" << transform.transform.translation.x);
    // ROS_WARN_STREAM("OBS Y" << transform.transform.translation.y);
    // ROS_WARN_STREAM("OBS Z" << transform.transform.translation.z);

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

//* Safety CBF for first obstacle
params.x[0] = ee_pose_linear[0];
params.x[1] = ee_pose_linear[1];
params.x[2] = ee_pose_linear[2];
params.x[3] = roll_ee;
params.x[4] = pitch_ee;
params.x[5] = yaw_ee;

double d_2 = 0.0;

for(int i= 0; i< 3; i++){
   d_2 += pow(ee_pose_linear[i] - params.sigma_obs[i] , 2);
}

double d = sqrt(d_2);
double gamma_safe = 50.0; // was 50.0

params.h_safe[0] =  - gamma_safe * (d_2 - pow(D,2));

if(params.h_safe[0] > 0.0){

    ROS_INFO_STREAM("STOP RIGHT THERE CRIMINAL SCUM!");
    ROS_INFO_STREAM("d: " << d);

    //std_srvs::Trigger warn_srv;
    //warningMotionClient.call(warn_srv);
    std_msgs::Bool warn_msg;
    warn_msg.data = true;
    safety_pub_.publish(warn_msg);
}

//* Safety CBF for second obstacle
double d_tb2 = 0.0;

for(int i= 0; i< 3; i++){
   d_tb2 += pow(ee_pose_linear[i] - params.sigma_obs2[i] , 2);
}

double d_tb = sqrt(d_tb2);

params.h_safe_2[0] =  - gamma_safe * (d_tb2 - pow(D,2));
// ROS_INFO_STREAM("d tb: " << d_tb);

if(params.h_safe_2[0] > 0.0){

    ROS_INFO_STREAM("STOP RIGHT THERE CRIMINAL TURTLEBOT!");
    ROS_INFO_STREAM("d tb: " << d_tb);

    //std_srvs::Trigger warn_srv;
    //warningMotionClient.call(warn_srv);
    std_msgs::Bool warn_msg;
    warn_msg.data = true;
    safety_pub_.publish(warn_msg);
}

// ROS_WARN_STREAM("d: " << d);
// ROS_INFO_STREAM("Safety CBF: " << params.h_safe[0]);

/***************************************************************************************/
/*********************************ADMITTANCE CONTROLLER*********************************/
/***************************************************************************************/
Eigen::VectorXd adm_desired_accelaration(6);
Eigen::VectorXd des_twist_adm(6);

//* Increase gradually the M and D matrix linearly with the distance from the obstacle
double D_stiff = 0.5; //distance after which the admittance gets stiffer
double K_adm = - (d - D_stiff)/(D_stiff - D);

if(K_adm <= 0.0)
    K_adm = 0.0;

double K_repulsive = 10.0;
Eigen::VectorXd F_rep(6);

if(REAL_ROBOT_){
    // F_rep[0] = - K_adm * 1 / (ee_pose_linear[0] - transform.transform.translation.x);
    // F_rep[1] = - K_adm * 1 / (ee_pose_linear[1] - transform.transform.translation.y);
    // F_rep[2] = - K_adm * 1 / (ee_pose_linear[2] - transform.transform.translation.z);
    F_rep[3] = 0.0;
    F_rep[4] = 0.0;
    F_rep[5] = 0.0;
}

for(int i = 0; i < 3; i++){
    if(F_rep[i] >= 15.0)
        F_rep[i] = 15.0;

    if(F_rep[i] <= -15.0)
        F_rep[i] = -15.0;
}

// ROS_ERROR_STREAM("REPULSIVE FORCE X : " << F_rep[0]);
// ROS_ERROR_STREAM("REPULSIVE FORCE Y : " << F_rep[1]);
// ROS_ERROR_STREAM("REPULSIVE FORCE Z : " << F_rep[2]);

// Apply the force and compute the desired admittance //! RICORDA DI INSERIRE IL VALORE DI RITORNO DAL TOPIC DI VELOCITA
Eigen::VectorXd dotq_data(6);
for(int i = 0; i < 6; i++)
    dotq_data[i] = initial_vel_array[i];

desired_twist_ = jacobian_arm * dotq_data;
adm_desired_accelaration = M_tot_.inverse() * ( - D_tot_ * desired_twist_ + F_ext_ - F_rep);

for(int i = 0; i < adm_desired_accelaration.size(); i++){
    if(F_ext_[i] == 0.0)
        adm_desired_accelaration[i] = 0.0;
}

for(int i = 0; i < adm_desired_accelaration.size(); i++){
    if(F_ext_[i] != 0.0)
        des_twist_adm[i] = desired_twist_[i] + adm_desired_accelaration[i] * cycle_time;
    else
        des_twist_adm[i] = 0.0;
}

// ROS_WARN_STREAM("X ADM PRE OPT: " << des_twist_adm[0]);
// ROS_WARN_STREAM("Y ADM PRE OPT: " << des_twist_adm[1]);
// ROS_WARN_STREAM("Z ADM PRE OPT: " << des_twist_adm[2]);

Eigen::VectorXd dotq_adm(6);
dotq_adm = J_inv * des_twist_adm;

// INSERT THE DESIRED TWIST FROM ADMITTANCE CONTROL
for(int i = 0; i< 6; i++)
    params.dotq_adm[i] = 0.0; //! Disabled admittance control for this experiment

/***************************************************************************************/
/**************************************ENERGY TANK**************************************/
/***************************************************************************************/
Eigen::VectorXd A = -cycle_time * (F_ext_);
for(int i = 0; i < 6; i++)
    //params.A[i] = A[i];
    //params.A[i] = 0.0;
double B = - TANK_MIN_VALUE - sum_of_delta_ + tank_energy_;
// params.B[0] = B;
//params.B[0] = 50;

/***************************************************************************************/
/*************************************EXECUTION CBFs************************************/
/***************************************************************************************/

// DEFINE HERE THE CBF-RELATED FUNCTION h1 = - 1/2 * square(norm(sigma-sigma_0)) FOR TASK COMPLETION
double quad_norm = 0.0;
double gamma_goal = 10.0; //was 5.0

if(!JOINT_SPACE_){
    // Compute linear distance
    for(int i=0; i<3; i++)
        quad_norm += pow(params.sigma[i] - params.sigma_0[i], 2);

    // Compute angular distance
    for(int i=3; i<6; i++){
        quad_norm += pow(atan2(sin(params.sigma_0[i] - params.sigma[i]), cos(params.sigma_0[i] - params.sigma[i])),2);
    }

    goal_distance_ = sqrt(quad_norm); // distance from the goal
    ROS_WARN_STREAM("Goal distance: " << goal_distance_);
}
else{
    // Compute angular distance in the joint space
    for(int i=0; i<6; i++)
        quad_norm += pow(params.sigma[i] - params.sigma_0[i], 2);
    
    goal_distance_ = sqrt(quad_norm);
}

// Parameters to activate or deactivate the slack variables' influence
params.theta[0] = 1.0;
params.theta[1] = 1.0;
params.theta[2] = 1.0;

params.h_goal[0] = - gamma_goal * quad_norm;

double gamma_thresh = 0.4;
if (params.h_goal[0] <= -gamma_thresh)  // was 0.1
    params.h_goal[0] = -gamma_thresh;

if(params.h_goal[0] < -gamma_thresh)
    params.theta[0] = 0.0;

ROS_INFO_STREAM_THROTTLE(5,"Goal CBF: " << params.h_goal[0]);

// DEFINE THE CBF-RELATED FUNCTION h_lim TO AVOID JOINT LIMITS
double h2 = 0.0;
double h_lim_array[6];

for(int i=0; i<6; i++){
    params.Q_lim[i] = ( (joint_limits[i] - 2 * initial_pos_array[i] + (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
}

for(int i=0; i<6; i++){
    params.h_lim[i] =  -0.5 * ( (joint_limits[i] - initial_pos_array[i]) * (initial_pos_array[i] - (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
    h_lim_array[i] = -0.5 * ( (joint_limits[i] - initial_pos_array[i]) * (initial_pos_array[i] - (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
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

// HANDLE THE PRIORITIZATION OF TASKS AND SLACK VARIABLES
params.l[0] = 60.0; //was 60.0 //!

// IMPOSING SPEED AND ACCELERATION LIMITS
params.dotq_max[0] =  0.25;
params.dotq_min[0] = -0.25;

for(int i = 0; i< 6; i++)
    params.a_max[i] = 0.5 * cycle_time; // was 0.25

for(int i=0; i<6 ; i++)
    params.dotq_prev[i] = dotq_prev_[i];

/***************************************************************************************/
/*******************PUBLISH THE SOLUTION OF THE OPTIMIZATION PROBLEM********************/
/***************************************************************************************/
settings.verbose = 0;
settings.max_iters = 50;
// settings.eps = 1e-11;
// settings.resid_tol = 1e-9;
// settings.refine_steps = 5;
// settings.kkt_reg = 1e-5;
long num_iters = solve();

for(int i = 0; i < 6; i++){
    if(isnan(vars.dotq[i])){
        vars.dotq[i] = 0.0;
    ROS_ERROR_STREAM("OUTPUT NON VALIDO AIUTO");
    }
}

for(int i = 0; i < 2; i++){
    if(isnan(vars.delta[i]))
        vars.delta[i] = 0.0;
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

//!!!!!!!!!
// for(int i = 0; i < 6; i ++){
//     if(q_dot[i] >= 0.25)
//         q_dot[i]=0.25;
//     else if(q_dot[i] <= -0.25)
//         q_dot[i]=-0.25;
// }

for(int i = 0; i<6; i++){
    //ROS_INFO_STREAM("DOTQ" << i  <<": " << vars.dotq[i]);
    if((vars.dotq[i] > 1.58) || (vars.dotq[i] < -1.58))
        ROS_ERROR_STREAM("VAI TROPPO VELOCE OHHH CALMA!!!!!!!");
}

x_dot = jacobian_arm * q_dot; //! RICORDA DI INSERIRE IL VALORE DI RITORNO DAL TOPIC DI VELOCITA

// ROS_WARN_STREAM("X ADM POST OPT: " << x_dot[0]);
// ROS_WARN_STREAM("Y ADM POST OPT: " << x_dot[1]);
// ROS_WARN_STREAM("Z ADM POST OPT: " << x_dot[2]);

Eigen::VectorXd modulation_matrix = x_dot / tank_state_;  //! VERIFY IF FUNZIONA

double delta_update = (pow(cycle_time, 2)/ (2 * pow(tank_state_, 2)))*(pow((F_ext_).transpose() * x_dot, 2));
// Uncomment when using variable admittance controller
sum_of_delta_ += delta_update;
tank_energy_ += cycle_time * (F_ext_).transpose() * x_dot + delta_update;
tank_state_ += cycle_time * modulation_matrix.transpose() * (F_ext_);

std_msgs::Float64 tank_msg;
tank_msg.data = tank_energy_;

tank_pub_.publish(tank_msg);

if(tank_state_ >= 2 * sqrt(TANK_MAX_VALUE))
    tank_state_ = 2 * sqrt(TANK_MAX_VALUE);

if(tank_state_ <= -2 * sqrt(TANK_MAX_VALUE))
    tank_state_ = -2 * sqrt(TANK_MAX_VALUE);

if(tank_energy_ >= TANK_MAX_VALUE)
    tank_energy_ = TANK_MAX_VALUE;

if(tank_state_ <= 0.0)
    ROS_ERROR_STREAM("TANK EMPTY!");

// ROS_INFO_STREAM("TANK STATE: " << tank_state_);
// ROS_INFO_STREAM("TANK ENERGY: " << tank_energy_);

// Output to files
tank_file_ << ros::Time::now().toSec() - start_time_;
cbf_file_ << ros::Time::now().toSec() - start_time_;
force_file_ << ros::Time::now().toSec() - start_time_;
dotq_file_ << ros::Time::now().toSec() - start_time_;
dist_file_ << ros::Time::now().toSec() - start_time_;
pose_file_ << ros::Time::now().toSec() - start_time_;
potential_file_ << ros::Time::now().toSec() - start_time_;
tank_file_ << " " << tank_state_ << " " << tank_energy_;
cbf_file_ << " " << gamma_goal * quad_norm << " " << gamma_safe * (d_2 - pow(D,2));

for(int i = 0; i < 6; i++){
    cbf_file_ << " " << h_lim_array[i];
}

for(int i = 0; i < 6; i++){
    force_file_ << " " << F_ext_[i];
}

for(int i = 0; i < 6; i++){
    dotq_file_ << " " << q_dot[i];
}

for(int i = 0; i < 3; i++){
    potential_file_ << " " << F_rep[i];
}

dist_file_ << " " << d << std::endl;

pose_file_ << " " << ee_pose_linear[0] << " " << ee_pose_linear[1] << " " << ee_pose_linear[2];
tank_file_ << std::endl;
cbf_file_ << std::endl;
force_file_ << std::endl;
dotq_file_ << std::endl;
dist_file_ << std::endl;
pose_file_ << std::endl;
potential_file_ << std::endl;

cycle_time = 0.002;

// Fixing the overtime due to the first cycle being slower
if(first_cycle_)
    first_cycle_ = false; 

// std::cout << "Node cycle time: " << ros::Time::now().toSec() - current_time << std::endl;

//* Create the messages and publishing them
trajectory_msgs::JointTrajectory arm_pose;
arm_pose.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
arm_pose.points.resize(1);
arm_pose.points[0].time_from_start = ros::Duration(0.001);

std_msgs::Float64MultiArray msg;

for(int i=0; i< dotq_vec.size(); i++)
    dotq_vec[i] = q_dot[i];

msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
msg.layout.dim[0].size = dotq_vec.size();
msg.layout.dim[0].stride = 1;
msg.layout.dim[0].label = "vel";

// copy in the data
msg.data.clear();
msg.data.insert(msg.data.end(), dotq_vec.begin(), dotq_vec.end());

arm_pose_pub_.publish(msg);
double sleep_deprivation = ros::Time::now().toSec() - current_time;

updateGoal();

// if(sleep_deprivation < 0.002)
//     ros::Duration(0.002 - (ros::Time::now().toSec() - current_time)).sleep();

//! Uncomment for using a position controller
/*double delta_q[6] = {q_dot[0] * cycle_time, 
                     q_dot[1] * cycle_time, 
                     q_dot[2] * cycle_time,
                     q_dot[3] * cycle_time,
                     q_dot[4] * cycle_time,
                     q_dot[5] * cycle_time} ;

arm_pose.points[0].positions = {initial_pos_array[0] + delta_q[0],
                                initial_pos_array[1] + delta_q[1],
                                initial_pos_array[2] + delta_q[2],
                                initial_pos_array[3] + delta_q[3],
                                initial_pos_array[4] + delta_q[4],
                                initial_pos_array[5] + delta_q[5]};

arm_pose_pub_.publish(arm_pose);*/



/**********************************************/
/******************** DEBUG *******************/
/**********************************************/


}





