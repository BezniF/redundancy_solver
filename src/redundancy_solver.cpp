#include "redundancy_solver.h"
#include "solver.h"
#include "solver.c"
#include "ldl.c"
#include "matrix_support.c"
#include "util.c"

RedundancySolver::RedundancySolver() {

    joints_pos_sub_= nh_.subscribe("/prbt/manipulator_joint_trajectory_controller/state", 1, &RedundancySolver::jointPosCb, this);
    base_pose_sub_=nh_.subscribe("/odom", 1 ,&RedundancySolver::basePosCb, this);
    joy_sub_ = nh_.subscribe("/joy", 1, &RedundancySolver::joyCallback, this);
    wrench_sub_ = nh_.subscribe("wrench", 1, &RedundancySolver::forceCallback, this);
    // TODO: subscriber to the Omega controller for a speed command

    base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //arm_pose_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Single_Point_Trajectory", 1000);
    arm_pose_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/prbt/manipulator_joint_trajectory_controller/command", 1);
    ee_tip_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_ee_tip", 1);
    //ee_tip_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    first_cycle_ = true;

    joint_limits.resize(6);
    joint_limits = {2.967, 2.095, 2.356, 2.967, 2.966, 3.123};

    cycle_time = 0.02;

    // Admittance parameters init
    std::vector<double> M_p;
    std::vector<double> M_a;
    std::vector<double> D_p;
    std::vector<double> D_a;
    std::vector<double> des_twist_init (6, 0.0);

    if (!nh_.getParam("mass_platform", M_p)) {
        ROS_ERROR("Couldn't retrieve the desired mass platform.");
    }

    if (!nh_.getParam("mass_arm", M_a)) {
        ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    }

    if (!nh_.getParam("damping_platform", D_p)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the platform.");
    }

    if (!nh_.getParam("damping_arm", D_a)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    }

    M_p_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_p.data());
    M_a_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_a.data());
    D_p_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_p.data());
    D_a_ = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_a.data());

    M_tot_ = M_p_ + M_a_;
    D_tot_ = D_p_ + D_a_;
        
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
    //robot_model_loader = new robot_model_loader::RobotModelLoader ("robot_description");

    // Output to file stream
    std::stringstream file_path;
    file_path << "/home/federico/benzi_ws/config/simulations/redundancy_solver/tank.txt";
    tank_file_.open(file_path.str());

    std::stringstream file_path2;
    file_path2 << "/home/federico/benzi_ws/config/simulations/redundancy_solver/cbf.txt";
    cbf_file_.open(file_path2.str());

    std::stringstream file_path3;
    file_path3 << "/home/federico/benzi_ws/config/simulations/redundancy_solver/force.txt";
    force_file_.open(file_path3.str());

    start_time_ = ros::Time::now().toSec();

    interaction_ = false;

}

RedundancySolver::~RedundancySolver() {

    tank_file_.close();
    cbf_file_.close();
    force_file_.close();
}

//* Callback for the wrench coming from the Ati Mini 45 F/T sensor
void RedundancySolver::forceCallback(const geometry_msgs::WrenchStamped& msg){

    F_ext_[0] = msg.wrench.force.x;
    F_ext_[1] = msg.wrench.force.y;
    F_ext_[2] = msg.wrench.force.z;
    // F_ext_[3] = msg.wrench.torque.x;
    // F_ext_[4] = msg.wrench.torque.y;
    // F_ext_[5] = msg.wrench.torque.z;
    // F_ext_[1] = 0.0;
    // F_ext_[2] = 0.0;
    F_ext_[3] = 0.0;
    F_ext_[4] = 0.0;
    F_ext_[5] = 0.0;

    for(int i = 0; i < F_ext_.size(); i++){
        ROS_ERROR_STREAM("FORCE " << i << ": " << F_ext_[i]);
    }

    for(int i = 0; i < F_ext_.size(); i++){
        if((F_ext_[i] > 0.0) && (F_ext_[i] < 20.0))
            F_ext_[i] = 0.0;

        else if((F_ext_[i] < 0.0) && (F_ext_[i] > -20.0))
            F_ext_[i] = 0.0;
    }
    

}

//* Callback for debugging stuff using the joystick
void RedundancySolver::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    enable_force_1_ = (bool)joy->buttons[4];
    enable_force_2_ = (bool)joy->buttons[5];

}

//* Callbacks for setting initial values
void RedundancySolver::jointPosCb(const control_msgs::JointTrajectoryControllerStateConstPtr& msg){

    initial_pos_joints_ =  *msg;

    for(int i=0; i < 6; i++){
        initial_pos_array[i] = initial_pos_joints_.actual.positions[i];
        initial_vel_array[i] = initial_pos_joints_.actual.velocities[i];
        
    }

}


void RedundancySolver::basePosCb(const nav_msgs::OdometryConstPtr& msg){

    initial_base_odom_ = *msg;
    initial_base_pose_ = initial_base_odom_.pose.pose;
    initial_vel_base_= initial_base_odom_.twist.twist;
    
}

//* Spinner

void RedundancySolver::spin(){

// ********* INIALIZE MODEL AND READ POSE HERE ********* //

// double current_time = ros::Time::now().toSec();

// // Instantiating the robot model
// robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); //!
// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

// // Creating the robot kinematic state and setting it to the current joints positions
// robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
// kinematic_state->setVariablePositions(initial_pos_array);
// const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
// const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

// // Pasting the kinematic model to the Joint Groups in MoveIt!
// std::vector<double> joint_values;
// kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

// kinematic_state->enforceBounds();


// // Computing the Jacobian of the arm
// Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
// Eigen::MatrixXd jacobian_arm;
// kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//                              reference_point_position,
//                              jacobian_arm);

// // Computing the actual position of the end-effector using the arm Jacobian
// const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("prbt_flange");

// // Compute the pose of the end-effector with respect to the /prbt_base frame
// Eigen::Vector3d ee_pose_linear = end_effector_state.translation();
// Eigen::Matrix3d ee_pose_angular = end_effector_state.rotation();

// // Computing current RPY of the EE
// Eigen::Vector3d euler_angles_ee = ee_pose_angular.eulerAngles(2, 1, 0);
// double roll_ee_b = euler_angles_ee[2];
// double pitch_ee_b = euler_angles_ee[1];
// double yaw_ee_b = euler_angles_ee[0];

// // **************** TF TRANSFORMS HERE **************** //

// // Apply the correct tf transforms and store the datas for later usage (world is the prbt's base frame)
// geometry_msgs::TransformStamped transform = tfBuffer->lookupTransform("odom", "world",
//                                ros::Time(0), ros::Duration(3.0));

// /*geometry_msgs::TransformStamped transform = tfBuffer->lookupTransform("world", "world",
//                                ros::Time(0), ros::Duration(3.0));*/

// geometry_msgs::PoseStamped ee_pose_msg;
// ee_pose_msg.header.frame_id = "prbt_flange";
// ee_pose_msg.header.stamp = ros::Time::now();
// ee_pose_msg.pose.position.x = ee_pose_linear[0];
// ee_pose_msg.pose.position.y = ee_pose_linear[1];
// ee_pose_msg.pose.position.z = ee_pose_linear[2];

// Eigen::Quaterniond q_ee(end_effector_state.rotation());
// geometry_msgs::Quaternion q_msg;
// q_msg.x = q_ee.x();
// q_msg.y = q_ee.y();
// q_msg.z = q_ee.z();
// q_msg.w = q_ee.w();

// ee_pose_msg.pose.orientation = q_msg;

// geometry_msgs::PoseStamped ee_pose_transformed_msg; 

// tf2::doTransform(ee_pose_msg, ee_pose_transformed_msg, transform);

// // PUBLISH THE EE POSE WRT ROBOT BASE HERE FOR ATI MINI 45
// geometry_msgs::PoseStamped ee_pose_ati_mini_msg = ee_pose_msg;

// // ee_pose_ati_mini_msg.pose.position.x = - ee_pose_ati_mini_msg.pose.position.x;
// // ee_pose_ati_mini_msg.pose.position.y = - ee_pose_ati_mini_msg.pose.position.y;
// // ee_pose_ati_mini_msg.pose.orientation.z = - ee_pose_ati_mini_msg.pose.orientation.z;

// ee_tip_pub_.publish(ee_pose_ati_mini_msg);

// Eigen::Quaterniond q_ee_final;
// q_ee_final.x() = ee_pose_transformed_msg.pose.orientation.x;
// q_ee_final.y() = ee_pose_transformed_msg.pose.orientation.y;
// q_ee_final.z() = ee_pose_transformed_msg.pose.orientation.z;
// q_ee_final.w() = ee_pose_transformed_msg.pose.orientation.w;

// // Manual conversion from Quaternion to RPY (Eigen fucks this up)
// double sinr_cosp = 2 * (q_ee_final.w() * q_ee_final.x() + q_ee_final.y() * q_ee_final.z());
// double cosr_cosp = 1 - 2 * (q_ee_final.x() * q_ee_final.x() + q_ee_final.y() * q_ee_final.y());
// double ee_final_roll = std::atan2(sinr_cosp, cosr_cosp);

// double ee_final_pitch;
// double sinp = 2 * (q_ee_final.w() * q_ee_final.y() - q_ee_final.z() * q_ee_final.x());
//     if (std::abs(sinp) >= 1)
//         ee_final_pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//     else
//         ee_final_pitch = std::asin(sinp);

// double ee_final_yaw;
// double siny_cosp = 2 * (q_ee_final.w() * q_ee_final.z() + q_ee_final.x() * q_ee_final.y());
//     double cosy_cosp = 1 - 2 * (q_ee_final.y() * q_ee_final.y() + q_ee_final.z() * q_ee_final.z());
//     ee_final_yaw = std::atan2(siny_cosp, cosy_cosp);

// ROS_INFO_STREAM("X : " << ee_pose_transformed_msg.pose.position.x);
// ROS_INFO_STREAM("Y : " << ee_pose_transformed_msg.pose.position.y);
// ROS_INFO_STREAM("Z : " << ee_pose_transformed_msg.pose.position.z);
// ROS_INFO_STREAM("R : " << ee_final_roll);
// ROS_INFO_STREAM("P : " << ee_final_pitch);
// ROS_INFO_STREAM("Y : " << ee_final_yaw);

// // ************* MOBILE BASE HERE ************** //

// // Jacobian of the base is just the Identity6 Matrix since the MPO500 is omnidirectional
// Eigen::MatrixXd jacobian_base = Eigen::MatrixXd::Identity(6,6);

// // Computing the base's pose
// double x_base = initial_base_pose_.position.x;
// double y_base = initial_base_pose_.position.y;
// double z_base = initial_base_pose_.position.z;
// double q_x = initial_base_pose_.orientation.x;
// double q_y = initial_base_pose_.orientation.y;
// double q_z = initial_base_pose_.orientation.z;
// double q_w = initial_base_pose_.orientation.w;

// // From quaterion to RPY of mobile base
// Eigen::Quaterniond q;
// q.x() = q_x;
// q.y() = q_y;
// q.z() = q_z;
// q.w() = q_w;

// Eigen::Vector3d euler_base = q.toRotationMatrix().eulerAngles(2, 1, 0);
// double roll_base = euler_base[2];
// double pitch_base = euler_base[1];
// double yaw_base = euler_base[0];

// // ********** TRANSFORM MATRICES HERE ********** //

// Eigen::Affine3d tfToEigen = tf2::transformToEigen(transform);
// Eigen::MatrixXd transformToOdom = tfToEigen.matrix();

// // Extrapolate the transformation matrix from EE to mobile base (not needed anymore?)
// Eigen::MatrixXd T_ee_0 = end_effector_state.matrix();

// // Transform from manipulator's frame to mobile base's frame
// Eigen::MatrixXd T_0_b(4,4);
//                 T_0_b <<  -1.0, 0.0, 0.0,  0.2,
//                            0.0,-1.0, 0.0,  0.0,
//                            0.0, 0.0, 1.0,-0.39,
//                            0.0, 0.0, 0.0,  1.0;

// // Transform from manipulator's frame to /odom frame (namely the absolute reference system)
// Eigen::MatrixXd R_0_w = transformToOdom;
// R_0_w.conservativeResize(3,3);

// Eigen::MatrixXd Transform_ee_odom(6,6); //!
// /*Transform_ee_odom <<  R_0_w,
//                       Eigen::Matrix3d::Zero(3,3),
//                       Eigen::Matrix3d::Zero(3,3),
//                       R_0_w;*/

// // Tranform the current pose of the EE into the base's frame for insertion in matrix H (ee_mapping matrix)
// Eigen::Vector4d ee_pose_b(ee_pose_linear[0], ee_pose_linear[1], ee_pose_linear[2], 1.0);
// Eigen::Vector4d ee_pose_base;
// ee_pose_base = T_0_b * ee_pose_b;

// // Adapt ee pose in base's frame according to its position on the mobile base
// double x_ee_b = ee_pose_base[0];
// double y_ee_b = ee_pose_base[1];

// // Computing the mapping matrices
// Eigen::MatrixXd ee_mapping_matrix(6,6);

// /*ee_mapping_matrix << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,//-y_ee_b,
//                      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,//x_ee_b,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0;//1.0;*/

// ee_mapping_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

// // Computing the transformation matrix for RPY (are these even needed anymore?)
// double R = ee_final_roll;
// double P = ee_final_pitch;
// double Y = ee_final_yaw;

// Eigen::MatrixXd transform_RPY(6,6);
// transform_RPY <<Eigen::Matrix3d::Identity(3,3), 
//                 Eigen::Matrix3d::Zero(3,3),
//                 Eigen::Matrix3d::Zero(3,3),
//                 (Eigen::Matrix3d() << cos(Y)*cos(P),  -sin(Y), 0, 
//                                       sin(Y)*cos(P),   cos(Y), 0,
//                                      -sin(P)       ,        0, 1).finished();
     
// R = roll_ee_b;
// P = pitch_ee_b;
// Y = yaw_ee_b;

// Eigen::MatrixXd transform_RPY_arm(6,6);
// transform_RPY_arm <<Eigen::Matrix3d::Identity(3,3), 
//                 Eigen::Matrix3d::Zero(3,3),
//                 Eigen::Matrix3d::Zero(3,3),
//                 (Eigen::Matrix3d() << cos(Y)*cos(P),  -sin(Y), 0, 
//                                       sin(Y)*cos(P),   cos(Y), 0,
//                                      -sin(P)       ,        0, 1).finished();

// // Apply the transformation matrices and compute the augmented Jacobian and its pseudoinverse
// //* REMOVE LATER * //
// Transform_ee_odom << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//                       0.0,-1.0, 0.0, 0.0, 0.0, 0.0,
//                       0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
//                       0.0, 0.0, 0.0,-1.0, 0.0, 0.0,
//                       0.0, 0.0, 0.0, 0.0,-1.0, 0.0,
//                       0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

// jacobian_arm = Transform_ee_odom * jacobian_arm; //!
// Eigen::MatrixXd jacobian_base_mapped = ee_mapping_matrix * jacobian_base;

// Eigen::MatrixXd jacobian_augmented(6, 12);
// jacobian_augmented << jacobian_base_mapped, jacobian_arm;
// Eigen::MatrixXd J_pinv = jacobian_augmented.completeOrthogonalDecomposition().pseudoInverse();


// /***************************************************************************************/
// /*********************************ADMITTANCE CONTROLLER*********************************/
// /***************************************************************************************/
// Eigen::VectorXd adm_desired_accelaration(6);
// Eigen::VectorXd des_twist_adm(6);

// // Debug forces
// /*if(enable_force_1_)
//     F_ext_[0] = -100.0;
// if(enable_force_2_)
//     F_ext_[0] = 100.0;
// if((!enable_force_2_) && (!enable_force_1_))
//     F_ext_[0] = 0.0;
// */

// //* Increase gradually the M and D matrix to test the preservation of passivity
// /*if(F_ext_[0] != 0.0){
//     for(int j = 0; j < M_tot_.cols(); j++){
//         for(int i=0; i< M_tot_.rows(); i++){
//             if(M_tot_(i,j) !=0.0)
//                 M_tot_(i,j) += 0.175;
//         }

//     }
//     for(int j = 0; j < D_tot_.cols(); j++){
//         for(int i=0; i< D_tot_.rows(); i++){
//             if(D_tot_(i,j) !=0.0)
//                 D_tot_(i,j) += 0.05;
//         }

//     }
// }*/

// //* Apply the force and compute the desired admittance
// adm_desired_accelaration = M_tot_.inverse() * ( - D_tot_ * desired_twist_ + F_ext_);
// // adm_desired_accelaration = M_tot_.inverse() * ( F_ext_);

// for(int i = 0; i < adm_desired_accelaration.size(); i++){
//     if(F_ext_[i] == 0.0)
//         adm_desired_accelaration[i] = 0.0;
// }

// // ! ADMITTANCE COMPUTATIONS HERE
// double max_acc = 0.5;
// for(int i = 0; i < adm_desired_accelaration.size(); i++){
//     if(adm_desired_accelaration[i] >= max_acc){
//         adm_desired_accelaration[i] = max_acc;
//     }
//     else if(adm_desired_accelaration[i] <= -max_acc){
//         adm_desired_accelaration[i] = -max_acc;
//     }
// }

// for(int i = 0; i < adm_desired_accelaration.size(); i++){
//     if(F_ext_[i] != 0.0)
//         des_twist_adm[i] = desired_twist_[i] + adm_desired_accelaration[i] * cycle_time;
//         // des_twist_adm[i] = 0.2 + adm_desired_accelaration[i] * cycle_time;
//         //des_twist_adm[i] = 0.2;
//     else
//         des_twist_adm[i] = 0.0;
// }

// /***************************************************************************************/
// /*************************************PENALTY MATRIX************************************/
// /***************************************************************************************/
// Eigen::VectorXd m(6);
// for(int i = 0; i < 6; i++)
//     m[i] = 1.0;

// for(int i= 0; i < 6; i++){
//     if(F_ext_[i] != 0.0){
//         //m[i] = fabs(5000.0 * F_ext_[i]);
//         m[i] = 500000.0;
//     }
// }

// for(int i = 0; i < 6; i++)
//     params.M[i] = m[i];

// /***************************************************************************************/
// /**************************************ENERGY TANK**************************************/
// /***************************************************************************************/
// Eigen::VectorXd A = -cycle_time * F_ext_;
// for(int i = 0; i < 6; i++)
//     params.A[i] = A[i];
// double B = - TANK_MIN_VALUE - sum_of_delta_ + tank_energy_;
// //double B = - TANK_MIN_VALUE + tank_energy_;
// params.B[0] = B;

// /***************************************************************************************/
// /**********************LOADING THE DATA FOR THE OPTIMIZATION PROBLEM********************/
// /***************************************************************************************/
// set_defaults();  // Set basic algorithm parameters.
// setup_indexing();

// if(first_cycle_){
//     start_ = ros::Time::now().toSec(); 
// }

// double T = ros::Time::now().toSec() - start_;

// // INSERT THE DESIRED TWIST FROM ADMITTANCE CONTROL
// for(int i = 0; i< 6; i++)
//     //params.dotx_adm[i] = 0.0;
//     params.dotx_adm[i] = des_twist_adm[i];

// velocity_commands.resize(6, 0.0);
// position_commands.resize(6, 0.0);

// // LOAD THE CURRENT POSE OF THE END-EFFECTOR
// params.sigma[0] = ee_pose_transformed_msg.pose.position.x;
// params.sigma[1] = ee_pose_transformed_msg.pose.position.y;
// params.sigma[2] = ee_pose_transformed_msg.pose.position.z;
// params.sigma[3] = ee_final_roll;
// params.sigma[4] = ee_final_pitch;
// params.sigma[5] = ee_final_yaw;

// // SELECT HERE THE DESIRED FINAL END-EFFECTOR POSITION
// // params.sigma_0[0] = 6.0;
// // params.sigma_0[1] = 0.0;
// // params.sigma_0[2] = 1.0;
// // params.sigma_0[3] = 0.0;
// // params.sigma_0[4] = 0.0;
// // params.sigma_0[5] = 0.2;

// params.sigma_0[0] = 0.6;
// params.sigma_0[1] = 0.0;
// params.sigma_0[2] = 1.0;
// params.sigma_0[3] = 0.0;
// params.sigma_0[4] = 0.0;
// params.sigma_0[5] = -1.0;

// // params.sigma_0[0] = ee_pose_transformed_msg.pose.position.x;
// // params.sigma_0[1] = ee_pose_transformed_msg.pose.position.y;
// // params.sigma_0[2] = ee_pose_transformed_msg.pose.position.z;
// // params.sigma_0[3] = ee_final_roll;
// // params.sigma_0[4] = ee_final_pitch;
// // params.sigma_0[5] = ee_final_yaw;

// // SELECT HERE THE DESIRED FINAL END-EFFECTOR VELOCITY
// params.Sigma[0] = 0.0;
// params.Sigma[1] = 0.0;
// params.Sigma[2] = 0.0;
// params.Sigma[3] = 0.0;
// params.Sigma[4] = 0.0;
// params.Sigma[5] = 0.0;

// // SELECT HERE THE POSE OF THE OBSTACLE AND THE DESIRED DISTANCE TO BE KEPT //!
// /*params.sigma_obs[0]= 3.0;
// params.sigma_obs[1]= 0.0;
// params.sigma_obs[2]= 1.0;
// params.sigma_obs[3]= ee_final_roll;
// params.sigma_obs[4]= ee_final_pitch;
// params.sigma_obs[5]= ee_final_yaw;

// double D = 1.0;
// double d_2 = 0.0;

// for(int i= 0; i< 3; i++){
//     d_2 += pow(params.sigma[i] - params.sigma_obs[i] , 2);
// }

// double d = sqrt(d_2);
// double gamma_safe = 2.0; // was 5.0
// //ROS_INFO_STREAM("Distance: " << d);

// params.h_safe[0] =  - gamma_safe * pow((d_2 - pow(D,2)), 3); //? minus sign required


// if(params.h_safe[0] > 0.0){

//     ROS_ERROR_STREAM("STOP RIGHT THERE CRIMINAL SCUM!");
//     ROS_ERROR_STREAM("d: " << d);
// }

// ROS_INFO_STREAM("Safety CBF: " << params.h_safe[0]);*/

// // DEFINE HERE THE CBF-RELATED FUNCTION h1 = - 1/2 * square(norm(sigma-sigma_0)) FOR TASK COMPLETION
// double quad_norm = 0.0;
// double gamma_goal = 5.0;

// for(int i=0; i<2; i++)
//     quad_norm += pow(params.sigma[i] - params.sigma_0[i], 2);
// quad_norm += pow(params.sigma[2] - params.sigma_0[2], 2);

// params.h_goal[0] =  - gamma_goal * quad_norm;

// //! CHECK IF CORRECT

// interaction_ = false;

// for(int i = 0; i < F_ext_.size(); i++){
//     if(F_ext_[i] != 0.0)
//         interaction_ = true;
// }

// /*if(interaction_)
//     params.h_goal[0] =  0.0;*/

// ROS_INFO_STREAM("Goal CBF: " << params.h_goal[0]);

// // DEFINE THE CBF-RELATED FUNCTION h_lim TO AVOID JOINT LIMITS
// double h2 = 0.0;
// double h_lim_array[6];

// /*for(int i=0; i< 6; i++)
//     params.Q_lim[i] = 0.0;

// for(int i=6; i<12; i++){
//     int n = i-6;
//     params.Q_lim[i] = ( (joint_limits[n] - 2 * initial_pos_array[n] + (-joint_limits[n])) ) / ( pow(joint_limits[n] - (-joint_limits[n]), 2) );
// }

// for(int i=0; i<6; i++){
//     params.h_lim[i] =  0.0;
// }

// for(int i=0; i<6; i++){
//     params.h_lim[i + 6] =  -0.5 * ( (joint_limits[i] - initial_pos_array[i]) * (initial_pos_array[i] - (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
//     h_lim_array[i] = -0.5 * ( (joint_limits[i] - initial_pos_array[i]) * (initial_pos_array[i] - (-joint_limits[i])) ) / ( pow(joint_limits[i] - (-joint_limits[i]), 2) );
// }

// ROS_INFO_STREAM("Limits CBF: " << params.h_lim[6]);*/

// // LOAD THE CURRENT CONFIGURATION'S JACOBIAN
// // *NdR: CVXgen stores matrices as flat-arrays in column major form (namely Aij = params.A[(i-1) + (j-1)*m)])
// int n = 0;
// for(int j=0; j < J_pinv.cols(); j++){
//     for(int i=0; i < J_pinv.rows(); i++){

//         //params.J_pinv[n] = J_pinv(i,j);
//         n++;
//     }
// }

// // HANDLE THE PRIORITIZATION OF TASKS AND SLACK VARIABLES
// //params.l[0] = 7.0; //was 4.0

// double k = 3;
// Eigen::MatrixXd K(2,3);

// /*if(d < 1.25 * D){
//     K << 1/k, -1,   0,
//           -1,  0, 1/k; 
// }

// else if(F_ext_[0] != 0.0){
//     K <<  1/k,   0, -1,
//            -1, 1/k,  0;
// }

// else {
//     K <<  -1, 1/k, 0,
//          1/k,   0,-1;
// }*/

// K << -1, 1/k;

// // params.K[0] = -1;
// // params.K[1] = 1/k;

// /*int n_1 = 0;
// for(int j=0; j < K.cols(); j++){
//     for(int i=0; i < K.rows(); i++){
//         params.K[n_1] = K(i,j);
//         n_1++;
//     }
// }*/

// /***************************************************************************************/
// /*******************PUBLISH THE SOLUTION OF THE OPTIMIZATION PROBLEM********************/
// /***************************************************************************************/
// settings.verbose = 0;
// long num_iters = solve();

// // ROS_INFO_STREAM("Delta 1: " << vars.delta[0]);
// // ROS_INFO_STREAM("Delta 2: " << vars.delta[1]);
// //ROS_INFO_STREAM("Delta 3: " << vars.delta[2]);

// //* UPDATE THE TANK VARIABLES
// Eigen::VectorXd q_dot(12);
// // Eigen::VectorXd q_dot(6);
// Eigen::VectorXd x_dot(6);

// for(int i = 0; i< x_dot.size(); i++)
//     x_dot[i] = vars.dotx[i];

// /*for(int i = 0; i < 3; i++){
//     if(x_dot[i] >= 0.25)
//         x_dot[i] = 0.25;
//     else if(x_dot[i] <= -0.25)
//         x_dot[i] = -0.25;
// }*/

// ROS_INFO_STREAM("X DOT: " << vars.dotx[0]);
// ROS_INFO_STREAM("Y DOT: " << vars.dotx[1]);
// ROS_INFO_STREAM("Z DOT: " << vars.dotx[2]);

// for(int i = 0; i < 6; i++){
//     if(x_dot[i] >= 0.40)
//         x_dot[i] = 0.40;
//     else if(x_dot[i] <= -0.40)
//         x_dot[i] = -0.40;
// }

// q_dot =  J_pinv * x_dot;
// //q_dot =  jacobian_arm.inverse() * des_twist_adm;

// desired_twist_ = x_dot;

// ROS_INFO_STREAM("X ADM: " << des_twist_adm[0]);
// ROS_INFO_STREAM("Y ADM: " << des_twist_adm[1]);
// ROS_INFO_STREAM("Z ADM: " << des_twist_adm[2]);

// Eigen::VectorXd modulation_matrix = x_dot / tank_state_;
// double delta_update = (pow(cycle_time, 2)/ (2 * pow(tank_state_, 2)))*(pow(F_ext_.transpose() * x_dot, 2));
// sum_of_delta_ += delta_update;
// tank_energy_ += cycle_time * F_ext_.transpose() * x_dot + delta_update;
// tank_state_ += cycle_time * modulation_matrix.transpose() * F_ext_;

// if(tank_state_ >= 2 * sqrt(TANK_MAX_VALUE))
//     tank_state_ = 2 * sqrt(TANK_MAX_VALUE);

// if(tank_state_ <= -2 * sqrt(TANK_MAX_VALUE))
//     tank_state_ = -2 * sqrt(TANK_MAX_VALUE);

// if(tank_energy_ >= TANK_MAX_VALUE)
//     tank_energy_ = TANK_MAX_VALUE;

// if(tank_state_ <= 0.0)
//     ROS_ERROR_STREAM("TANK EMPTY!");

// ROS_INFO_STREAM("TANK STATE: " << tank_state_);
// ROS_INFO_STREAM("TANK ENERGY: " << tank_energy_);


// // Output to files
// tank_file_ << ros::Time::now().toSec() - start_time_;
// cbf_file_ << ros::Time::now().toSec() - start_time_;
// force_file_ << ros::Time::now().toSec() - start_time_;
// tank_file_ << " " << tank_state_ << " " << tank_energy_;
// cbf_file_ << " " << - gamma_goal * quad_norm; //<< " " << - gamma_safe * pow((d_2 - pow(D,2)), 3);

// for(int i = 0; i < 6; i++){
//     cbf_file_ << " " << h_lim_array[i];
// }

// for(int i = 0; i < 6; i++){
//     force_file_ << " " << F_ext_[i];
// }

// tank_file_ << std::endl;
// cbf_file_ << std::endl;
// force_file_ << std::endl;

// // Generating the motion messages
// geometry_msgs::Twist cmd_vel;

// // *Speed saturation
// double max_vel_base = 0.25;
// double max_omega_base = 0.15;
// //double max_q_dot = 0.174533;
// double max_q_dot = 0.349066;

// cmd_vel.linear.x = q_dot[0];
// cmd_vel.linear.y = q_dot[1];
// cmd_vel.linear.z = q_dot[2];
// cmd_vel.angular.x =q_dot[3];
// cmd_vel.angular.y =q_dot[4];
// cmd_vel.angular.z =q_dot[5];

// /*if(params.h_goal[0] > -0.8){
//     max_vel_base = 0.05;
//     max_omega_base = 0.005;
// }*/

// if (cmd_vel.linear.x >= max_vel_base)
//     cmd_vel.linear.x = max_vel_base;
// else if (cmd_vel.linear.x <= - max_vel_base)
//     cmd_vel.linear.x = -max_vel_base;

// if (cmd_vel.linear.y >= max_vel_base)
//     cmd_vel.linear.y = max_vel_base;
// else if (cmd_vel.linear.y <= - max_vel_base)
//     cmd_vel.linear.y = -max_vel_base;

// if (cmd_vel.angular.z >= max_omega_base)
//     cmd_vel.angular.z = max_omega_base;
// else if (cmd_vel.angular.z <= - max_omega_base)
//     cmd_vel.angular.z = -max_omega_base;


// cycle_time = ros::Time::now().toSec() - current_time;


// // Fixing the overtime due to the first cycle being slower
// if(first_cycle_){
//     cycle_time = 0.02;
//     first_cycle_ = false; 
// }

// std::cout << "Cycle time: "<< cycle_time << std::endl;

// // Create the messages and publishing them
// trajectory_msgs::JointTrajectory arm_pose;
// arm_pose.joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
// arm_pose.points.resize(1);
// arm_pose.points[0].time_from_start = ros::Duration(cycle_time);
// // arm_pose.points[0].time_from_start = ros::Duration(0);

// for(int i= 6; i <12; i++){
//     if(q_dot[i] > max_q_dot)
//         q_dot[i] = max_q_dot;
//     else if (q_dot[i] < -max_q_dot)
//         q_dot[i] = -max_q_dot;
// }

// double delta_q[6] = {q_dot[6] * cycle_time, 
//                      q_dot[7] * cycle_time, 
//                      q_dot[8] * cycle_time,
//                      q_dot[9] * cycle_time,
//                      q_dot[10] * cycle_time,
//                      q_dot[11] * cycle_time} ;

// arm_pose.points[0].positions = {initial_pos_array[0] + delta_q[0],
//                                 initial_pos_array[1] + delta_q[1],
//                                 initial_pos_array[2] + delta_q[2],
//                                 initial_pos_array[3] + delta_q[3],
//                                 initial_pos_array[4] + delta_q[4],
//                                 initial_pos_array[5] + delta_q[5]};
//                                 //};

// /*for(int i= 0; i <6; i++){
//     if(q_dot[i] > max_q_dot)
//         q_dot[i] = max_q_dot;
//     else if (q_dot[i] < -max_q_dot)
//         q_dot[i] = -max_q_dot;
// }

// double delta_q[6] = {q_dot[0] * cycle_time, 
//                      q_dot[1] * cycle_time, 
//                      q_dot[2] * cycle_time,
//                      q_dot[3] * cycle_time,
//                      q_dot[4] * cycle_time,
//                      q_dot[5] * cycle_time} ;

// arm_pose.points[0].positions = {initial_pos_array[0] + delta_q[0],
//                                 initial_pos_array[1] + delta_q[1],
//                                 initial_pos_array[2] + delta_q[2],
//                                 initial_pos_array[3] + delta_q[3],
//                                 initial_pos_array[4] + delta_q[4],
//                                 //initial_pos_array[5] + delta_q[5]};
//                                 -1.3};*/

// base_vel_pub_.publish(cmd_vel);
// arm_pose_pub_.publish(arm_pose);

// ros::Duration(0.005).sleep();

/**********************************************/
/******************** DEBUG *******************/
/**********************************************/


}





