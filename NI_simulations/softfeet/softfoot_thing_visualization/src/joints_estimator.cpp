/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#include <fstream>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "yaml-cpp/yaml.h"

#include "softfoot_thing_visualization/utils/parsing_utilities.h"
#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JE        0       // Prints out additional info about the Object
#define     DEBUG_JS        0       // Prints out info about estimated joint states
#define     DEBUG_PARSED    0       // Prints out info about parsed stuff
#define     DEBUG_ANGLES    0       // Prints out only raw estimated angles
#define     DEBUG_CHAIN     1       // Publishes frames to RViz

#define     N_CAL_IT        100     // Number of calibration iterations
#define     UPPER_COUP      60.0    // Upper (downwards) coupling angle limit for arch links
#define     LOWER_COUP      0.0     // Lower (upwards) coupling angle limit for arch links
#define     TIPINSMAX       0.03    // Maximum admissible tip insertion distance for chain

using namespace softfoot_thing_visualization;

JointsEstimator::JointsEstimator(ros::NodeHandle& nh , int foot_id, std::string foot_name) : async_spinner(4) {

    // Initializing main variables
    this->foot_id_ = foot_id;
    this->foot_name_ = foot_name;
    this->robot_name_ = this->foot_name_ + "_" + std::to_string(this->foot_id_);
    this->pkg_path_ = ros::package::getPath("softfoot_thing_visualization");

    // Initializing ros variables
    this->je_nh_ = nh;

    // Setting up the filter (This is needed here otherwise when switching controllers, filter memory is not cleared!)
    this->lp_filter_.clear();
    for (int i = 0; i < 4; i++) {
        std::vector<std::shared_ptr<filters::FilterChain<double>>> tmp_lp_vec;
        for (int j = 0; j < 3; j++) {
            tmp_lp_vec.push_back(std::make_shared<filters::FilterChain<double>>("double"));
            tmp_lp_vec[j]->configure("softfoot_viz/low_pass_filter", this->je_nh_);
        }
        this->lp_filter_.push_back(tmp_lp_vec);
    }

    // Initializing subscribers
    this->sub_imu_acc_ = this->je_nh_.subscribe<nmmi_msgs::inertialSensorArray>(this->imu_topic_acc_,
        1, &JointsEstimator::acc_callback, this);
    nmmi_msgs::inertialSensorArray::ConstPtr temp_msg_2 = ros::topic::waitForMessage
        <nmmi_msgs::inertialSensorArray>(this->imu_topic_acc_, ros::Duration(2.0));
    
    this->sub_imu_gyro_ = this->je_nh_.subscribe<nmmi_msgs::inertialSensorArray>(this->imu_topic_gyro_,
        1, &JointsEstimator::gyro_callback, this);
    nmmi_msgs::inertialSensorArray::ConstPtr temp_msg_3 = ros::topic::waitForMessage
        <nmmi_msgs::inertialSensorArray>(this->imu_topic_gyro_, ros::Duration(2.0));
    
    this->pub_js_ = this->je_nh_.advertise<sensor_msgs::JointState>
        ("/" + this->robot_name_ + "/joint_states", 1);

    // Temporarily building parsable variables here (TODO: parse them)
    this->joint_pairs_ = {{0, 1}, {0, 3}, {1, 2}};
    this->joint_names_ = {"front_arch_joint", "back_arch_joint", "roll_joint"};
    this->joint_frame_names_ = {"front_arch_link", "back_arch_link", "roll_link"};

    // Temporarily building parsable variables here (TODO: parse them)
    Eigen::Vector3d x_loc(1, 0, 0);
    Eigen::Vector3d y_loc(0, 1, 0);
    Eigen::Vector3d z_loc(0, 0, 1);
    this->axes_pairs_ = {{y_loc, x_loc}, {-y_loc, -y_loc}, {y_loc, -z_loc}};

    // Setting the complimentary filter weights
    this->gyro_weights_.resize(this->joint_pairs_.size());
    std::fill(this->gyro_weights_.begin(), this->gyro_weights_.end(), 0.5);

    // Parsing needed parameters
    if (!this->parse_parameters(this->je_nh_)) {
        ROS_FATAL_STREAM("JointsEstimator::JointsEstimator : Could not get parameters for " 
            << this->robot_name_ << " !");
        this->je_nh_.shutdown();
    }

    // Setting up the solvers for foot chain reconstruction
    this->lma_weight_ << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    this->fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->chain_chain_));
    this->fk_solver_ins_.reset(new KDL::ChainFkSolverPos_recursive(this->ins_chain_));
    this->ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(this->chain_chain_));
    this->ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(this->chain_chain_,this->chain_min_,
        this->chain_max_,*this->fk_pos_solver_,*this->ik_vel_solver_, 1000, 1e-3));
    this->ik_lma_solver_.reset(new KDL::ChainIkSolverPos_LMA(this->chain_chain_));

    // Filling up main parts of the joint state msg and setting size of values
    for (auto it : this->joint_names_) {
        this->joint_states_.name.push_back(this->robot_name_ 
            + "_" + it);
        this->joint_states_.position.push_back(0.0);
    }
    this->joint_values_.resize(this->joint_pairs_.size());
    this->joint_values_old_.resize(this->joint_pairs_.size());
    this->joint_values_gyro_.resize(this->joint_pairs_.size());
    this->js_values_.resize(this->joint_pairs_.size());

    // Setting joint states and base of the foot chain
    this->q_chain_.resize(this->chain_chain_.getNrOfJoints());
    this->q_ins_.resize(this->ins_chain_.getNrOfJoints());
    this->q_chain_lma_.resize(this->chain_chain_.getNrOfJoints());
    this->q_chain_base_.resize(this->chain_chain_.getNrOfJoints());
    // The following is by trial and error
    this->q_chain_base_ << 0.5, -0.1, -0.15, -0.15, 0.0, -0.2, -0.2, -0.2, -0.1;
    // this->q_chain_base_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    this->q_chain_.data = this->q_chain_base_;

    // Adding also chain joint states
    for(auto it : this->chain_names_) {
        for (int i = 1; i <= this->chain_chain_.getNrOfJoints(); i++) {
            this->joint_states_.name.push_back(this->robot_name_
                + "_" + it + "_" + std::to_string(i) + "_joint");
            this->joint_states_.position.push_back(0.0);
        }
    }

    // Eventually adding the leg joint to jointstates
    if (this->publish_leg_pose_) {
        ROS_WARN_STREAM("Publishing also leg pose for " << this->robot_name_ << "!");
        this->joint_states_.name.push_back("world_to_leg_" + std::to_string(this->foot_id_));
        this->joint_states_.position.push_back(0.0);
    }

    // Setting all accelerations and ang vels to null
    this->acc_vec_0_.resize(4);
    this->acc_vec_.resize(4);
    this->acc_vec_olds_.resize(4);
    this->acc_vec_raw_.resize(4);
    this->gyro_vec_.resize(4);
    for (int i = 0; i < 4; i++) {
        this->acc_vec_0_[i] = Eigen::Vector3d::Zero();
        this->acc_vec_[i] = Eigen::Vector3d::Zero();
        this->acc_vec_olds_[i] = Eigen::Vector3d::Zero();
        this->acc_vec_raw_[i] = Eigen::Vector3d::Zero();
        this->gyro_vec_[i] = Eigen::Vector3d::Zero();
    }

    // Initializing time variables
    this->last_integration_time_ = ros::Time::now();

}

JointsEstimator::~JointsEstimator(){

    // Nothing to do here

}

// Function that calibrates the sensing
void JointsEstimator::calibrate(){

    this->async_spinner.start();

    Eigen::Vector3d sample;

    for (int k = 0; k < N_CAL_IT; k++) {
        // Spin to recieve messages
        ros::spinOnce();

        // Fill up the initial accelerations and others
        for (int i = 0; i < 4; i++) {
            sample << this->imu_acc_[i].x, this->imu_acc_[i].y, this->imu_acc_[i].z;
            this->acc_vec_0_[i] = (this->acc_vec_0_[i] + sample);
        }
    }

    // Divide by N_CAL_IT to get the mean
    for (int i = 0; i < 4; i++) {
        this->acc_vec_0_[i] = (this->acc_vec_0_[i] / N_CAL_IT);
    }

    // Setting current and old accs to initial
    this->acc_vec_ = this->acc_vec_0_;
    this->acc_vec_olds_ = this->acc_vec_0_;

    // Set calibrated flag
    this->calibrated_ = true;

    this->async_spinner.stop();

}

// Function that calibrates the sensing and saves the calibration data to yaml
void JointsEstimator::calibrate_and_save(std::string file_name){

    // Get the path to the softfoot viz config folder and append the file name
    std::string config_file_path = this->pkg_path_ + "/configs/" + file_name + ".yaml";
    std::cout << "File path is " << config_file_path << "." << std::endl;

    // Create yaml emitter and prepare it with foot details
    YAML::Emitter yaml_out;
    yaml_out << YAML::Comment("Calibration Data for SoftFoot Joint Estimation");

    // Calibrating the foot sensors
    this->calibrate();
    std::cout << "The calibrated accelerations are: " << std::endl;
    for (auto it : this->acc_vec_0_) {
        std::cout << it.transpose() << std::endl;
    }
    
    // Writing result of calibration to yaml
    yaml_out << YAML::BeginMap;                     // begin: foot name
    yaml_out << YAML::Key << this->robot_name_;

    yaml_out << YAML::BeginMap;                     // begin: base_accelerations
    yaml_out << YAML::Key << "base_accelerations";

    // Saving all of the base calibrated accelerations
    yaml_out << YAML::BeginSeq;
    for (auto it : this->acc_vec_0_) {
        std::vector<double> temp = {it(0), it(1), it(2)};
        yaml_out << YAML::Flow << temp;
    }
    yaml_out << YAML::EndSeq;

    yaml_out << YAML::EndMap;                       // end: foot name
    yaml_out << YAML::EndMap;                       // end: base_accelerations

    // Save the emmitter to file
    std::ofstream file_out(config_file_path);
    file_out << yaml_out.c_str();

}

// Function that spins the estimator
bool JointsEstimator::check_calibration(){

    // Check if the foot has been calibrated
    if (!this->calibrated_) {
        ROS_WARN_STREAM("No on the fly calibration was requested for " << this->robot_name_ 
            << ", looking for an offline calibration data file for this foot!");

        // Trying to parse an offline calibration yaml
        if (!this->je_nh_.getParam(this->robot_name_, this->je_params_)) {
            ROS_FATAL_STREAM("No offline calibration found for " << this->robot_name_ 
            << ", this is bad!");
            return false;
        } else {
            // Getting parameters
            Eigen::MatrixXd base_accelerations(4, 3);
            parseParameter(this->je_params_, base_accelerations, "base_accelerations");

            // Check for parameter consistency
            if (base_accelerations.rows() != this->acc_vec_0_.size() || 
                base_accelerations.cols() != 3) {
                    ROS_FATAL_STREAM("The parsed calibration for " << this->robot_name_ 
                        << " is inconsistent, this is bad!");
                    return false;
            }

            // Filling vector of base accelerations from parsed matrix
            Eigen::Vector3d sample;
            for (int i = 0; i < this->acc_vec_0_.size(); i++) {
                sample << base_accelerations(i, 0), base_accelerations(i, 1), base_accelerations(i, 2);
                this->acc_vec_0_[i] = sample;
            }

            std::cout << "The parsed calibration matrix is " << std::endl;
            std::cout << base_accelerations << std::endl;
        }

        // Set calibrated
        this->calibrated_ = true;
    }

    return true;

}

// Function that estimates the joint angles
bool JointsEstimator::estimate(){

    // Check if the foot has been calibrated
    if (!this->calibrated_) {
        ROS_FATAL_STREAM("Foot " << this->robot_name_  
            << "has not been calibrated, this is bad!");
        return false;
    }

    // Save old raw joint values to be used for gyro integration
    this->joint_values_old_ = this->js_values_;

    // Estimating the angles from accelerations
    for (int i = 0; i < this->joint_pairs_.size(); i++) {
        this->joint_values_[i] = this->compute_joint_state_from_pair(this->joint_pairs_[i]);
    }

    // Integrating the angles from angular velocities
    for (int i = 0; i < this->joint_pairs_.size(); i++) {
        this->joint_values_gyro_[i] = this->integrate_joint_state_from_pair(this->joint_pairs_[i]);
    }

    // Filling up jointstates and publishing
    this->fill_and_publish();

    if (DEBUG_ANGLES) {
        ROS_INFO_STREAM("The estimated acc joint angle for imu pair (" << this->joint_pairs_[0].first
            << ", " << this->joint_pairs_[0].second << ") is " << this->joint_values_[0] << "rads \n");
        ROS_INFO_STREAM("The estimated acc joint angle for imu pair (" << this->joint_pairs_[1].first
            << ", " << this->joint_pairs_[1].second << ") is " << this->joint_values_[1] << "rads \n");
        ROS_INFO_STREAM("The estimated acc joint angle for imu pair (" << this->joint_pairs_[2].first
            << ", " << this->joint_pairs_[2].second << ") is " << this->joint_values_[2] << "rads \n");
        ROS_INFO_STREAM("The estimated gyro joint angle for imu pair (" << this->joint_pairs_[0].first
            << ", " << this->joint_pairs_[0].second << ") is " << this->joint_values_gyro_[0] << "rads \n");
        ROS_INFO_STREAM("The estimated gyro joint angle for imu pair (" << this->joint_pairs_[1].first
            << ", " << this->joint_pairs_[1].second << ") is " << this->joint_values_gyro_[1] << "rads \n");
        ROS_INFO_STREAM("The estimated gyro joint angle for imu pair (" << this->joint_pairs_[2].first
            << ", " << this->joint_pairs_[2].second << ") is " << this->joint_values_gyro_[2] << "rads \n");
    }

    if (DEBUG_JS) {
        ROS_INFO_STREAM("The estimated joint state for imu pair (" << this->joint_pairs_[0].first 
            << ", " << this->joint_pairs_[0].second << ") is " << this->js_values_[0] << "rads \n");
        ROS_INFO_STREAM("The estimated joint state for imu pair (" << this->joint_pairs_[1].first 
            << ", " << this->joint_pairs_[1].second << ") is " << this->js_values_[1] << "rads \n");
        ROS_INFO_STREAM("The estimated joint state for imu pair (" << this->joint_pairs_[2].first 
            << ", " << this->joint_pairs_[2].second << ") is " << this->js_values_[2] << "rads \n");
    }

    // Returning
    return true;

}

// Function to parse parameters
bool JointsEstimator::parse_parameters(ros::NodeHandle& nh){
    
    // TODO: parse needed params

    // Parse if leg pose has to be published
    if (!this->je_nh_.getParam("softfoot_viz/publish_leg_pose", this->publish_leg_pose_)) {
        ROS_WARN_STREAM("Could not understand if you want me to publish leg pose for " << this->robot_name_ 
        << ", this is strange! By default, I don't publish!");
    }

    // Check if filter is needed
    if (!this->je_nh_.getParam("softfoot_viz/use_filter", this->use_filter_)) {
        ROS_WARN_STREAM("Could not understand if you want me to filter the measurments for " << this->robot_name_ 
        << ", this is strange! By default, I don't filter!");
    }

    // Check if gyro fusion is requested
    if (!this->je_nh_.getParam("softfoot_viz/use_gyro", this->use_gyro_)) {
        ROS_WARN_STREAM("Could not understand if you want me to fues also the gyro for " << this->robot_name_
        << ", this is strange! By default, I don't filter!");
    }

    // Parsing joint limits of the foot (joint_names_ needs to be set before)
    if (!this->get_joint_limits(nh)) {
        ROS_FATAL_STREAM("Unable to get the joint limits for " << this->robot_name_ << "!");
        return false;
    }

    // Everything parsed correctly
    return true;

}

// Function to get joint limits
bool JointsEstimator::get_joint_limits(ros::NodeHandle& nh){
    
    // Construct an URDF model from the xml string
    std::string xml_string;
    if (!nh.getParam("robot_description", xml_string)) {
        ROS_FATAL("Cannot find robot_description, shutting down!");
        return false;
    }
    
    if (xml_string.size() == 0) {
        ROS_FATAL("Unable to load robot model from robot_description!");
        return false;
    }
    
    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string)) {
        ROS_FATAL("Failed to get robot model from description!");
        return false;
    }
    
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct kdl tree of robot!");
        return false;
    }

    // Get the joint limits from the tree according to robot name
    bool all_limits_found = true;
    std::shared_ptr<const urdf::Joint> joint;
    this->joint_limits_.resize(this->joint_names_.size());
    for (int i = 0; i < this->joint_names_.size(); i++) {
        joint = model.getJoint(this->robot_name_ + "_" + this->joint_names_[i]);
        this->joint_limits_[i] = {joint->limits->lower, joint->limits->upper};
    }

    if (DEBUG_PARSED) {
        ROS_INFO_STREAM("The joint limits of " << this->robot_name_ << " are ");
        std::cout << "/ - ";
        for (auto it : this->joint_limits_) {
            std::cout << "(" << it.first << ", " << it.second << ") ";
        }
        std::cout << "- /\n" << std::endl;
    }

    // Getting also the kinematic chain of the foot chain and around it (chains start from front_roll_link)
    kdl_tree.getChain(this->robot_name_ 
        + "_front_roll_link", this->foot_name_ + "_" 
        + std::to_string(this->foot_id_) + "_" + this->chain_names_[0] + "_tip_link",
        this->chain_chain_);

    kdl_tree.getChain(this->robot_name_ 
        + "_front_roll_link", this->foot_name_ + "_" 
        + std::to_string(this->foot_id_) + "_" + this->chain_names_[0] + "_insertion_link",
        this->ins_chain_);

    // Parse also the joint limits for the foot chain
    this->chain_min_.resize(this->chain_chain_.getNrOfJoints());
    this->chain_max_.resize(this->chain_chain_.getNrOfJoints());

    // Parsing from 9_link and not tip_link because of issues with fixed joints
    int index;
    urdf::JointConstSharedPtr joint_;
    urdf::LinkConstSharedPtr link_ = model.getLink(this->foot_name_ 
        + "_" + std::to_string(this->foot_id_) + + "_" + this->chain_names_[0] + "_9_link");

    for (int i = 0; i < this->chain_chain_.getNrOfJoints() && link_; i++) {
        joint_ = model.getJoint(link_->parent_joint->name);
        index = this->chain_chain_.getNrOfJoints() - i - 1;
    
        this->chain_min_(index) = joint_->limits->lower;
        this->chain_max_(index) = joint_->limits->upper;

        link_ = model.getLink(link_->getParent()->name);
    }

    // Finally return success
    return true;

}

// Function to increase gyro weights only when needed
void JointsEstimator::compute_gyro_weights(){

    // Iterate over the accelerations and see if any in algorithmic singularity
    Eigen::Vector3d acceleration, axis;
    double weight;

    for (int i = 0; i < this->joint_pairs_.size(); i++) {

        /* 1) First imu of joint */
        acceleration = this->acc_vec_[this->joint_pairs_[i].first];
        axis = this->axes_pairs_[i].first;

        // Check how much the acceleration is aligned with axis
        acceleration.normalize(); axis.normalize();
        weight = acceleration.dot(axis);

        /* 2) Second imu of joint */
        acceleration = this->acc_vec_[this->joint_pairs_[i].second];
        axis = this->axes_pairs_[i].second;

        // Check how much the acceleration is aligned with axis
        acceleration.normalize(); axis.normalize();
        if (acceleration.dot(axis) > weight) {
            weight = acceleration.dot(axis);
        }

        // Debug print out
        if (DEBUG_ANGLES) {
            std::cout << "The " << i << "the joint gyro weight is " <<
                         std::abs(weight) << std::endl;
        }

        // Absolute valuew of weight is exactly the gyro_weight for joint;
        this->gyro_weights_[i] = std::abs(weight);
    }

}

// Function to correct the offset form estimated angles
void JointsEstimator::correct_offset(){

    // Compute the weights for the complementary filter
    this->compute_gyro_weights();

    // Complementary filter using weighted acc. and gyro estimated joint states
    for (int i = 0; i < this->joint_values_.size(); i++) {
        if (this->use_gyro_) {
            // TODO: make the weights variable and dynamic case by case
            this->js_values_[i] = (1.0 - this->gyro_weights_[i]) * this->joint_values_[i]
                    + this->gyro_weights_[i] * this->joint_values_gyro_[i];
        } else {
            this->js_values_[i] = this->joint_values_[i];
        }
    }

}

// Function to enforce joint limits
void JointsEstimator::enforce_limits(){
    
    // Saturate if estimated joint values outside limits
    for (int i = 0; i < this->js_values_.size(); i++) {
        if (this->js_values_[i] <= this->joint_limits_[i].first) {
            this->js_values_[i] = this->joint_limits_[i].first;
        } else if (this->js_values_[i] >= this->joint_limits_[i].second) {
            this->js_values_[i] = this->joint_limits_[i].second;
        }
    }

}

// Function to enforce arch links joint coupling
void JointsEstimator::enforce_coupling(){

    // Saturate if coupling if violated
    if ((this->js_values_[0] + this->js_values_[1]) > deg2rad(double(UPPER_COUP))) {
        this->js_values_[1] = deg2rad(double(UPPER_COUP)) - this->js_values_[0];
    } else if ((this->js_values_[0] + this->js_values_[1]) < deg2rad(double(LOWER_COUP))) {
        this->js_values_[1] = deg2rad(double(LOWER_COUP)) - this->js_values_[0];
    }

}

// Function to echo transform from pair of frames
Eigen::Affine3d JointsEstimator::getTransform(std::string frame_1, std::string frame_2){
    
    // Tf echoing using input frame names
    try {
		this->tf_listener_.waitForTransform(std::string("/") + frame_1,
      std::string("/") + frame_2, ros::Time(0), ros::Duration(1.0) );
		this->tf_listener_.lookupTransform(std::string("/") + frame_1,
      std::string("/") + frame_2, ros::Time(0), this->stamped_transform_);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Converting to Affine3d
    Eigen::Affine3d affine;
    tf::Transform transform(this->stamped_transform_.getRotation(),
      this->stamped_transform_.getOrigin());
    tf::transformTFToEigen(transform, affine);

    return affine;

}

// Function to get the joint axes fron joint name
Eigen::Vector3d JointsEstimator::get_joint_axis(std::string joint_name){

    // Getting the transform to joint frame
    int pos = std::find(this->joint_names_.begin(), this->joint_names_.end(),
         joint_name) - this->joint_names_.begin();
    if (pos >= this->joint_names_.size()) {
        ROS_FATAL_STREAM("JointsEstimator::get_joint_axis : You specified for " << this->robot_name_ 
            << " a joint name which is unknown to me!");
        this->je_nh_.shutdown();
    }
    Eigen::Affine3d joint_frame = this->getTransform("world", this->foot_name_ + "_" 
        + std::to_string(this->foot_id_) + "_" + this->joint_frame_names_[pos]);

    // Getting the joint's axis in world frame (TODO: parse local axis)
    Eigen::Vector3d loc_axis;
    if (pos == 0) {
        loc_axis << 0, 1, 0;
    } else if (pos == 1) {
        loc_axis << 0, -1, 0;
    } else if (pos == 2) {
        loc_axis << 1, 0, 0;
    } else {
        ROS_FATAL_STREAM("JointsEstimator::get_joint_axis : You specified for " << this->robot_name_ 
            << " a joint name which is unknown to me! But this should have not happened!!!");
        this->je_nh_.shutdown();
    }

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("Got axis for " << joint_name << ": \n" << joint_frame.rotation() * loc_axis);
        ROS_INFO_STREAM("Frame rotation is \n" << joint_frame.rotation());
        ROS_INFO_STREAM("Local axis is \n" << loc_axis << "\n");
    }

    // Return joint axis in global frame
    return joint_frame.rotation() * loc_axis;

}

// Function to compute perpendicular plane to a given vector
bool JointsEstimator::compute_perpendiculars(Eigen::Vector3d in, Eigen::Vector3d &out_1,
    Eigen::Vector3d &out_2){

    // If input vector is near to null, return
    if ((in - Eigen::Vector3d::Zero()).isMuchSmallerThan(0.0001)) return false;
    
    Eigen::Vector3d v; Eigen::Vector3d w;
    // Checking if the input vector is on some main plane, else get traditional perpendicular
    if (std::abs(in(0) - 0.0) < 0.0001) {
        v << 1, 0, 0;
    } else if (std::abs(in(1) - 0.0) < 0.0001) {
        v << 0, 1, 0;
    } else if (std::abs(in(2) - 0.0) < 0.0001) {
        v << 0, 0, 1;
    } else {
        v << -in(1), in(0), in(2);
    }

    // Computing the next vector normal to both in and w
    w = in.cross(v);

    // Normalizing v and w and retuning
    v.normalize(); w.normalize();
    out_1 = v; out_2 = w;

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("The normals to \n" << in << "\n are \n" << out_1 << "\n and \n" << out_2 << "\n");
    }

    return true;
}

// Function to compute the joint angle from pair of imu ids
float JointsEstimator::compute_joint_state_from_pair(std::pair<int, int> imu_pair){

    // 1) Getting joint name for current imu pair
    int pos = std::find(this->joint_pairs_.begin(), this->joint_pairs_.end(),
         imu_pair) - this->joint_pairs_.begin();
    if (pos >= this->joint_pairs_.size()) {
        ROS_FATAL_STREAM("JointsEstimator::compute_joint_state_from_pair : You specified for " 
            << this->robot_name_ << "  an imu pair which is unknown to me!");
        this->je_nh_.shutdown();
    }

    // Defining some variables
    Eigen::Vector3d axis, acc_0, acc_1;
    double determinant, dot_product;
    float angle_1, angle_2, js;
    
    // 2.1) Case imu1 of pair: compute the angle variation in acc vector around joint axis
    axis = this->axes_pairs_[pos].first; axis.normalize();
    acc_0 = this->acc_vec_0_[this->joint_pairs_[pos].first]; acc_0.normalize();
    acc_1 = this->acc_vec_[this->joint_pairs_[pos].first]; acc_1.normalize();

    // 2.2) Compute the first angle (variation of first imu inclination around the joint axis)
    determinant = axis.dot(acc_0.cross(acc_1));             // Calculating det as triple product
    dot_product = acc_0.dot(acc_1);                         // Getting the dot product
    angle_1 = (float) atan2(determinant, dot_product);

    // 3.1) Case imu2 of pair: compute the angle variation in acc vector around joint axis
    axis = this->axes_pairs_[pos].second; axis.normalize();
    acc_0 = this->acc_vec_0_[this->joint_pairs_[pos].second]; acc_0.normalize();
    acc_1 = this->acc_vec_[this->joint_pairs_[pos].second]; acc_1.normalize();

    // 3.2) Compute the second angle (variation of second imu inclination around the joint axis)
    determinant = axis.dot(acc_0.cross(acc_1));             // Calculating det as triple product
    dot_product = acc_0.dot(acc_1);                         // Getting the dot product
    angle_2 = (float) atan2(determinant, dot_product);

    // 4) Compute the final joint state combining the two angles
    js = angle_1 - angle_2;

    // 5) Return the found joint state of the joint relative to the specifies pair
    return js;

}

// Function to additionally integrate the joint angle using gyro measurments
float JointsEstimator::integrate_joint_state_from_pair(std::pair<int, int> imu_pair){

    // 1) Getting joint name for current imu pair
    int pos = std::find(this->joint_pairs_.begin(), this->joint_pairs_.end(),
         imu_pair) - this->joint_pairs_.begin();
    if (pos >= this->joint_pairs_.size()) {
        ROS_FATAL_STREAM("JointsEstimator::integrate_joint_state_from_pair : You specified for "
            << this->robot_name_ << "  an imu pair which is unknown to me!");
        this->je_nh_.shutdown();
    }

    // Defining some variables
    Eigen::Vector3d axis, gyro;
    float ang_vel_1, ang_vel_2, js;

    // 2.1) Case imu1 of pair: get the angula velocity around joint axis
    axis = this->axes_pairs_[pos].first; axis.normalize();
    gyro = this->gyro_vec_[this->joint_pairs_[pos].first];
    ang_vel_1 = gyro.dot(axis);

    // 2.2) Case imu2 of pair: get the angula velocity around joint axis
    axis = this->axes_pairs_[pos].second; axis.normalize();
    gyro = this->gyro_vec_[this->joint_pairs_[pos].second];
    ang_vel_2 = gyro.dot(axis);

    // 3) Computing joint value by integration
    this->dt_ = ros::Time::now() - this->last_integration_time_;
    js = this->joint_values_old_[pos] + (ang_vel_1 + ang_vel_2) * this->dt_.toSec();

    // Setting last integration time and returning
    this->last_integration_time_ = ros::Time::now();
    return js;
}

// Function to avoid gradient descent NR for chain getting stuck
void JointsEstimator::facilitate_chain_ik(){

    // Get distance of tip from insertion
    this->tip_to_ins_ = this->getTransform(this->robot_name_ 
        + "_" + this->chain_names_[0] + "_tip_link", this->robot_name_
        + "_" + this->chain_names_[0] + "_insertion_link");

    // If the distance is too much, reset the chain joint config to zeros
    if (this->tip_to_ins_.translation().norm() > double(TIPINSMAX)) {
        this->q_chain_.data = this->q_chain_base_;
    }

}

// Function to compute soft chain IK
void JointsEstimator::chain_ik(){

    // Fill the joint array of the kinematic chain aroung the soft chain (TODO: remove hard code)
    this->q_ins_.data << this->js_values_[2], this->js_values_[0],
        this->js_values_[1], -this->js_values_[3];

    // Get the pose of the chain insertion in chain base
    this->fk_solver_ins_->JntToCart(this->q_ins_, this->real_ins_pose_);

    // Publish debug frames
    if (DEBUG_CHAIN) {
        tf::transformKDLToTF(this->real_ins_pose_ , this->debug_transform_);
        this->tf_broadcaster_.sendTransform(tf::StampedTransform(this->debug_transform_, 
            ros::Time::now(), this->robot_name_ 
            + "_front_roll_link", "debug_chain_tip"));
    }

    // Avoid ik getting stuck
    this->facilitate_chain_ik();

    // First compute ik using Levenberg-Marquardt for singularity robustness
    this->q_chain_.data = this->q_chain_base_;
    int res_lma = this->ik_pos_solver_->CartToJnt(this->q_chain_, this->real_ins_pose_, 
        this->q_chain_lma_);

    // Compute ik of the tip pose from the Levenberg-Marquardt guess to enforce joint limits
    int res = this->ik_lma_solver_->CartToJnt(this->q_chain_lma_, this->real_ins_pose_, 
        this->q_chain_);

}

// Function to compute the leg joint state
float JointsEstimator::compute_leg_joint(){

    // Defining needed variables
    Eigen::Vector3d axis, acc_0, acc_1;
    double determinant, dot_product;
    float js;
    
    // 2) Suppose the 1st imu of the 1st joint pair is in the ankle - getting the vectors
    axis = this->axes_pairs_[0].first; axis.normalize();
    acc_0 = this->acc_vec_0_[this->joint_pairs_[0].first]; acc_0.normalize();
    acc_1 = this->acc_vec_[this->joint_pairs_[0].first]; acc_1.normalize();

    // 3) Compute the first angle (variation of first imu inclination around the joint axis)
    determinant = axis.dot(acc_0.cross(acc_1));             // Calculating det as triple product
    dot_product = acc_0.dot(acc_1);                         // Getting the dot product
    js = (float) atan2(determinant, dot_product);

    // 5) Saturate if estimated joint values outside limits (TODO: remove hard code)
    if (js <= -0.5) {
        js = -0.5;
    } else if (js >= 0.5) {
        js = 0.5;
    }

    // Return the leg joint state (this might be opposite in sign)
    return -js;

}

// Function to check if joint states are "publishable"
bool JointsEstimator::states_publishable(){

    // Get the two frames from fk
    this->fk_pos_solver_->JntToCart(this->q_chain_, this->real_tip_pose_);
    this->fk_solver_ins_->JntToCart(this->q_ins_, this->real_ins_pose_);

    // Check if chain ik has actually been achieved and return accordingly
    if (KDL::Equal(this->real_tip_pose_.p, this->real_ins_pose_.p, 0.02)) {
        return true;
    } else {
        return false;
    }

}

// Function to fill joint states with est. values and publish
void JointsEstimator::fill_and_publish(){

    // Correcting offset and enforcing the limits and coupling on current estimation
    this->correct_offset();
    this->enforce_limits();
    this->enforce_coupling();

    // Estimate the soft chain joint states
    this->chain_ik();

    // Estimate leg joint state
    this->leg_js_ = this->compute_leg_joint();

    // Filling up the msg
    this->joint_states_.header.stamp = ros::Time::now();
    for (int i = 0; i < this->joint_values_.size(); i++) {
        this->joint_states_.position[i] = (this->js_values_[i]);
    }

    // Fill chain joint states (HP: same joint states for all joints)
    int starting_index = this->joint_names_.size();
    int ending_index = this->joint_names_.size() + this->chain_chain_.getNrOfJoints();
    for (auto it : this->chain_names_) {
        for (int i = starting_index; i < ending_index; i++) {
            this->joint_states_.position[i] = (this->q_chain_.data(i - starting_index));
        }
        starting_index += this->chain_chain_.getNrOfJoints();
        ending_index += this->chain_chain_.getNrOfJoints();
    }

    // Eventually filling in also the leg joint state
    if (this->publish_leg_pose_) {
        this->joint_states_.position[this->joint_names_.size() + 3 * this->chain_chain_.getNrOfJoints()] =
            (this->leg_js_);
    }

    // Publish to the topic
    if (this->states_publishable()) this->pub_js_.publish(this->joint_states_);

}

// Callback to imu accelerations topic
void JointsEstimator::acc_callback(const nmmi_msgs::inertialSensorArray::ConstPtr &msg){

    // Get the imu angles of foot after clearing old angles
    this->imu_acc_.resize(4);
    for (int i = 0; i < msg->m.size(); i++) {
        if (msg->m[i].board_id == this->foot_id_) {
            this->imu_acc_[msg->m[i].id] = msg->m[i];
        }
    }

    // Debug print out
    if (DEBUG_JE) {
        std::cout << "\n *-----------* \n IMU messages recieved \n *-----------* \n";
        ROS_INFO_STREAM("Saved accelerations for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ - ";
        for (auto it : this->imu_acc_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/\n" << std::endl;
    }

    // Save raw accelerations
    for (int i = 0; i < 4; i++) {
        this->acc_vec_raw_[i] << this->imu_acc_[i].x, this->imu_acc_[i].y, this->imu_acc_[i].z;
//        std::cout << "Saved Acc: " << this->imu_acc_[i].x << this->imu_acc_[i].y << this->imu_acc_[i].z << std::endl;
    }

    // Save old accelerations and push new ones
    this->acc_vec_olds_ = this->acc_vec_;

    // Filter accelerations if needed
    if (this->use_filter_) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                this->lp_filter_.at(i).at(j)->update(this->acc_vec_raw_[i](j), this->acc_vec_[i](j));
            }
        }
    } else {
        this->acc_vec_ = this->acc_vec_raw_;
    }

    // Remove high accelerations replacing them with old ones
    for (int i = 0; i < 4; i++) {
        if (this->imu_acc_[i].x > 1.5 || this->imu_acc_[i].y > 1.5 || this->imu_acc_[i].z > 1.5) {
            this->acc_vec_[i] = this->acc_vec_olds_[i];
        }
    }

}

// Callback to imu angular velocities topic
void JointsEstimator::gyro_callback(const nmmi_msgs::inertialSensorArray::ConstPtr &msg){

    // Get the imu angles of foot after clearing old angles
    this->imu_gyro_.resize(4);
    for (int i = 0; i < msg->m.size(); i++) {
        if (msg->m[i].board_id == this->foot_id_) {
            this->imu_gyro_[msg->m[i].id] = msg->m[i];
        }
    }

    // Debug print out
    if (DEBUG_JE) {
        std::cout << "\n *-----------* \n IMU messages recieved \n *-----------* \n";
        ROS_INFO_STREAM("Saved angular velocities for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ - ";
        for (auto it : this->imu_gyro_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/\n" << std::endl;
    }

    // Save raw angular velocities
    for (int i = 0; i < 4; i++) {
        this->gyro_vec_[i] << this->deg2rad(this->imu_gyro_[i].x), this->deg2rad(this->imu_gyro_[i].y), this->deg2rad(this->imu_gyro_[i].z);
    }

}
