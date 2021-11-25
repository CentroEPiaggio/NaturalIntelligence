/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#ifndef JOINTS_ESTIMATOR_H
#define JOINTS_ESTIMATOR_H

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <string>
#include <mutex>

// MSG includes
#include <sensor_msgs/JointState.h>
#include "nmmi_msgs/inertialSensor.h"
#include "nmmi_msgs/inertialSensorArray.h"

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

// Filter includes
#include "filters/filter_chain.h"

// Other includes
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

namespace softfoot_thing_visualization {

class JointsEstimator {

    public:

        JointsEstimator(ros::NodeHandle& nh, int foot_id, std::string foot_name);

        ~JointsEstimator();

        // Function that calibrates the sensing
        void calibrate();

        // Function that calibrates the sensing and saves the calibration data to yaml
        void calibrate_and_save(std::string file_name);

        // Function that spins the estimator
        bool check_calibration();

        // Function that estimates the joint angles
        bool estimate();

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Function to get joint limits
        bool get_joint_limits(ros::NodeHandle& nh);

        // Function to increase gyro weights only when needed
        void compute_gyro_weights();

        // Function to correct the offset form estimated angles
        void correct_offset();

        // Function to enforce joint limits
        void enforce_limits();

        // Function to enforce arch links joint coupling
        void enforce_coupling();

        // Function to echo transform from pair of frames
        Eigen::Affine3d getTransform(std::string frame_1, std::string frame_2);

        // Function to get the joint axes fron joint name
        Eigen::Vector3d get_joint_axis(std::string joint_name);

        // Function to compute perpendicular plane to a given vector
        bool compute_perpendiculars(Eigen::Vector3d in, Eigen::Vector3d &out_1, Eigen::Vector3d &out_2);

        // Function to compute the joint angle from pair of imu ids
        float compute_joint_state_from_pair(std::pair<int, int> imu_pair);

        // Function to additionally integrate the joint angle using gyro measurments
        float integrate_joint_state_from_pair(std::pair<int, int> imu_pair);

        // Function to avoid gradient descent NR for chain getting stuck
        void facilitate_chain_ik();

        // Function to compute soft chain IK
        void chain_ik();

        // Function to compute the leg joint state
        float compute_leg_joint();

        // Function to check if joint states are "publishable"
        bool states_publishable();

        // Function to fill joint states with est. values and publish
        void fill_and_publish();

        // Callback to imu accelerations topic
        void acc_callback(const nmmi_msgs::inertialSensorArray::ConstPtr &msg);

        // Callback to imu angular velocities topic
        void gyro_callback(const nmmi_msgs::inertialSensorArray::ConstPtr &msg);


        // Auxiliary funtion for deg2rad conversion
        inline double deg2rad (double degrees) {
            static const double k_pi_on_180 = 4.0 * atan (1.0) / 180.0;
            return degrees * k_pi_on_180;
        }

        // Auxiliary funtion for rad2deg conversion
        inline double rad2deg (double radians) {
            static const double k_180_on_pi = 180.0 / 4.0 * atan (1.0);
            return radians * k_180_on_pi;
        }

        // ROS variables
        ros::NodeHandle je_nh_;
        ros::Subscriber sub_imu_acc_;
        ros::Subscriber sub_imu_gyro_;
        ros::Publisher pub_js_;

        ros::AsyncSpinner async_spinner;

        // Initialization variables
        bool calibrated_ = false;

        // Transform listener and stamped transform for lookupTransform
        tf::TransformListener tf_listener_;
        tf::StampedTransform stamped_transform_;

        // qb readings and the mutex
        std::mutex imu_mutex_;                                      // Not used yet
        std::vector<nmmi_msgs::inertialSensor> imu_acc_;         // Raw acceleration msg from qb
        std::vector<nmmi_msgs::inertialSensor> imu_gyro_;        // Raw gyro msg from qb

        // Acceleration vectors
        std::vector<Eigen::Vector3d> acc_vec_0_;                    // Calibration acceleration
        std::vector<Eigen::Vector3d> acc_vec_;                      // Current acceleration
        std::vector<Eigen::Vector3d> acc_vec_olds_;                 // Previous acceleration
        std::vector<Eigen::Vector3d> acc_vec_raw_;                  // Raw unfiltered acceleration

        // Angular velocity vectors
        std::vector<Eigen::Vector3d> gyro_vec_;                     // Current acceleration
        ros::Time last_integration_time_;                           // For computing integration dt
        ros::Duration dt_;                                          // Gyro integration step

        // Low pass filter for accelerations
        std::vector<std::vector<std::shared_ptr<filters::FilterChain<double>>>> lp_filter_;

        // Complementary filter weights
        std::vector<double> gyro_weights_;

        // Joint variables
        std::vector<float> joint_values_;                           // Raw joint values
        std::vector<float> joint_values_old_;                       // Old raw joint values
        std::vector<float> joint_values_gyro_;                      // Raw joint values integrated from gyro
        std::vector<float> js_values_;                              // Joint states 
        std::vector<std::pair<float, float>> joint_limits_;         // Parsed from robot model
        sensor_msgs::JointState joint_states_;                      // Joint states message

        // Soft chain variables
        std::vector<std::string> chain_names_ = {"middle_chain", "left_chain", "right_chain"};
        KDL::Chain chain_chain_;                                    // Kinematic chain of the chain
        KDL::Chain ins_chain_;                                      // Kinematic chain around the chain to insertion
        KDL::JntArray chain_min_;                                   // Upper joint limits of chain
        KDL::JntArray chain_max_;                                   // Lower joint limits of chain
        KDL::JntArray q_chain_;                                     // Chain joint states
        KDL::JntArray q_ins_;                                       // Around chain joint states (4 joints)
        KDL::JntArray q_chain_lma_;                                 // Chain sigularity robust joint states
        KDL::Frame chain_ins_pose_;                                 // Pose of chain tip in chain base (desired)
        Eigen::Affine3d tip_to_ins_;                                // Transform from tip to chain insertion
        Eigen::VectorXd q_chain_base_;                              // Base config of chain to force upwards ik
        
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_ins_;      // FK solver front roll -> insertion
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;      // FK solver front roll -> chain tip
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;           // IK vel solver front roll -> chain tip
        boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;          // IK pos solver front roll -> chain tip
        boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_lma_solver_;            // IK LMA solver front roll -> chain tip
        Eigen::Matrix< double, 6, 1 > lma_weight_;
        
        // Leg variables (used if publish_leg_pose_ = true)
        double leg_js_ = 0.0;

        // For debug
        tf::TransformBroadcaster tf_broadcaster_;
        tf::Transform debug_transform_;
        KDL::Frame real_tip_pose_;                                  // Pose of chain tip in chain base (actual)
        KDL::Frame real_ins_pose_;                                  // Pose of chain tip in chain base (actual)

        // Constants
        int foot_id_;
        std::string foot_name_;
        std::string robot_name_;                                    // Foot name + id
        std::string pkg_path_;                                       // Path to this package (parsed later)
        std::string imu_topic_ = "/qb_class_imu/quat";
        std::string imu_topic_acc_ = "/qb_class_imu/acc";
        std::string imu_topic_gyro_ = "/qb_class_imu/gyro";

        // Parsed variables
        bool publish_leg_pose_ = false;                             // Estimate leg pose from imu and publish
        bool use_filter_ = false;                                   // Flag to use LP filter on measurments
        bool use_gyro_ = false;                                     // Flag to fuse gyro integration
        std::vector<std::pair<int, int>> joint_pairs_;              // Pairs of imu ids for each joint
        std::vector<std::string> joint_names_;                      // Names of each joint
        std::vector<std::string> joint_frame_names_;                // Names of the frames of each joint
        std::vector<std::pair<Eigen::Vector3d, 
            Eigen::Vector3d>> axes_pairs_;                          // For each imu pair, the axis of sensor frame aligned with joint axis
        XmlRpc::XmlRpcValue je_params_;                             // For nested params

};

}

#endif // JOINTS_ESTIMATOR_H
