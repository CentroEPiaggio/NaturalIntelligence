#ifndef SOFTFOOT_GAZEBO_PLUGIN_H_
#define SOFTFOOT_GAZEBO_PLUGIN_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Gazebo Math
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// ROS
#include <ros/ros.h>

namespace gazebo
{
    // A plugin to simulate the softfoot in gazebo.
    class SoftFootGazeboPlugin : public ModelPlugin {
        
        public:

            // Constructor
            SoftFootGazeboPlugin();

            // Plugin Load Function
            virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

            // Plugin Update Function
            virtual void OnUpdate();


        private:

            // Auxiliary funtion for deg2rad conversion
            inline double deg2rad (double degrees) {
                static const double k_pi_on_180 = 4.0 * atan (1.0) / 180.0;
                return degrees * k_pi_on_180;
            }

            // Pointers to physics variables
            sdf::ElementPtr sdf_;
            physics::ModelPtr model_;
            physics::WorldPtr world_;
            physics::PhysicsEnginePtr engine_;


            // Insertion joints
            physics::JointPtr insertion_joint_l_;
            physics::JointPtr insertion_joint_m_;
            physics::JointPtr insertion_joint_r_;
            ignition::math::Vector3d insertion_axis_;

            // Arch joints and spring constant
            physics::JointPtr fa_joint_;
            physics::JointPtr ba_joint_;
            double current_angle_;
            double rest_angle_ = 0.0;
            double spring_k_ = 5.0;

            // Pointers to the links to join for chains
            physics::LinkPtr link_l_;                   // left_chain_9_link
            physics::LinkPtr link_m_;                   // middle_chain_9_link
            physics::LinkPtr link_r_;                   // right_chain_9_link
            physics::LinkPtr link_des_;                 // back_roll_link

            // Fixed transforms
            ignition::math::Pose3d roll_to_ins_;
            ignition::math::Pose3d chain_9_to_tip_;

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection_;

            // Namespace of the foot
            std::string foot_namespace_;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_
