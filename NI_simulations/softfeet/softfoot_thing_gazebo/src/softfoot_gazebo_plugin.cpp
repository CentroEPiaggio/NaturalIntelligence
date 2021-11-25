// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Custom include
#include "softfoot_thing_gazebo/softfoot_gazebo_plugin.hpp"

using namespace gazebo;

// Constructor
SoftFootGazeboPlugin::SoftFootGazeboPlugin(){

}

// Plugin Load Function
void SoftFootGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf){

    // Safety check
    if (model->GetJointCount() == 0) {
        ROS_FATAL_STREAM("Invalid number joints, SoftFoot Gazebo Plugin not loaded!\n");
        return;
    }

    // Save the model and link pointers for later use.
    this->sdf_ = sdf;
    this->model_ = model;
    this->world_ = model->GetWorld();
    this->engine_ = this->world_->Physics();

    // Check if model can be found
    if (!model) {
        ROS_FATAL_STREAM("Parent model is NULL! SoftFoot Gazebo Plugin cannot be loaded!\n");
        return;
    }

    // Get the namespace of the foot
    if (sdf->HasElement("robotNamespace")) {
        this->foot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        this->foot_namespace_ = model->GetName();
    }

    // Get the chain tip links
    this->link_l_ = model->GetLink(this->foot_namespace_ + "_left_chain_9_link");
    this->link_m_ = model->GetLink(this->foot_namespace_ + "_middle_chain_9_link");
    this->link_r_ = model->GetLink(this->foot_namespace_ + "_right_chain_9_link");

    // Get the back roll link
    this->link_des_ = model->GetLink(this->foot_namespace_ + "_back_roll_link");

    // Set the fixed transforms (TODO: change this with getting the transforms from SDF)
    this->roll_to_ins_.Set(-0.002, 0.000, -0.012,-1.676, 0.000, -1.571);
    this->chain_9_to_tip_.Set(0.000, 0.000, 0.013, 0.000, 0.000, 0.000);
    this->insertion_axis_ = ignition::math::Vector3d(0.0, 1.0, 0.0);

    // Adding joint between left_chain 9_link and back_roll_link
    this->insertion_joint_l_ = model->CreateJoint(this->foot_namespace_ + "_left_chain_attach_joint",
                                                 "revolute", this->link_l_, this->link_des_);
    this->insertion_joint_l_->Load(this->link_l_, this->link_des_, this->roll_to_ins_);
    this->insertion_joint_l_->Attach(this->link_l_, this->link_des_);
    this->insertion_joint_l_->SetModel(model);
    this->insertion_joint_l_->SetAxis(0, this->insertion_axis_);
    this->insertion_joint_l_->SetLowerLimit(0, this->deg2rad(-45.0));
    this->insertion_joint_l_->SetUpperLimit(0, this->deg2rad(45.0));
    this->insertion_joint_l_->SetEffortLimit(0, 1.0);
    this->insertion_joint_l_->SetVelocityLimit(0, 1.0);
    this->insertion_joint_l_->SetDamping(0, 0.0001);
    this->insertion_joint_l_->SetParam("friction", 0, 0.0001);

    // Adding joint between middle_chain 9_link and back_roll_link
    this->insertion_joint_m_ = model->CreateJoint(this->foot_namespace_ + "_middle_chain_attach_joint",
                                                 "revolute", this->link_m_, this->link_des_);
    this->insertion_joint_m_->Attach(this->link_m_, this->link_des_);
    this->insertion_joint_m_->Load(this->link_m_, this->link_des_, this->roll_to_ins_);
    this->insertion_joint_m_->SetModel(model);
    this->insertion_joint_m_->SetAxis(0, this->insertion_axis_);
    this->insertion_joint_m_->SetLowerLimit(0, this->deg2rad(-45.0));
    this->insertion_joint_m_->SetUpperLimit(0, this->deg2rad(45.0));
    this->insertion_joint_m_->SetEffortLimit(0, 1.0);
    this->insertion_joint_m_->SetVelocityLimit(0, 1.0);
    this->insertion_joint_m_->SetDamping(0, 0.0001);
    this->insertion_joint_m_->SetParam("friction", 0, 0.0001);

    // Adding joint between right_chain 9_link and back_roll_link
    this->insertion_joint_r_ = model->CreateJoint(this->foot_namespace_ + "_right_chain_attach_joint",
                                                 "revolute", this->link_r_, this->link_des_);
    this->insertion_joint_r_->Load(this->link_r_, this->link_des_, this->roll_to_ins_);
    this->insertion_joint_r_->Attach(this->link_r_, this->link_des_);
    this->insertion_joint_r_->SetModel(model);
    this->insertion_joint_r_->SetAxis(0, this->insertion_axis_);
    this->insertion_joint_r_->SetLowerLimit(0, this->deg2rad(-45.0));
    this->insertion_joint_r_->SetUpperLimit(0, this->deg2rad(45.0));
    this->insertion_joint_r_->SetEffortLimit(0, 1.0);
    this->insertion_joint_r_->SetVelocityLimit(0, 1.0);
    this->insertion_joint_r_->SetDamping(0, 0.0001);
    this->insertion_joint_r_->SetParam("friction", 0, 0.0001);

    // Getting arch joints from model and rest positions (For spring)
    this->fa_joint_ = model->GetJoint(this->foot_namespace_ + "_front_arch_joint");
    this->ba_joint_ = model->GetJoint(this->foot_namespace_ + "_back_arch_joint");
    this->rest_angle_ = this->fa_joint_->Position() + this->ba_joint_->Position();

    // Listen to the update event
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SoftFootGazeboPlugin::OnUpdate, this));

    // Plugin loaded successfully
    ROS_WARN_STREAM("Loaded plugin for " << this->foot_namespace_ << " simulation!\n");

}

// Plugin Update Function
void SoftFootGazeboPlugin::OnUpdate(){

    // Simulate the spring force on the arch links
    this->current_angle_ = this->fa_joint_->Position() + this->ba_joint_->Position();
    this->fa_joint_->SetForce(0, -this->spring_k_ * (this->current_angle_ - this->rest_angle_));
    this->ba_joint_->SetForce(0, -this->spring_k_ * (this->current_angle_ - this->rest_angle_));

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);
