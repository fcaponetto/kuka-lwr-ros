#include "lwr_controllers/joint_controller.h"
#include <pluginlib/class_list_macros.h>

namespace lwr_controllers
{

JointController::JointController() :
        thrott_time(1),
        num_joints(7)
{}

JointController::~JointController() {}

bool JointController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
{
    ROS_INFO(" ~~~~~~~~~~~~~~~~~~~ INIT Joint CONTROLLER ~~~~~~~~~~~~~~~~~~~~");

    KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n);

    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());

    K_cmd.resize(kdl_chain_.getNrOfJoints());
    D_cmd.resize(kdl_chain_.getNrOfJoints());

    pos_cmd_.resize(kdl_chain_.getNrOfJoints());
    tau_cmd_.resize(kdl_chain_.getNrOfJoints());

    joint_msr_.resize(kdl_chain_.getNrOfJoints());

    joint_cddynamics.reset(new motion::CDDynamics(7,1e-6,1));

    motion::Vector velLimits(7);
    for(std::size_t i = 0; i < 7; i++){
        velLimits(i)  = 0.25; // x ms^-1
    }
    joint_cddynamics->SetVelocityLimits(velLimits);

    // subscriber
    sub_command_joint_pos_ = n.subscribe("command_joint_pos",1, &JointController::command_joint_pos,    this);
    ROS_INFO("JointControllers::init finished initialise [controllers]!");
    q_target_.resize(num_joints);

    // get joint positions
    for(std::size_t i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_.q(i)         = joint_handles_[i].getPosition();
        joint_msr_.qdot(i)      = joint_handles_[i].getVelocity();
        joint_msr_.qdotdot(i)   = joint_handles_[i].getAcceleration();
        joint_des_.q(i)         = joint_msr_.q(i);
        joint_des_.qdot(i)      = 0;
        pos_cmd_(i)             = joint_des_.q(i);

    }
    ROS_INFO("JointControllers::init finished initialise [joint position values]!");

    ROS_INFO("Joint_controllers initialised!");
    return true;

}

void JointController::starting(const ros::Time &time)
{
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_msr_.q(i)         = joint_handles_[i].getPosition();
        joint_msr_.qdot(i)      = joint_handles_[i].getVelocity();
        joint_msr_.qdotdot(i)   = joint_handles_[i].getAcceleration();

        joint_des_.q(i)    = joint_msr_.q(i);
        pos_cmd_(i)        = joint_des_.q(i);
        tau_cmd_(i)        = 0;
        K_cmd(i)           = 0;
        D_cmd(i)           = 0.7;

        joint_handles_[i].setCommandPosition(pos_cmd_(i));
        joint_handles_[i].setCommandTorque(tau_cmd_(i));
        joint_handles_[i].setCommandStiffness(K_cmd(i));
        joint_handles_[i].setCommandDamping(D_cmd(i));
    }
    ROS_INFO(" JointControllers::starting finished!");
}

void JointController::update(const ros::Time &time, const ros::Duration &period)
{
    /// get measured joint positions and velocity
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
        // get the joint values
        joint_msr_.q(i)           = joint_handles_[i].getPosition();
        joint_msr_.qdot(i)        = joint_handles_[i].getVelocity();
    }

    /// compute desired values
    ROS_INFO_STREAM_THROTTLE(thrott_time,"ctrl_mode ===> JOINT_POSITION");
    joint_cddynamics->SetDt(period.toSec());
    joint_cddynamics->SetTarget(q_target_.data);
    joint_cddynamics->Update();
    joint_cddynamics->GetState(joint_des_.q.data);

    /// set desidered position
    for(size_t i=0; i<joint_handles_.size(); i++) {
        K_cmd(i)         = K_(i);
        D_cmd(i)         = D_(i);
        tau_cmd_(i)      = 0;
        pos_cmd_(i)      = joint_des_.q(i);
    }

    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_handles_[i].setCommandPosition(pos_cmd_(i));
        joint_handles_[i].setCommandTorque(tau_cmd_(i));
        joint_handles_[i].setCommandStiffness(K_cmd(i));
        joint_handles_[i].setCommandDamping(D_cmd(i));
    }
}

void JointController::command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg){
    ROS_INFO("INSIDE JOINT CONTROLLER CALLBACK");


    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", num_joints);
    }
    else if (msg->data.size() != num_joints) {
        ROS_INFO("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        ROS_INFO("JOINT POSITION CTRL SETTING TARGET");
        for (std::size_t j = 0; j < num_joints; ++j){
            q_target_(j)     = msg->data[j];
        }
        std::cout << "target joints: " <<  q_target_.data << std::endl;
    }

}
}
PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointController, controller_interface::ControllerBase)