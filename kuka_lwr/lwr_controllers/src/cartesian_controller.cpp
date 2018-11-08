#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <pluginlib/class_list_macros.h>
#include "lwr_controllers/cartesian_controller.h"

namespace lwr_controllers
{

CartesianController::CartesianController():
    thrott_time(1),
    num_joints(7)
{

}

CartesianController::~CartesianController()
{

}

bool CartesianController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
{
    ROS_INFO(" ~~~~~~~~~~~~~~~~~~~ INIT CARTESIAN CONTROLLER ~~~~~~~~~~~~~~~~~~~~");

    KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n);

    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());

//    K_pos_.resize(kdl_chain_.getNrOfJoints());
//    K_vel_.resize(kdl_chain_.getNrOfJoints());

    K_cmd.resize(kdl_chain_.getNrOfJoints());
    D_cmd.resize(kdl_chain_.getNrOfJoints());
    pos_cmd_.resize(kdl_chain_.getNrOfJoints());
    tau_cmd_.resize(kdl_chain_.getNrOfJoints());

    joint_msr_.resize(kdl_chain_.getNrOfJoints());

    J_.resize(kdl_chain_.getNrOfJoints());
    ROS_INFO("CartesianController::init finished initialise [Jacobian]!");

    /// Solvers (Kinematics, etc...)
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fk_vel_solver_.reset(       new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
    ROS_INFO("CartesianController::init finished initialise [kinematic solvers]!");

    // subscriber
    sub_command_pose_ = n.subscribe("command_pos",1, &CartesianController::command_cart_pos,    this);
    ROS_INFO("CartesianController::init finished initialise [controllers]!");
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
    ROS_INFO("CartesianController::init finished initialise [joint position values]!");

    ROS_INFO("CartesianController initialised!");
    return true;
}

void CartesianController::starting(const ros::Time &time)
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
    ROS_INFO(" CartesianController::starting finished!");
}
static bool cmd_flag_ = false;
void CartesianController::update(const ros::Time &time, const ros::Duration &period)
{
    // get measured joint positions and velocity
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
        // get the joint values
        joint_msr_.q(i)           = joint_handles_[i].getPosition();
        joint_msr_.qdot(i)        = joint_handles_[i].getVelocity();
    }


    if(cmd_flag_) {
        /// forward kinematic x_msr = fkine ( joint_msr)
        fk_pos_solver_->JntToCart(joint_msr_.q, x_msr_);

        /// getting quaternion from rotation matrix
        x_msr_.M.GetQuaternion(quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2), quat_curr_.a);
        x_des_.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);

        /// compute desired values
//    ROS_INFO_STREAM_THROTTLE(thrott_time,"ctrl_mode ===> CARTESIAN_POSITION");

        /// from joint values, create the Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_.q, J_);
        /// computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        skew_symmetric(quat_des_.v, skew_);

        /// [quat_des] * quat_curr
        for (int i = 0; i < skew_.rows(); i++) {
            v_temp_(i) = 0.0;
            for (int k = 0; k < skew_.cols(); k++)
                v_temp_(i) += skew_(i, k) * (quat_curr_.v(k));
        }

        // end-effector orientation error
        x_err_.vel = x_des_.p - x_msr_.p; // TODO check
        x_err_.rot = quat_curr_.a * quat_des_.v - quat_des_.a * quat_curr_.v - v_temp_;

        // computing q_dot
        for (int i = 0; i < J_pinv_.rows(); i++) {
            joint_des_.qdot(i) = 0.0;
            for (int k = 0; k < J_pinv_.cols(); k++)
                joint_des_.qdot(i) += J_pinv_(i, k) * x_err_(k); //removed scaling factor of .7
        }

        // integrating q_dot -> getting q (Euler method)
        for (std::size_t i = 0; i < static_cast<std::size_t>(joint_des_.q.data.size()); i++) {
            joint_des_.q(i) += period.toSec() * joint_des_.qdot(i);
        }

        if (Equal(x_msr_, x_des_, 0.005)) {
            ROS_INFO("On target");
            cmd_flag_ = false;
        }
    }



    ROS_INFO_STREAM_THROTTLE(1,"Joint msr "<< joint_msr_.q.data(0) << " " << joint_msr_.q.data(1) << " " <<
                                            joint_msr_.q.data(2) << " " << joint_msr_.q.data(3) << " " <<
                                            joint_msr_.q.data(4) << " " <<joint_msr_.q.data(5) << " " <<
                                            joint_msr_.q.data(6));
    ROS_INFO_STREAM_THROTTLE(1,"Joint des "<< joint_des_.q.data(0) << " " << joint_des_.q.data(1) << " " <<
                                            joint_des_.q.data(2) << " " << joint_des_.q.data(3) << " " <<
                                            joint_des_.q.data(4) << " " <<joint_des_.q.data(5) << " " <<
                                            joint_des_.q.data(6) << "\n");
    ROS_INFO_STREAM_THROTTLE(1,"cart error: " << x_err_.vel(0) << " " << x_err_.vel(1) << " " << x_err_.vel(2)); // TODO check

    ROS_INFO_STREAM_THROTTLE(1, "Stiffness " << K_(0) << " " << K_(1) << " " << K_(2) << " " << K_(3) << " " << K_(4) << " " << K_(5) << " " << K_(6));


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

void CartesianController::command_cart_pos(const geometry_msgs::PoseConstPtr &msg)
{
    ROS_INFO("~~~~~~ INSIDE CARTESIAN CONTROLLER CALLBACK ~~~~~~~~");

    KDL::Frame frame_des_(
            KDL::Rotation::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w),
            KDL::Vector(msg->position.x,msg->position.y,msg->position.z));

    x_des_      = frame_des_;
    cmd_flag_ = true;
}
}
PLUGINLIB_EXPORT_CLASS( lwr_controllers::CartesianController, controller_interface::ControllerBase)