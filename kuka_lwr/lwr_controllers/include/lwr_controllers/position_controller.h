#ifndef ROS_WORKSPACE_POSITION_CONTROLLER_H
#define ROS_WORKSPACE_POSITION_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <ros/ros.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>

#include "controllers/joint_position.h"

namespace lwr_controllers
{
class PositionController : public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
{
public:

    PositionController();

    ~PositionController();

    bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

private:

    int thrott_time;

    /* Controll Manager */
    controllers::Change_ctrl_mode                          ctrl_mode;

    boost::scoped_ptr<controllers::Joint_position>         joint_position_controller;

    KDL::JntArray       tau_cmd_;
    KDL::JntArray       pos_cmd_;
    KDL::JntArray       K_, D_,K_cmd,D_cmd, K_pos_, K_vel_;

    KDL::Frame          x_msr_;         // measured end-effector position
    KDL::FrameVel       x_dt_msr_;      // measured end-effector velocity
    KDL::Frame          x_des_;         // desired pose
    KDL::Jacobian       J_;             // Jacobian
    KDL::JntArrayVel    joint_vel_msr_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver>         jnt_to_jac_solver_;

    boost::shared_ptr<KDL::ChainFkSolverPos_recursive>  fk_pos_solver_;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive>  fk_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_pos_solver_;

};

}
#endif //ROS_WORKSPACE_POSITION_CONTROLLER_H
