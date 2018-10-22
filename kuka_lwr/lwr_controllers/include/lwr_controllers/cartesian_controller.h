#ifndef _CARTESIAN_CONTROLLER_H_
#define _CARTESIAN_CONTROLLER_H_

#include <ros/ros.h>
#include "KinematicChainControllerBase.h"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>

#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/Pose.h>

namespace lwr_controllers
{

class CartesianController : public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
{

public:
    CartesianController();

    ~CartesianController();

    bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

private:

    int thrott_time;

    std::size_t         num_joints;

    ros::Subscriber     sub_command_pose_;

    KDL::JntArray       tau_cmd_;   // torque
    KDL::JntArray       pos_cmd_;   // position
    KDL::JntArray       K_,K_cmd;   // stiffness
    KDL::JntArray       D_,D_cmd;   // damping
    KDL::JntArray       q_target_;

    KDL::Frame          x_msr_;         // measured end-effector position
    KDL::Jacobian       J_;             // Jacobian
    KDL::Frame          x_des_;         // desired pose

    KDL::Vector         v_temp_;

    KDL::Twist          x_err_;

    Eigen::MatrixXd     J_pinv_;        // Jacobian pseudo inverse
    Eigen::Matrix<double,3,3>   skew_;  // skew-symmetric matrix

    struct quaternion_
    {
        KDL::Vector v;
        double a;
    } quat_curr_, quat_des_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver>         jnt_to_jac_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive>  fk_pos_solver_;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive>  fk_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL>      ik_pos_solver_;

    void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);
};
}
#endif //_CARTESIAN_CONTROLLER_H_
