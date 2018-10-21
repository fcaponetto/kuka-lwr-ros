#ifndef ROS_WORKSPACE_POSITION_CONTROLLER_H
#define ROS_WORKSPACE_POSITION_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>

#include "controllers/joint_position.h"

namespace lwr_controllers
{
class JointController : public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
{
public:

    JointController();

    ~JointController();

    bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

private:

    int thrott_time;

    std::size_t         num_joints;

    ros::Subscriber     sub_command_joint_pos_;

    KDL::JntArray       tau_cmd_;   // torque
    KDL::JntArray       pos_cmd_;   // position
    KDL::JntArray       K_,K_cmd;   // stiffness
    KDL::JntArray       D_,D_cmd;   // damping
    KDL::JntArray       q_target_;

    boost::scoped_ptr<motion::CDDynamics>   joint_cddynamics;

    void command_joint_pos(const std_msgs::Float64MultiArray::ConstPtr &msg);

};

}
#endif //ROS_WORKSPACE_POSITION_CONTROLLER_H
