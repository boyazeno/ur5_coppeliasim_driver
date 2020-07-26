#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
//#include "ur5_coppeliasim_driver/b0RemoteApi.h"
typedef hardware_interface::JointStateHandle JointStateHandle; 
typedef hardware_interface::JointHandle  JointHandle;
typedef joint_limits_interface::EffortJointSaturationHandle EffortJointSaturationHandle;
typedef joint_limits_interface::PositionJointSaturationHandle PositionJointSaturationHandle;

class UR5 : public hardware_interface::RobotHW
{
    // reference to https://github.com/SlateRobotics/tr1_hardware_interface
    public:
        UR5();
        ~UR5();
        void init();
        //void startCoppeliasim();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration last_time);

    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        joint_limits_interface::JointLimits limits;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;

        ros::Publisher joint_state_pub_;
        ros::Subscriber joint_state_sub_;

        double joint_position_[6];
        double joint_velocity_[6];
        double joint_effort_[6];
        double joint_effort_command_[6];
        double joint_position_command_[6];

        std_msgs::Float32MultiArray temp_joint_position_velocity_;

        ros::NodeHandlePtr nh_;
        ros::Timer my_control_loop_;
        ros::Duration last_time_;
        double loop_hz_;
        std::mutex mu_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    private:
        void jointStates_callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
};
