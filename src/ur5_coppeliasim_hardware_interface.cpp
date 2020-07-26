#include "ur5_coppeliasim_driver/ur5_coppeliasim_hardware_interface.hpp"

UR5::UR5():joint_position_command_ ({0,0,0,0,0,0})
{
    nh_.reset(new ros::NodeHandle());
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, *nh_));
    loop_hz_=50;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    my_control_loop_ = nh_->createTimer(update_freq, &UR5::update, this);
}

UR5::~UR5() {
}

void UR5::init()
{
    // Load urdf file of robot
    std::string urdf_file;
    nh_->getParam("urdf_file",urdf_file);
    urdf::Model model;
    if(!model.initFile(urdf_file))
    {
        ROS_ERROR("Could not load urdf file!");
    }
    else
    {
        ROS_INFO("URDF file loaded.");
    }
    
    // Initialize buffer message.
    temp_joint_position_velocity_.data = {0,0,0,0,0,0,0,0,0,0,0,0};
    


    // Create joint_state_interface for all joints
    JointStateHandle jsShoulderPan("shoulder_pan_joint", &joint_position_[0], &joint_velocity_[0],&joint_effort_[0]);
    JointStateHandle jsShoulderLift("shoulder_lift_joint", &joint_position_[1], &joint_velocity_[1],&joint_effort_[1]);
    JointStateHandle jsElbow("elbow_joint", &joint_position_[2], &joint_velocity_[2],&joint_effort_[2]);
    JointStateHandle jsWrist1("wrist_1_joint", &joint_position_[3], &joint_velocity_[3],&joint_effort_[3]);
    JointStateHandle jsWrist2("wrist_2_joint", &joint_position_[4], &joint_velocity_[4],&joint_effort_[4]);
    JointStateHandle jsWrist3("wrist_3_joint", &joint_position_[5], &joint_velocity_[5],&joint_effort_[5]);

    joint_state_interface_.registerHandle(jsShoulderPan);
    joint_state_interface_.registerHandle(jsShoulderLift);
    joint_state_interface_.registerHandle(jsElbow);
    joint_state_interface_.registerHandle(jsWrist1);
    joint_state_interface_.registerHandle(jsWrist2);
    joint_state_interface_.registerHandle(jsWrist3);

    // Create position joint interface for all joints
    JointHandle jpShoulderPan(jsShoulderPan, &joint_position_command_[0]);
    JointHandle jpShoulderLift(jsShoulderLift, &joint_position_command_[1]);
    JointHandle jpElbow(jsElbow, &joint_position_command_[2]);
    JointHandle jpWrist1(jsWrist1, &joint_position_command_[3]);
    JointHandle jpWrist2(jsWrist2, &joint_position_command_[4]);
    JointHandle jpWrist3(jsWrist3, &joint_position_command_[5]);

    position_joint_interface_.registerHandle(jpShoulderPan); 
    position_joint_interface_.registerHandle(jpShoulderLift); 
    position_joint_interface_.registerHandle(jpElbow); 
    position_joint_interface_.registerHandle(jpWrist1); 
    position_joint_interface_.registerHandle(jpWrist2); 
    position_joint_interface_.registerHandle(jpWrist3); 

    // Creat joint limit for all joints
    joint_limits_interface::getJointLimits(model.getJoint("shoulder_pan_joint"),  limits);
    PositionJointSaturationHandle pjsShouldPan(jpShoulderPan,limits);
    positionJointSaturationInterface.registerHandle(pjsShouldPan);

    joint_limits_interface::getJointLimits(model.getJoint("shoulder_lift_joint"),  limits);
    PositionJointSaturationHandle pjsShouldLift(jpShoulderLift,limits);
    positionJointSaturationInterface.registerHandle(pjsShouldLift);

    joint_limits_interface::getJointLimits(model.getJoint("elbow_joint"),  limits);
    PositionJointSaturationHandle pjsElbow(jpElbow,limits);
    positionJointSaturationInterface.registerHandle(pjsElbow);

    joint_limits_interface::getJointLimits(model.getJoint("wrist_1_joint"),  limits);
    PositionJointSaturationHandle pjsWrist1(jpWrist1,limits);
    positionJointSaturationInterface.registerHandle(pjsWrist1);

    joint_limits_interface::getJointLimits(model.getJoint("wrist_2_joint"),  limits);
    PositionJointSaturationHandle pjsWrist2(jpWrist2,limits);
    positionJointSaturationInterface.registerHandle(pjsWrist2);

    joint_limits_interface::getJointLimits(model.getJoint("wrist_3_joint"),  limits);
    PositionJointSaturationHandle pjsWrist3(jpWrist3,limits);
    positionJointSaturationInterface.registerHandle(pjsWrist3);

    // Register all joints interfaces    
    hardware_interface::InterfaceManager::registerInterface(&joint_state_interface_);
    hardware_interface::InterfaceManager::registerInterface(&effort_joint_interface_);
    hardware_interface::InterfaceManager::registerInterface(&position_joint_interface_);
    hardware_interface::InterfaceManager::registerInterface(&effortJointSaturationInterface);
    hardware_interface::InterfaceManager::registerInterface(&positionJointSaturationInterface);    

    joint_state_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("ur5_joint_command",1);
    joint_state_sub_ = nh_->subscribe<std_msgs::Float32MultiArray>("ur5_joint_states", 1, &UR5::jointStates_callback, this);
}

/*
void UR5::startCoppeliasim()
{
    b0RemoteApi client("ur5_client","b0RemoteApi");
}

*/

void UR5::update(const ros::TimerEvent& e)
{
    last_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(::ros::Time::now(), last_time_);
    write(last_time_);
}

void UR5::read()
{
    mu_.lock();
    // Read joint_position, joint_velocity and effort from coppliasim
    joint_position_[0] = (double)temp_joint_position_velocity_.data[0] ;
    joint_position_[1] = (double)temp_joint_position_velocity_.data[1] ;
    joint_position_[2] = (double)temp_joint_position_velocity_.data[2] ;
    joint_position_[3] = (double)temp_joint_position_velocity_.data[3] ;
    joint_position_[4] = (double)temp_joint_position_velocity_.data[4] ;
    joint_position_[5] = (double)temp_joint_position_velocity_.data[5] ;

    joint_velocity_[0] = (double)temp_joint_position_velocity_.data[6];
    joint_velocity_[1] = (double)temp_joint_position_velocity_.data[7];
    joint_velocity_[2] = (double)temp_joint_position_velocity_.data[8];
    joint_velocity_[3] = (double)temp_joint_position_velocity_.data[9];
    joint_velocity_[4] = (double)temp_joint_position_velocity_.data[10];
    joint_velocity_[5] = (double)temp_joint_position_velocity_.data[11];
    mu_.unlock();
}

void UR5::write(ros::Duration last_time)
{
    // Write joint_position_command_ to coppeliasim
    effortJointSaturationInterface.enforceLimits(last_time);
    positionJointSaturationInterface.enforceLimits(last_time);

    std_msgs::Float32MultiArray msg;
    msg.data={ (float)joint_position_command_[0],
                            (float)joint_position_command_[1],
                            (float)joint_position_command_[2],
                            (float)joint_position_command_[3],
                            (float)joint_position_command_[4],
                            (float)joint_position_command_[5] };
    joint_state_pub_.publish(msg);
}

void UR5::jointStates_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    mu_.lock();
    temp_joint_position_velocity_ = *msg;
    mu_.unlock();
}

int main(int argc, char** argv)
{
    //Initialze the ROS node.
    ros::init(argc, argv, "ur5_hardware_inerface_node");
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(4); 
    
    UR5 robot;
    spinner.spin();
    return 0;
}