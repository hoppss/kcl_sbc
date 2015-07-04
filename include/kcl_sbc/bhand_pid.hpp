#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <bhand_controller/State.h>
#include <bhand_controller/SetControlMode.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <kcl_sbc/PID_paramsConfig.h>
#include <sr_grasp_msgs/KCL_ContactStateStamped.h>

#define PI 3.14159265

class BHand_PID{
public:
    BHand_PID();
    ros::NodeHandle nh;
    void reconf_callback(kcl_sbc::PID_paramsConfig &config, uint32_t level);
    void ft_cb1(const sr_grasp_msgs::KCL_ContactStateStamped &msg);
    void ft_cb2(const sr_grasp_msgs::KCL_ContactStateStamped &msg);
    void js_cb(const sensor_msgs::JointState &msg);
    double controller(double ref,double input);
    ros::Publisher jcom_pub,pos_con_pub,com_pub;
    double command1,command2;
    sensor_msgs::JointState joint_command,cur_joints;
    std::string control_topic;
    int motor_index;

protected:
    double P,I,D,ref,prev,ie,a_windup,deadband;

};
