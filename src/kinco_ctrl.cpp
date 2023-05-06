#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>

#include <iostream>

#define LEFT_MOTOR_CAN_ID 0x601
#define RIGHT_MOTOR_CAN_ID 0x602

#define LEFT_MOTOR_PDO_ID 0x201
#define RIGHT_MOTOR_PDO_ID 0x202


#define DISTANCE_WHEELS  0.562
#define DIAMETER_WHEELS    0.169
#define PI 3.1415926
#define EXP_DOUBLE 1e-6

bool flag_send = false;
ros::Publisher roscan_send_message;

void setCANopenMotorToward(unsigned int ID, uint8_t toward)
{
    ROS_INFO("Ready to send can_message : motor toward");
    setlocale(LC_ALL, "");

    uint8_t transition[8];
    can_msgs::Frame can_frame;
    memset(&transition, 0, sizeof(transition));

    //只发送一个字节0x2F 发送两个字节0x2B 发送四个字节0x23 
    transition[0] = 0x2F;

    //设置电机方向的索引为607E低位在前高位在后
    transition[1] = 0x7E;
    transition[2] = 0x60;

    //子索引为00
    transition[3] = 0x00;

    //设置方向，低位在前高位在后

    transition[4] = toward;
    transition[5] = 0x00;
    transition[6] = 0x00;
    transition[7] = 0x00;

    can_frame.id = ID;

    can_frame.is_extended = true;
    can_frame.is_rtr = false;
    can_frame.is_error = false;
    can_frame.dlc = 8; //有效数据长度

    for (uint8_t i = 0; i < can_frame.dlc; i++)
    {

        can_frame.data[i] = transition[i];
    }

    roscan_send_message.publish(can_frame);

    ROS_INFO("send messages succeed : work model shift");
}

void setCANopenMotorSpeed(unsigned int ID, double speed)
{
    bool flag_sign = false;
    
    if(speed < 0)
    {
        flag_sign = true;
    }

    double rpm = fabs(speed);
    int encoder_resolution = 10000;
    int dec = rpm *512* encoder_resolution/1875;

    if(flag_sign == true)
    {
        dec = (~dec) + 1;
    }

    ROS_INFO("Ready to send can_message : motor speed  0x%X" , dec);
    setlocale(LC_ALL, "");

    uint8_t transition[8];
    can_msgs::Frame can_frame;
    memset(&transition, 0, sizeof(transition));

    //设置轮子转速
    unsigned int encode_tmp = 0xff;
    transition[0] = dec&encode_tmp;
    transition[1] = (dec>>8)&encode_tmp;
    transition[2] = (dec>>16)&encode_tmp;
    transition[3] = (dec>>24)&encode_tmp;
 
    can_frame.id = ID;

    can_frame.is_extended = true;
    can_frame.is_rtr = false;
    can_frame.is_error = false;
    can_frame.dlc = 4; //有效数据长度

    for (uint8_t i = 0; i < can_frame.dlc; i++)
    {

        can_frame.data[i] = transition[i];
    }

    roscan_send_message.publish(can_frame);

    ROS_INFO("send messages succeed : motor speed %X %X %X %X",transition[0],transition[1],transition[2],transition[3]);
}

void setCANopenMotorWorkMdl(unsigned int ID , uint8_t work_mdl)
{

    ROS_INFO("Ready to send can_message : work model shift");
    setlocale(LC_ALL, "");

    uint8_t transition[8];
    can_msgs::Frame can_frame;
    memset(&transition, 0, sizeof(transition));

    //只发送一个字节0x2F 发送两个字节0x2B 发送四个字节0x23
    transition[0] = 0x2F;

    //设置工作模式的索引为6060低位在前高位在后
    transition[1] = 0x60;
    transition[2] = 0x60;

    //子索引为00
    transition[3] = 0x00;

    //位置模式是0 速度模式是3
    transition[4] = work_mdl;

    //剩下全部补0即可
    transition[5] = 0x00;
    transition[6] = 0x00;
    transition[7] = 0x00;

    can_frame.id = ID;

    can_frame.is_extended = true;
    can_frame.is_rtr = false;
    can_frame.is_error = false;
    can_frame.dlc = 8; //有效数据长度

    for (uint8_t i = 0; i < can_frame.dlc; i++)
    {

        can_frame.data[i] = transition[i];
    }

    roscan_send_message.publish(can_frame);

    ROS_INFO("send messages succeed : work model shift");

}

void send_can_message()

{
    ros::NodeHandle roscan("~");

    ros::Publisher roscan_send_message;

    roscan_send_message = roscan.advertise<can_msgs::Frame>("/sent_messages", 100);

    while (ros::ok())
    {

        ROS_INFO("start send can_message");

        setlocale(LC_ALL, "");

        uint8_t transition[8]; 

        can_msgs::Frame can_frame;

        memset(&transition, 0, sizeof(transition)); 

        transition[0] = 0x0a;

        transition[1] = 0x0b;

        transition[2] = 0x20;

        transition[3] = 0x00;

        transition[4] = 0xe8;

        transition[5] = 0x03;

        transition[6] = 0x00;

        transition[7] = 0x00;

        can_frame.id = 0x601; //需要修改

        can_frame.is_extended = true;

        can_frame.is_rtr = false;

        can_frame.is_error = false;

        can_frame.dlc = 8; //有效数据长度

        for (uint8_t i = 0; i < can_frame.dlc; i++)
        {

            can_frame.data[i] = transition[i];
        }

        roscan_send_message.publish(can_frame);

        ROS_INFO("send messages succeed!");

        //ros::Duration(0.3).sleep();

        ros::spinOnce();
    }
}


void WorkMdlCallback(const std_msgs::Int8 & msg)
{
    ROS_INFO("work model setting : %d" ,msg.data);
    setCANopenMotorWorkMdl(LEFT_MOTOR_CAN_ID,static_cast<uint8_t>(msg.data));
    setCANopenMotorWorkMdl(RIGHT_MOTOR_CAN_ID,static_cast<uint8_t>(msg.data));
    //send_can_message();
}

void MotorSpeedCallback(const std_msgs::Float32MultiArray & msg)
{
    ROS_INFO("motor speed setting : %f" ,msg.data[1]);
    //如果是左轮
    unsigned int id;
    uint8_t dir;

    if(msg.data.size() == 2)
    {
        if(msg.data[0] < 0)
        {
            ROS_INFO("motor speed setting : left wheel");
            id = LEFT_MOTOR_PDO_ID;
        }
        //如果是右轮
        else if(msg.data[0] > 0)
        {
            ROS_INFO("motor speed setting : right wheel");
            id = RIGHT_MOTOR_PDO_ID;
        }
        setCANopenMotorSpeed(id,msg.data[1]);
    }
    else if(msg.data.size() == 3)
    {
        id = LEFT_MOTOR_PDO_ID;
        setCANopenMotorSpeed(id,msg.data[1]);
        id = RIGHT_MOTOR_PDO_ID;
        setCANopenMotorSpeed(id,msg.data[2]);
    }

}

void cmdVelCallback(const geometry_msgs::Twist & msg)
{
//    vel_left = (curr_cmd.lin - curr_cmd.ang * L/ 2.0) / left_wheel_radius;
//    vel_right = (curr_cmd.lin + curr_cmd.ang * L/ 2.0) / right_wheel_radius;
//    m/s
    double v_line_x = msg.linear.x;
    double v_angr_z = msg.angular.z;
    double radius = DIAMETER_WHEELS / 2;
    double dist_w = DISTANCE_WHEELS;

    double vel_left = (v_line_x - v_angr_z * dist_w / 2.0) / radius;  // m/s
    double vel_right = (v_line_x + v_angr_z * dist_w / 2.0) / radius;

    double speed_left = vel_left * 2 * PI;     // rad/s
    double speed_right = vel_right * 2 * PI;

    speed_left *= 9.55; //rpm
    speed_right *= 9.55;

    unsigned int id;
    id = LEFT_MOTOR_PDO_ID;
    setCANopenMotorSpeed(id,speed_left);
    id = RIGHT_MOTOR_PDO_ID;
    setCANopenMotorSpeed(id,-speed_right);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "can_sender");
    ros::NodeHandle n("~");

    ros::Subscriber work_mdl_sub = n.subscribe("/work_mdl_shift" , 60 , WorkMdlCallback);
    ros::Subscriber motor_speed_sub = n.subscribe("/motor_speed" , 60 , MotorSpeedCallback);

    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel" , 60 , cmdVelCallback);

    roscan_send_message = n.advertise<can_msgs::Frame>("/sent_messages", 100);

    ros::spin();

}
