/*
==========
ROS node for control using a SixAxis Joystick. The ROS joy package is needed.
This code implements the following functionality:
  > Configure joystick / map keys
  > Subscribe to joystick input from Joy package
  > Implement teleop functionality provided by teleop_twist_keyboard
  > Implement start/stop functionality for the robot
  > Implement start/stop recording functionality for DL/Rosbags
  > Publish twist message
by Aditya Kamath
adityakamath.github.io
github.com/adityakamath
==========
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>

class TeleopRobot
{
public:
    TeleopRobot();
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    ros::NodeHandle nh;
    
    int l_axis, a_axis, l_scale_axis, a_scale_axis, const_l_button, const_l_up, const_l_down, on_off_button, select_button, max_modes;
    
    double l_scale         = 1.30; //calibrared values, will change for each robot platform
    double a_scale         = 1.90; //calibrated values, will change for each robot platform
    double const_l_scale   = 1;
    bool   start_pressed   = false;
    bool   const_l_pressed = false;
    int    select_pressed  = 0;
    
    ros::Publisher  vel_pub;
    ros::Subscriber joy_sub;
    ros::Publisher  mode_pub;
};

TeleopRobot::TeleopRobot():
    l_axis(1), //default values
    a_axis(3),
    l_scale_axis(7),
    a_scale_axis(6),
    const_l_button(3),
    const_l_up(1),
    const_l_down(2),
    on_off_button(7),
    select_button(6),
    max_modes(3),
{
    if(nh.hasParam("/joystick")){
        nh.param("start",    on_off_button);              //Turn Teleop On/Off         = start     :            = 0/1
        nh.param("ljoy_ud",  l_axis, l_axis);             //Linear axis                = left joy  : up/down    = +1/-1
        nh.param("rjoy_lr",  a_axis, a_axis);             //Angular axis               = right joy : left/right = +1/-1      
        nh.param("lpad_ud",  l_scale_axis, l_scale_axis); //Linear Scale Axis          = left pad  : up/down    = +1/-1
        nh.param("lpad_lr",  a_scale_axis, a_scale_axis); //Angular Scale Axis         = left pad  : left/right = +1/-1
        nh.param("buttonY",  const_l_button);             //Constant Linear On/Off     = Y         :            = 0/1
        nh.param("buttonB",  const_l_up);                 //Constant Linear Scale Up   = B         :            = 0/1
        nh.param("buttonX",  const_l_down);               //Constant Linear Scale Down = X         :            = 0/1
        nh.param("select",   select_button);              //Select modes               = select    :            = 0/1
        nh.param("nmodes",   max_modes);                  //Maximum number of modes    = nmodes    : default    = 3
        ROS_INFO("[JOY] Loaded config from parameter server");
    }
        
    vel_pub  = nh.advertise<geometry_msgs::Twist>("/joy_node/cmd_vel", 1);
    
    joy_sub  = nh.subscribe<sensor_msgs::Joy>("joy", 5, &TeleopRobot::joyCallback, this);
        
    mode_pub = nh.advertise<std_msgs::Int8>("/joy_node/mode", 1);
        
}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    
    // scale up/down linear/angular speed by 10%
    l_scale += joy->axes[l_scale_axis]*0.10;
    a_scale -= joy->axes[a_scale_axis]*0.10;  //direction reversed 
    
    // scale up/down const linear speeds by 10%
    const_l_scale  += (joy->buttons[const_l_up] - joy->buttons[const_l_down])*0.10;
    
    // set on/off flag
    if(joy->buttons[on_off_button] == 1){
        start_pressed = !start_pressed;
    }
    
    // set select mode flag
    if(joy->buttons[select_button] == 1 && start_pressed){
        if(select_pressed < max_modes){
            select_pressed++;
        }
        else{
            select_pressed = 1;
        }
    }
    
    // select constant linear vel flag
    if(joy->buttons[const_l_button] == 1){
        const_l_pressed = !const_l_pressed;
    }
    
    // publish velocities
    if(!start_pressed){ //Teleop off
        twist.linear.x  = 0.0;
        twist.angular.z = 0.0;
    }
    else{ //Teleop on
        if(!const_l_pressed){ //Normal operation
            twist.linear.x  = l_scale*joy->axes[l_axis];
            ROS_DEBUG("l_scale = %f", l_scale);
            twist.angular.z = a_scale*joy->axes[a_axis];
            ROS_DEBUG("a_scale = %f", a_scale);
        }
        else{ //Const linear speed
            twist.linear.x  = const_l_scale;
            twist.angular.z = a_scale*joy->axes[a_axis];
        }
    }
    vel_pub.publish(twist);
    
    // publish mode
    std_msgs::Int8 mode_msg;
    if(!start_pressed){
        mode_msg.data = 0;
        ROS_DEBUG("[JOY] Selected mode: Joystick OFF");
    }
    else{
        mode_msg.data = select_pressed;
        ROS_DEBUG("[JOY] Selected mode: %d", mode_msg.data);
    }
    mode_pub.publish(mode_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_teleop");
    TeleopRobot teleop_robot;
    ROS_INFO("[JOY] Six-Axis Joystick driver initialized");
    ros::spin();
}