#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopRobot
{
public:
    TeleopRobot();
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    ros::NodeHandle nh; // //! TODO Make nh private ("~") and move nh to main loop, outside TeleopRobot class;
    
    int l_axis, a_axis, l_scale_axis, a_scale_axis, const_l_button, const_l_up, const_l_down, on_off_button;
    
    double l_scale         = 1;
    double a_scale         = 1; 
    double const_l_scale   = 1;
    bool   start_pressed   = false;
    bool   const_l_pressed = false;
    
    ros::Publisher  vel_pub;
    ros::Subscriber joy_sub;
};

TeleopRobot::TeleopRobot():
    l_axis(1),
    a_axis(3),
    l_scale_axis(7),
    a_scale_axis(6),
    const_l_button(3),
    const_l_up(1),
    const_l_down(2),
    on_off_button(7)
{
    if(nh.hasParam("/joystick")){
        nh.param("start",    on_off_button);              //Turn Teleop On/Off         = start     :            = 0/1
        nh.param("ljoy_ud",  l_axis, l_axis);             //Linear axis                = left joy  : up/down    = +1/-1
        nh.param("rjoy_lr",  a_axis, a_axis);             //Angular axis               = right joy : left/right = +1/-1      
        nh.param("lpad_ud",  l_scale_axis, l_scale_axis); //Linear Scale Axis          = left pad  : up/down    = +1/-1
        nh.param("lpad_lr",  a_scale_axis, a_scale_axis); //Angular Scale Axis         = left pad  : left/right = +1/-1
        nh.param("buttonY",  const_l_button);             //Constant Linear On/Off     = triangle  :            = 0/1
        nh.param("buttonB",  const_l_up);                 //Constant Linear Scale Up   = triangle  :            = 0/1
        nh.param("buttonX",  const_l_down);               //Constant Linear Scale Down = triangle  :            = 0/1
    }
        
    vel_pub = nh.advertise<geometry_msgs::Twist>("/joy_node/cmd_vel", 1);
    
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRobot::joyCallback, this);
        
}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    
    // scale up/down linear/angular speed by 10%
    l_scale += joy->axes[l_scale_axis]*0.10;
    a_scale -= joy->axes[a_scale_axis]*0.10;  //direction reversed  
    
    // scale up/down const linear speeds by 10%
    const_l_scale  += (joy->buttons[const_l_up] - joy->buttons[const_l_down])*0.10;
    
    if(joy->buttons[on_off_button] == 1)
    {
        start_pressed = !start_pressed;
    }
    
    if(joy->buttons[const_l_button] == 1)
    {
        const_l_pressed = !const_l_pressed;
    }
    
    if(!start_pressed) //Teleop off
    {
        twist.linear.x  = 0.0;
        twist.angular.z = 0.0;
    }
    else //Teleop on
    {
        if(!const_l_pressed) //Normal operation
        {
            twist.linear.x  = l_scale*joy->axes[l_axis];
            twist.angular.z = a_scale*joy->axes[a_axis];
        }
        else //Const linear speed
        {
            twist.linear.x  = const_l_scale;
            twist.angular.z = a_scale*joy->axes[a_axis];
        }
    }
    
    vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_teleop");
    TeleopRobot teleop_robot;
    ROS_INFO("[JOY] Six-Axis Joystick driver initialized");
    ros::spin();
}