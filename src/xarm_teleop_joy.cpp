#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopXArm
{
public:
  TeleopXArm();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int x_, y_,z_,a_,b_,c_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopXArm::TeleopXArm():
  x_(0),
  y_(1),
  z_(4),
  a_(6),
  b_(7),
  c_(3)
{

  nh_.param("x_axis", x_, x_);
  nh_.param("y_axis", y_, y_);
  nh_.param("z_axis", z_, z_);
  nh_.param("a_axis", a_, a_);
  nh_.param("b_axis", b_, b_);
  nh_.param("c_axis", c_, c_);

  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("TeleopXArm/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopXArm::joyCallback, this);

}

void TeleopXArm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.x = l_scale_*joy->axes[x_];
  twist.linear.y = l_scale_*joy->axes[y_];
  twist.linear.z = l_scale_*joy->axes[z_];
  twist.angular.x = a_scale_*joy->axes[a_];
  twist.angular.y = a_scale_*joy->axes[b_];
  twist.angular.z = a_scale_*joy->axes[c_];

  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_xarm");
  TeleopXArm teleop_xarm;
  ros::spin();
}