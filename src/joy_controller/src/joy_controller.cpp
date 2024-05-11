#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

class JoyController {
  private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher cmd_vel_pub_;

    double linear_scale_ = 0.1;
    double angular_scale_ = 0.5;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
        geometry_msgs::Twist twist;
        twist.linear.x = msg->axes[1] * linear_scale_;
        twist.angular.z = msg->axes[2] * angular_scale_;
        cmd_vel_pub_.publish(twist);
    }

  public:
    JoyController() : nh_() {
        joy_sub_ = nh_.subscribe("/joy", 10, &JoyController::joyCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1, true);
    };
    ~JoyController(){};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_contoroller");
    JoyController joy_controller;
    ros::spin();
    return 0;
}
