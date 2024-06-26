#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include "audio_player/audio_player.h"

class JoyController {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber death_judge_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher shoot_pub_;
    sensor_msgs::Joy last_joy_;
    bool has_joy_msg_ = false;
    bool is_dead_ = false;

    double max_linear_vel_;
    double max_angular_vel_;

    std::unique_ptr<AudioPlayer> audio_player_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
        if (is_dead_) {
            return;
        }
        geometry_msgs::Twist twist;
        twist.linear.x = msg->axes[1] * max_linear_vel_;
        twist.angular.z = msg->axes[2] * max_angular_vel_;
        cmd_vel_pub_.publish(twist);

        if (has_joy_msg_ && msg->buttons[5] == 1 && last_joy_.buttons[5] == 0) {
            std_msgs::Bool shoot;
            shoot.data = true;
            shoot_pub_.publish(shoot);
        }
        last_joy_ = *msg;
        has_joy_msg_ = true;
    }

    void deathJudgeCallback(const std_msgs::Bool::ConstPtr &msg) {
        if (msg->data) {
            is_dead_ = true;
            geometry_msgs::Twist twist;
            twist.linear.x = 0;
            twist.angular.z = 0;
            cmd_vel_pub_.publish(twist);
        }
    }

  public:
    JoyController() : nh_(), pnh_("~") {
        joy_sub_ = nh_.subscribe("joy", 10, &JoyController::joyCallback, this);
        death_judge_sub_ = nh_.subscribe("death_judge", 10, &JoyController::deathJudgeCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1, true);
        shoot_pub_ = nh_.advertise<std_msgs::Bool>("shoot", 1, true);

        max_linear_vel_ = pnh_.param<double>("max_linear_vel", 0.5);
        max_angular_vel_ = pnh_.param<double>("max_angular_vel", 1.0);
    };
    ~JoyController(){};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_contoroller");
    JoyController joy_controller;
    ros::spin();
    return 0;
}
