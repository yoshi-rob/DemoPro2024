#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include <future>
#include <thread>
#include "audio_player/audio_player.h"

class JoyController {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber joy_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher shoot_pub_;
    sensor_msgs::Joy last_joy_;
    bool has_joy_msg_ = false;

    double max_linear_vel_;
    double max_angular_vel_;

    std::unique_ptr<AudioPlayer> audio_player_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
        geometry_msgs::Twist twist;
        twist.linear.x = msg->axes[1] * max_linear_vel_;
        twist.angular.z = msg->axes[3] * max_angular_vel_;
        cmd_vel_pub_.publish(twist);

        if (has_joy_msg_ && msg->buttons[5] == 1 && last_joy_.buttons[5] == 0) {
            // 音声出力を非同期で実行
            std::async(std::launch::async, [this]() {
                audio_player_->playSound("shoot");
                std::this_thread::sleep_for(std::chrono::seconds(3));
            });

            std_msgs::Bool shoot;
            shoot.data = true;
            shoot_pub_.publish(shoot);
        }
        last_joy_ = *msg;
        has_joy_msg_ = true;
    }

  public:
    JoyController() : nh_(), pnh_("~") {
        joy_sub_ = nh_.subscribe("joy", 10, &JoyController::joyCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1, true);
        shoot_pub_ = nh_.advertise<std_msgs::Bool>("shoot", 1, true);

        max_linear_vel_ = pnh_.param<double>("max_linear_vel", 0.5);
        max_angular_vel_ = pnh_.param<double>("max_angular_vel", 1.0);

        audio_player_ = std::make_unique<AudioPlayer>();
        audio_player_->loadSound("shoot", "/home/nakao-t/DemoPro2024/src/joy_controller/sound/677839__carlfnf__sniper-shoot.wav");
    };
    ~JoyController(){};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_contoroller");
    JoyController joy_controller;
    ros::spin();
    return 0;
}
