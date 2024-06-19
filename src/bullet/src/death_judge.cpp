#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <future>
#include <thread>
#include "audio_player/audio_player.h"

class DeathJudge {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber bullets_sub_;
    ros::Publisher death_judge_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseArray bullet_poses_;
    std::string map_frame_id_;
    std::string robot_frame_id_;

    const double death_distance_ = 0.1;

    std::unique_ptr<AudioPlayer> audio_player_;

  public:
    DeathJudge() : nh_(), pnh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
        bullets_sub_ = nh_.subscribe("bullets", 10, &DeathJudge::bulletsCallback, this);
        death_judge_pub_ = nh_.advertise<std_msgs::Bool>("death_judge", 10);

        map_frame_id_ = pnh_.param<std::string>("map_frame_id", "map");
        robot_frame_id_ = pnh_.param<std::string>("robot_frame_id", "base_link");

        audio_player_ = std::make_unique<AudioPlayer>();
        // TODO: wavファイルを用意する
        audio_player_->loadSound("game_over", "/home/nakao-t/DemoPro2024/src/bullet/sound/game_over_voice.wav");
    }

    void bulletsCallback(const geometry_msgs::PoseArray::ConstPtr &bullets_msg) { bullet_poses_ = *bullets_msg; }

    void getRobotPose() {
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        robot_pose_.pose.position.x = transform.transform.translation.x;
        robot_pose_.pose.position.y = transform.transform.translation.y;
        robot_pose_.pose.position.z = transform.transform.translation.z;
        robot_pose_.pose.orientation = transform.transform.rotation;
    }

    bool isDead() {
        getRobotPose();
        for (const auto &bullet : bullet_poses_.poses) {
            double distance = std::hypot(bullet.position.x - robot_pose_.pose.position.x,
                                         bullet.position.y - robot_pose_.pose.position.y);
            if (distance < death_distance_) {
                std::async(std::launch::async, [this]() {
                    audio_player_->playSound("shoot");
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            );
                return true;
            }
        }
        return false;
    }

    void publishDeathJudge() {
        if (isDead()) {
            std_msgs::Bool death_judge_msg;
            death_judge_msg.data = true;
            death_judge_pub_.publish(death_judge_msg);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "death_judge");
    DeathJudge death_judge;
    ros::Rate rate(10);
    while (ros::ok()) {
        death_judge.publishDeathJudge();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}