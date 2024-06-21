#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "audio_player/audio_player.h"

class BulletLauncher {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber shoot_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher bullets_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::PoseStamped robot_pose_;
    std::string map_frame_id_;
    std::string robot_frame_id_;

    struct Bullet {
        geometry_msgs::Pose pose;
        ros::Time created_time;
        ros::Time last_time;
        double speed;
        double lifetime;
    };

    std::vector<Bullet> bullets_;
    geometry_msgs::PoseArray bullet_poses_;
    const int max_bullets_ = 5;
    double bullet_speed_;
    double bullet_lifetime_;
    nav_msgs::OccupancyGrid map_data_;
    std::unique_ptr<AudioPlayer> audio_player_;

  public:
    BulletLauncher() : nh_(), pnh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
        shoot_sub_ = nh_.subscribe("shoot", 10, &BulletLauncher::shootCallback, this);
        map_sub_ = nh_.subscribe("/map", 10, &BulletLauncher::mapCallback, this);
        bullets_pub_ = nh_.advertise<geometry_msgs::PoseArray>("bullets", 10);

        map_frame_id_ = pnh_.param<std::string>("map_frame_id", "map");
        robot_frame_id_ = pnh_.param<std::string>("robot_frame_id", "base_link");
        bullet_speed_ = pnh_.param<double>("bullet_speed", 1.0);
        bullet_lifetime_ = pnh_.param<double>("bullet_lifetime", 10.0);
        bullet_poses_.header.frame_id = map_frame_id_;

        audio_player_ = std::make_unique<AudioPlayer>();
        std::string package_path = ros::package::getPath("bullet");
        std::string sound_path = package_path + "/sound/shoot_gun.wav";
        audio_player_->loadSound("shoot", sound_path);
    }

    void shootCallback(const std_msgs::Bool::ConstPtr &shoot_msg) {
        if (shoot_msg->data) {
            if (bullets_.size() < max_bullets_) {
                audio_player_->playSound("shoot");
                createBullet();
            }
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg) {
        map_data_ = *map_msg;
        ROS_INFO("Map received");
    }

    void createBullet() {
        getRobotPose();
        Bullet new_bullet;
        new_bullet.pose = robot_pose_.pose;
        new_bullet.created_time = ros::Time::now();
        new_bullet.last_time = ros::Time::now();
        new_bullet.speed = bullet_speed_;
        new_bullet.lifetime = bullet_lifetime_;
        bullets_.push_back(new_bullet);
        ROS_INFO("Bullet created");
    }

    void updateBullet() {
        ros::Time current_time = ros::Time::now();
        bullet_poses_.poses.clear();

        for (auto &bullet : bullets_) {
            double dt = (current_time - bullet.last_time).toSec();
            bullet.last_time = current_time;

            double direction = tf2::getYaw(bullet.pose.orientation);
            double new_x = bullet.pose.position.x + bullet.speed * cos(direction) * dt;
            double new_y = bullet.pose.position.y + bullet.speed * sin(direction) * dt;

            if (isCollision(new_x, new_y, direction)) {
                tf2::Quaternion quat;
                quat.setRPY(0, 0, direction);
                bullet.pose.orientation = tf2::toMsg(quat);
                // ROS_INFO("Bullet bounced");
            } else {
                bullet.pose.position.x = new_x;
                bullet.pose.position.y = new_y;
            }

            if ((current_time - bullet.created_time).toSec() > bullet.lifetime) {
                ROS_INFO("Bullet expired");
                continue;
            }

            bullet_poses_.poses.push_back(bullet.pose);
        }

        bullet_poses_.header.stamp = current_time;
        bullets_pub_.publish(bullet_poses_);

        bullets_.erase(std::remove_if(bullets_.begin(), bullets_.end(),
                                      [&](const Bullet &bullet) {
                                          return (current_time - bullet.created_time).toSec() > bullet.lifetime;
                                      }),
                       bullets_.end());
    }

    bool isCollision(double x, double y, double &direction) {
        if (map_data_.data.empty()) {
            ROS_INFO("Map data not available");
            return false;
        }

        int map_x = (x - map_data_.info.origin.position.x) / map_data_.info.resolution;
        int map_y = (y - map_data_.info.origin.position.y) / map_data_.info.resolution;

        int offsets[3] = {-1, 0, 1};
        double normal_x = 0.0, normal_y = 0.0;
        bool collision = false;
        for (int dx : offsets) {
            for (int dy : offsets) {
                int nx = map_x + dx;
                int ny = map_y + dy;
                if (nx >= 0 && ny >= 0 && nx < map_data_.info.width && ny < map_data_.info.height) {
                    int index = ny * map_data_.info.width + nx;
                    if (map_data_.data[index] > 50) { // values > 50 represent an obstacle
                        normal_x += dx * map_data_.info.resolution;
                        normal_y += dy * map_data_.info.resolution;
                        collision = true;
                    }
                }
            }
        }
        if (collision) {
            double normal_angle = atan2(normal_y, normal_x);                       // 壁の法線方向
            direction = normalizeAngle(2 * (normal_angle + M_PI / 2) - direction); // 反射方向
            // ROS_INFO("Angle: %f", normal_angle);
            return true;
        }

        return false;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "bullet_launcher");
    BulletLauncher bullet_launcher;
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        bullet_launcher.updateBullet();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
