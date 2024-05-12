#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class BulletLauncher {
  private:
    ros::NodeHandle nh_;
    ros::Time last_time_;
    ros::Subscriber joy_sub_;
    sensor_msgs::Joy last_joy_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    geometry_msgs::PoseStamped robot_pose_;
    const std::string map_frame_id_ = "map";

    struct Bullet {
        std::string frame_id;
        geometry_msgs::PoseStamped pose;
        ros::Time created_time;
        ros::Time last_time;
        double speed;
        double lifetime;
    };

    std::vector<Bullet> bullets_;
    const int max_bullets_ = 5;
    int bullet_count_ = 0;
    const double bullet_speed_ = 0.2;
    const double bullet_lifetime_ = 10.0;

  public:
    BulletLauncher() : nh_(), tf_buffer_(), tf_listener_(tf_buffer_) {
        joy_sub_ = nh_.subscribe("joy", 10, &BulletLauncher::joyCallback, this);
        robot_pose_.header.frame_id = map_frame_id_;
        last_time_ = ros::Time::now();
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
        if (joy_msg->buttons[0] == 1 && last_joy_.buttons[0] == 0) {
            if (bullets_.size() < max_bullets_) {
                createBullet();
            }
        }
        last_joy_ = *joy_msg;
    }

    void createBullet() {
        geometry_msgs::PoseStamped bullet_pose;
        bullet_pose.header.stamp = ros::Time::now();
        bullet_pose.header.frame_id = map_frame_id_;
        bullet_pose.pose.position.x = robot_pose_.pose.position.x;
        bullet_pose.pose.position.y = robot_pose_.pose.position.y;
        bullet_pose.pose.position.z = robot_pose_.pose.position.z;
        bullet_pose.pose.orientation = robot_pose_.pose.orientation;
        // ROS_INFO("Bullet created at (%f, %f)", bullet_pose_.pose.position.x, bullet_pose_.pose.position.y);

        Bullet new_bullet;
        new_bullet.frame_id = "bullet" + std::to_string(bullet_count_++);
        new_bullet.pose = bullet_pose;
        new_bullet.created_time = ros::Time::now();
        new_bullet.last_time = ros::Time::now();
        new_bullet.speed = bullet_speed_;
        new_bullet.lifetime = bullet_lifetime_;
        bullets_.push_back(new_bullet);
        ROS_INFO("Bullet %s created", new_bullet.frame_id.c_str());
    }

    void updateBullet() {
        ros::Time current_time = ros::Time::now();

        for (auto &bullet : bullets_) {
            double dt = (current_time - bullet.last_time).toSec();
            bullet.last_time = current_time;

            double direction = tf2::getYaw(bullet.pose.pose.orientation);
            bullet.pose.pose.position.x += bullet.speed * cos(direction) * dt;
            bullet.pose.pose.position.y += bullet.speed * sin(direction) * dt;

            if ((current_time - bullet.created_time).toSec() > bullet.lifetime) {
                ROS_INFO("Bullet %s expired", bullet.frame_id.c_str());
                continue;
            }

            publishBulletTransform(bullet);
        }

        bullets_.erase(std::remove_if(bullets_.begin(), bullets_.end(),
                                      [&](const Bullet &bullet) {
                                          return (current_time - bullet.created_time).toSec() > bullet.lifetime;
                                      }),
                       bullets_.end());
    }

    void publishBulletTransform(const Bullet &bullet) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = bullet.pose.header.frame_id;
        transform.child_frame_id = bullet.frame_id;
        transform.transform.translation.x = bullet.pose.pose.position.x;
        transform.transform.translation.y = bullet.pose.pose.position.y;
        transform.transform.translation.z = bullet.pose.pose.position.z;
        transform.transform.rotation = bullet.pose.pose.orientation;
        tf_broadcaster_.sendTransform(transform);
    }

    void getRobotPose() {
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(map_frame_id_, "base_link", ros::Time(0));
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
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        bullet_launcher.getRobotPose();
        bullet_launcher.updateBullet();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
