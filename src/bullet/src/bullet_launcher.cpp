#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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
    geometry_msgs::PoseStamped bullet_pose_;
    const std::string map_frame_id_ = "map";
    const std::string bullet_frame_id_ = "bullet";
    const double bullet_speed_ = 0.2;
    const double bullet_lifetime_ = 10.0;
    bool bullet_exist_ = false;

  public:
    BulletLauncher() : nh_(), tf_buffer_(), tf_listener_(tf_buffer_) {
        joy_sub_ = nh_.subscribe("joy", 10, &BulletLauncher::joyCallback, this);
        bullet_pose_.header.frame_id = map_frame_id_;
        robot_pose_.header.frame_id = map_frame_id_;
        last_time_ = ros::Time::now();
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
        if (joy_msg->buttons[0] == 1 && last_joy_.buttons[0] == 0) {
            createBullet();
        }
        last_joy_ = *joy_msg;
    }

    void createBullet() {
        bullet_pose_.header.stamp = ros::Time::now();
        last_time_ = ros::Time::now();
        bullet_pose_.pose.position.x = robot_pose_.pose.position.x;
        bullet_pose_.pose.position.y = robot_pose_.pose.position.y;
        bullet_pose_.pose.position.z = robot_pose_.pose.position.z;
        bullet_pose_.pose.orientation = robot_pose_.pose.orientation;
        // ROS_INFO("Bullet created at (%f, %f)", bullet_pose_.pose.position.x, bullet_pose_.pose.position.y);
        bullet_exist_ = true;
    }

    void updateBullet() {
        if (!bullet_exist_) {
            return;
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        double bullet_direction = tf2::getYaw(bullet_pose_.pose.orientation);
        bullet_pose_.pose.position.x += bullet_speed_ * cos(bullet_direction) * dt;
        bullet_pose_.pose.position.y += bullet_speed_ * sin(bullet_direction) * dt;

        if ((current_time - bullet_pose_.header.stamp).toSec() > bullet_lifetime_) {
            bullet_exist_ = false;
        }

        publishBulletTransform();
    }

    void publishBulletTransform() {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = bullet_pose_.header.frame_id;
        transform.child_frame_id = bullet_frame_id_;
        transform.transform.translation.x = bullet_pose_.pose.position.x;
        transform.transform.translation.y = bullet_pose_.pose.position.y;
        transform.transform.translation.z = bullet_pose_.pose.position.z;
        transform.transform.rotation = bullet_pose_.pose.orientation;
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
