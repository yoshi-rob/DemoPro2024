#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class RobotTfBroadcaster {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_vel_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::Twist last_cmd_vel_;
    std::string robot_frame_id_;
    std::string odom_frame_id_;
    ros::Time last_time_;
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;

  public:
    RobotTfBroadcaster() : nh_(), pnh_("~") {
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &RobotTfBroadcaster::cmdVelCallback, this);

        x_ = pnh_.param<double>("initial_x", 0.0);
        y_ = pnh_.param<double>("initial_y", 0.0);
        th_ = pnh_.param<double>("initial_th", 0.0);
        robot_frame_id_ = pnh_.param<std::string>("robot_frame_id", "base_link");
        odom_frame_id_ = pnh_.param<std::string>("odom_frame_id", "odom");
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg) { last_cmd_vel_ = *cmd_vel_msg; }

    // 差動二輪ロボットの位置をtfでブロードキャストする
    void publishTransform() {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        x_ += last_cmd_vel_.linear.x * cos(th_) * dt;
        y_ += last_cmd_vel_.linear.x * sin(th_) * dt;
        th_ += last_cmd_vel_.angular.z * dt;

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = odom_frame_id_;
        transform.child_frame_id = robot_frame_id_;
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(transform);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_tf_broadcaster");
    RobotTfBroadcaster robot_tf_broadcaster;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        robot_tf_broadcaster.publishTransform();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}