#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

class MapServer {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid map_;

    void generateMap() {
        map_.header.frame_id = "map";
        map_.info.resolution = pnh_.param<double>("resolution", 0.05);
        map_.info.width = pnh_.param<int>("width", 5) / map_.info.resolution;
        map_.info.height = pnh_.param<int>("height", 3) / map_.info.resolution;
        ROS_INFO("Map size: %d x %d", map_.info.width, map_.info.height);
        map_.info.origin.position.x = 0;
        map_.info.origin.position.y = -(map_.info.height / 2 * map_.info.resolution);
        ROS_INFO("Map origin: (%f, %f)", map_.info.origin.position.x, map_.info.origin.position.y);
        map_.info.origin.position.z = 0;
        map_.info.origin.orientation.x = 0;
        map_.info.origin.orientation.y = 0;
        map_.info.origin.orientation.z = 0;
        map_.info.origin.orientation.w = 1;

        map_.data.resize(map_.info.width * map_.info.height, 0);
        for (int i = 0; i < map_.info.width; i++) {
            for (int j = 0; j < map_.info.height; j++) {
                if (i == 0 || j == 0 || i == map_.info.width - 1 || j == map_.info.height - 1) {
                    map_.data[i + j * map_.info.width] = 100;
                }
            }
        }

        // double center_x[] = {0.5, 2.5, 4.5};
        // double center_y[] = {-1.0, 1.0, -1.0};
        // double radius[] = {0.1, 0.1, 0.1};
        // for (int i = 0; i < map_.info.width; i++) {
        //     for (int j = 0; j < map_.info.height; j++) {
        //         for (int k = 0; k < sizeof(center_x) / sizeof(center_x[0]); k++) {
        //             double dx = i * map_.info.resolution + map_.info.origin.position.x - center_x[k];
        //             double dy = j * map_.info.resolution + map_.info.origin.position.y - center_y[k];
        //             if (dx * dx + dy * dy <= radius[k] * radius[k]) {
        //                 map_.data[i + j * map_.info.width] = 100;
        //                 ROS_INFO("Obstacle at (%d, %d)", i, j);
        //             }
        //             ROS_INFO("dx: %f, dy: %f", dx, dy);
        //         }
        //     }
        // }
    }

    void publishMap() {
        map_.header.stamp = ros::Time::now();
        map_pub_.publish(map_);
    }

  public:
    MapServer() : nh_(), pnh_("~") {
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        generateMap();
        publishMap();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_server");
    MapServer map_server;
    ros::spin();
    return 0;
}