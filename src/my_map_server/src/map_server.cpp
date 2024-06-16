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

        rect(1.5, -0.9, 0.1, 0.7); // (x, y) 座標と幅 w, 高さ h (m)
        rect(3.5, 0.3, 0.1, 0.7);
        line(2.0, 0.5, 3.0, -0.5); // (x1, y1) (x2, y2) (m)
        line(0.6, 0.4, 0.6, 1.0);
        line(4.4, -0.3, 4.4, -0.9);
    }

    void rect(double x, double y, double w, double h) {
        int x0 = (x - map_.info.origin.position.x) / map_.info.resolution;
        int y0 = (y - map_.info.origin.position.y) / map_.info.resolution;
        int width = w / map_.info.resolution;
        int height = h / map_.info.resolution;

        for (int i = x0; i < x0 + width; ++i) {
            for (int j = y0; j < y0 + height; ++j) {
                if (i >= 0 && i < map_.info.width && j >= 0 && j < map_.info.height) {
                    map_.data[i + j * map_.info.width] = 100;
                }
            }
        }
    }

    void line(double x1, double y1, double x2, double y2) {
        int x0 = (x1 - map_.info.origin.position.x) / map_.info.resolution;
        int y0 = (y1 - map_.info.origin.position.y) / map_.info.resolution;
        int x1_scaled = (x2 - map_.info.origin.position.x) / map_.info.resolution;
        int y1_scaled = (y2 - map_.info.origin.position.y) / map_.info.resolution;

        int dx = abs(x1_scaled - x0);
        int dy = abs(y1_scaled - y0);
        int sx = (x0 < x1_scaled) ? 1 : -1;
        int sy = (y0 < y1_scaled) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x0 >= 0 && x0 < map_.info.width && y0 >= 0 && y0 < map_.info.height) {
                map_.data[x0 + y0 * map_.info.width] = 100;
            }
            if (x0 == x1_scaled && y0 == y1_scaled)
                break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
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