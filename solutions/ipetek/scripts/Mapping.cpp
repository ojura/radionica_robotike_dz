#include <geometry_msgs/Point.h>
#define _USE_MATH_DEFINES
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <vector>

class Mapping {
   public:
    Mapping(const ros::NodeHandle& nh) : nh_(nh) {
        bagSub_ = nh_.subscribe("/scan", 1, &Mapping::bagCallback, this);
        laserPub_ = nh_.advertise<visualization_msgs::Marker>("point_positions", 0);

        static tf2_ros::TransformListener tf_listener_(tf_buffer_);
        pub_msg_.header.frame_id = "map";
        pub_msg_.color.a = 1;
        pub_msg_.color.r = 1;
        pub_msg_.scale.x = 0.05;
        pub_msg_.scale.y = 0.05;
        pub_msg_.type = visualization_msgs::Marker::POINTS;

        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        grid_msg_.header.stamp = ros::Time::now();
        grid_msg_.header.frame_id = "map";
        grid_msg_.info.resolution = 0.05;
        grid_msg_.info.height = 1200;
        grid_msg_.info.width = 1200;
        grid_msg_.info.origin.position.x = -15;
        grid_msg_.info.origin.position.y = -45;
        grid_msg_.info.origin.position.z = 0;
        grid_msg_.info.origin.orientation.x = 0;
        grid_msg_.info.origin.orientation.y = 0;
        grid_msg_.info.origin.orientation.z = 0;
        grid_msg_.info.origin.orientation.w = 1;
        grid_msg_.data = makeGrid(grid_msg_.info.height, grid_msg_.info.width);
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber bagSub_;
    ros::Publisher laserPub_;
    visualization_msgs::Marker pub_msg_;
    tf2_ros::Buffer tf_buffer_;
    nav_msgs::OccupancyGrid grid_msg_;
    ros::Publisher grid_pub_;
    int scanNum_ = 0;
    // int accumulate_every_n = 10;

    void bagCallback(const sensor_msgs::LaserScan& msg) {
        if (msg.header.stamp < pub_msg_.header.stamp) {
            ROS_INFO("Timestamp has jumped backwards, clearing the buffer.");
            pub_msg_.header.stamp = msg.header.stamp;
            tf_buffer_.clear();
            pub_msg_.points.clear();
            grid_msg_.data = makeGrid(grid_msg_.info.height, grid_msg_.info.width);
            return;
        }
        // if (scanNum_ != accumulate_every_n) {
        //    scanNum_++;
        //    return;
        //}
        scanNum_ = 0;
        geometry_msgs::Vector3 trans;
        geometry_msgs::Quaternion quat;
        double theta;
        try {
            geometry_msgs::TransformStamped tf_global_laser =
                tf_buffer_.lookupTransform("map", msg.header.frame_id, msg.header.stamp);
            trans = tf_global_laser.transform.translation;
            quat = tf_global_laser.transform.rotation;
            theta = 2 * atan2(quat.z, quat.w);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        pub_msg_.header.stamp = msg.header.stamp;
        // pub_msg_.points.clear();

        double angle;
        for (int i = 0; i < msg.ranges.size(); i++) {
            if (msg.ranges[i] == 0.0) continue;
            if (i > 1 && i < msg.ranges.size() - 1) {
                double diff_before = abs(msg.ranges[i] - msg.ranges[i - 1]);
                double diff_after = abs(msg.ranges[i] - msg.ranges[i + 1]);
                if (diff_before > 0.02 || diff_after > 0.02) continue;
            }

            double angle;
            geometry_msgs::Point p;
            angle = msg.angle_min + i * msg.angle_increment + theta;
            p.x = cos(angle) * msg.ranges[i] + trans.x;
            p.y = sin(angle) * msg.ranges[i] + trans.y;
            pub_msg_.points.push_back(p);

            int grid_y = round(p.y / grid_msg_.info.resolution) + 900;
            int grid_x = round(p.x / grid_msg_.info.resolution) + 300;
            grid_msg_.data[grid_y * grid_msg_.info.height + grid_x] = 100;

            // Bresenham's
            std::vector<std::pair<int, int>> linePoints =
                bresenham((trans.x / grid_msg_.info.resolution) + 300, (trans.y / grid_msg_.info.resolution) + 900,
                          grid_x, grid_y);
            for (auto& point : linePoints) {
                grid_msg_.data[point.second * grid_msg_.info.height + point.first] = 0;
            }
        }
        laserPub_.publish(pub_msg_);
        grid_msg_.header.stamp = msg.header.stamp;
        grid_pub_.publish(grid_msg_);
    }

    std::vector<int8_t> makeGrid(int height, int width, int init = -1) {
        std::vector<int8_t> vec(width * height, init);
        return vec;
    }

    std::vector<std::pair<int, int>> bresenham(int x, int y, int x2, int y2) {
        std::vector<std::pair<int, int>> ret;
        int dx, dy, p;
        dx = x2 - x;
        dy = y2 - y;
        p = 2 * dy - dx;
        while (x <= x2) {
            if (p < 0) {
                x++;
                p += 2 * dy;
            } else {
                x++;
                y++;
                p += 2 * (dy - dx);
            }
            ret.push_back({x, y});
        }
        return ret;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;
    Mapping bagtolaser(nh);
    ros::spin();

    return 0;
}
