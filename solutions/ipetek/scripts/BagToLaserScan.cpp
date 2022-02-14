#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <vector>

class BagToLaserScan {
   public:
    BagToLaserScan(const ros::NodeHandle& nh) : nh_(nh) {
        bagSub_ = nh_.subscribe("/points", 1, &BagToLaserScan::bagCallback, this);
        laserPub_ = nh_.advertise<sensor_msgs::LaserScan>("/laser", 1);

        pub_msg_.angle_min = 0;
        pub_msg_.angle_max = 2 * M_PI;
        pub_msg_.range_max = 100;
        pub_msg_.angle_increment = 2 * M_PI / (10000 - 1);
        pub_msg_.header.frame_id = "image";
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber bagSub_;
    ros::Publisher laserPub_;
    sensor_msgs::LaserScan pub_msg_;

    void bagCallback(const visualization_msgs::Marker::ConstPtr msg) {
        pub_msg_.ranges = std::vector<_Float32>(0);
        for (auto& p : msg->points) {
            pub_msg_.ranges.push_back(sqrt(pow(p.x, 2) + pow(p.y, 2)));
        }
        laserPub_.publish(pub_msg_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stage_wanderer");
    ros::NodeHandle nh;
    BagToLaserScan bagtolaser(nh);
    ros::spin();

    return 0;
}