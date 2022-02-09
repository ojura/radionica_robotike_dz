#define _USE_MATH_DEFINES
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>

#include <cmath>
#include <ctime>
#include <iostream>
#include <numeric>
#include <vector>

#define ANGULAR_SPEED 2
#define LINEAR_SPEED 5

class Wanderer {
   public:
    Wanderer(const ros::NodeHandle& nh) : nh_(nh) {
        baseScanSubscriber_ = nh_.subscribe("/base_scan", 1, &Wanderer::baseScanCallback, this);
        cmdVelPublisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        srand(time(NULL));
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber baseScanSubscriber_;
    ros::Publisher cmdVelPublisher_;

    // callback functions
    void baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { vel_publish(msg); }
    void vel_publish(const sensor_msgs::LaserScan::ConstPtr& lastScan) {
        geometry_msgs::Twist pub_msg;
        int arrMid = lastScan->ranges.size() / 2;
        float meanRight = std::accumulate(lastScan->ranges.begin(), lastScan->ranges.begin() + arrMid, 0.0) /
                          (float)lastScan->ranges.size();
        float meanLeft = std::accumulate(lastScan->ranges.begin() + arrMid + 1, lastScan->ranges.end(), 0.0) /
                         (float)lastScan->ranges.size();

        pub_msg.angular.z = (rand() % 3 - 1);
        pub_msg.linear.x = LINEAR_SPEED;
        if (lastScan->ranges[arrMid] < 0.5 || lastScan->ranges[arrMid / 2] < 0.5 ||
            lastScan->ranges[3 * arrMid / 2] < 0.5) {
            pub_msg.linear.x = 0;
            pub_msg.angular.z = meanLeft > meanRight ? ANGULAR_SPEED : -ANGULAR_SPEED;
        }
        cmdVelPublisher_.publish(pub_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stage_wanderer");
    ros::NodeHandle nh;
    Wanderer wanderer(nh);
    ros::spin();

    return 0;
}
