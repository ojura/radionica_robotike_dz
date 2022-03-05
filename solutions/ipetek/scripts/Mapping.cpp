#include "geometry_msgs/Point.h"
#define _USE_MATH_DEFINES
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
        bagSub_ = nh_.subscribe("/pioneer/scan", 1, &Mapping::bagCallback, this);
        laserPub_ = nh_.advertise<visualization_msgs::Marker>("point_positions", 0);
        static tf2_ros::TransformListener tf_listener_(tf_buffer_);
        pub_msg_.header.frame_id = "map";
        pub_msg_.color.a = 1;
        pub_msg_.color.r = 1;
        pub_msg_.scale.x = 0.05;
        pub_msg_.scale.y = 0.05;
        pub_msg_.type = visualization_msgs::Marker::POINTS;
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber bagSub_;
    ros::Publisher laserPub_;
    visualization_msgs::Marker pub_msg_;
    tf2_ros::Buffer tf_buffer_;
    int scanNum = 0;
    int accumulate_every_n = 10;

    void bagCallback(const sensor_msgs::LaserScan& msg) {
        if (msg.header.stamp < pub_msg_.header.stamp) {
            ROS_INFO("Timestamp has jumped backwards, clearing the buffer.");
            pub_msg_.header.stamp = msg.header.stamp;
            tf_buffer_.clear();
            return;
        }
        if (scanNum != accumulate_every_n) {
            scanNum++;
            return;
        }
        scanNum = 0;
        geometry_msgs::Vector3 trans;
        geometry_msgs::Quaternion quat;
        double theta;
        try {
            
            geometry_msgs::TransformStamped tf_global_laser =
                tf_buffer_.lookupTransform("map", msg.header.frame_id, msg.header.stamp);
            trans = tf_global_laser.transform.translation;
            quat = tf_global_laser.transform.rotation;
            theta = 2 * atan2(quat.z, quat.w);
        } catch (tf2::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ROS_INFO("2\n");
        }
        //pub_msg_.header.frame_id = msg.header.frame_id;
        pub_msg_.header.stamp = msg.header.stamp;
        // pub_msg_.points.clear();
        
        double angle;
        for(int i = 0; i < msg.ranges.size(); i++){
            if (msg.ranges[i] == 0.0) continue;
            geometry_msgs::Point p;
            angle = msg.angle_min + i * msg.angle_increment + theta;
            p.x = cos(angle) * msg.ranges[i] + trans.x;
            p.y = sin(angle) * msg.ranges[i] + trans.y;
            pub_msg_.points.push_back(p);
            
        }
        /*
        double angle = msg.angle_min + theta;
        for (auto& r : msg.ranges) {
            if (r == 0.0) continue;
            geometry_msgs::Point p;
            p.x = cos(angle) * r + trans.x;
            p.y = sin(angle) * r + trans.y;
            pub_msg_.points.push_back(p);
            angle += msg.angle_increment;
        }*/
        laserPub_.publish(pub_msg_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;
    Mapping bagtolaser(nh);
    ros::spin();

    return 0;
}