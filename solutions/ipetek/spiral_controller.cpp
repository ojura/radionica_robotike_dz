#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>
#define _USE_MATH_DEFINES
#include <cmath>


class Travel{
    private:
        ros::NodeHandle node;
        ros::Publisher pub_vel;
        ros::Timer pub_timer;
        double x = 10;
        double y = 0;
    public:
        Travel(){
            pub_vel = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
            pub_timer = node.createTimer(ros::Duration(0.1), &Travel::pubCallback, this);
        }
    private:
        void pubCallback(const ros::TimerEvent& event){
            geometry_msgs::Twist msg;
            msg.linear.x = y;
            msg.angular.z = x;
            pub_vel.publish(msg);
            x -= 0.05;
            y += 0.3;
        }
};

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "spiral_controller");
    Travel travel;

    ros::spin();
    return 0;
}
