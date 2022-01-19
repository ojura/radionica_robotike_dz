#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Point.h>
#include<turtlesim/Pose.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include<stdio.h>


int SCREEN_X = 1920; //EDIT
int SCREEN_Y = 1080; //EDIT
int LINEAR_SPEED = 5;
int ANGULAR_SPEED = 10;

class MouseTurtleController{
    private:
        ros::NodeHandle node;
        ros::Subscriber mouse_pos;
        ros::Subscriber turt_pos;
        ros::Publisher turt_vel;
        ros::Timer turt_pub_timer;

        turtlesim::Pose last_turtle_pos;
        geometry_msgs::Point last_mouse_pos;

    public:
        MouseTurtleController(){
            mouse_pos = node.subscribe("/mouse_position", 1, &MouseTurtleController::mouse_pos_callback, this);
            turt_pos = node.subscribe("/turtle1/pose", 1, &MouseTurtleController::turtle_pos_callback, this);
            turt_vel = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
            turt_pub_timer = node.createTimer(ros::Duration(0.01), &MouseTurtleController::turt_pub_callback, this);
        }
    
    private:
        void mouse_pos_callback(const geometry_msgs::Point::ConstPtr& msg){
            last_mouse_pos = *msg;
        }
        void turtle_pos_callback(const turtlesim::Pose::ConstPtr& msg){
            last_turtle_pos = *msg;
        }
        void turt_pub_callback(const ros::TimerEvent& event){
            geometry_msgs::Twist pub_data;
            geometry_msgs::Point desk_to_turt = desktop_to_turtlesim(last_mouse_pos);
            double curr_distance = get_distance(desk_to_turt);
            double curr_ang = get_ang_distance(desk_to_turt);
            if(curr_distance > 1){
                pub_data.linear.x = LINEAR_SPEED;
            }
            
            if(abs(curr_ang) > 0.3){
                if(curr_ang > 0)
                    pub_data.angular.z = ANGULAR_SPEED;
                else
                    pub_data.angular.z = -ANGULAR_SPEED;
            }
            
            turt_vel.publish(pub_data);
        }
    
    private:
        // map desktop coordinates (1920x1080) to turtlesims 11x11 coordinate system
        geometry_msgs::Point desktop_to_turtlesim(const geometry_msgs::Point& desktop_coord){
            geometry_msgs::Point ret_point;
            ret_point.x = (desktop_coord.x * 11) / SCREEN_X;
            ret_point.y = 11 - (desktop_coord.y * 11) / SCREEN_Y;
            return ret_point;
        }

        double get_distance(const geometry_msgs::Point& goal){
            return sqrt(pow((goal.x - last_turtle_pos.x),2) + pow((goal.y - last_turtle_pos.y), 2));
        }

        double get_ang_distance(const geometry_msgs::Point& goal){
            double ang = atan2(goal.y - last_turtle_pos.y, goal.x - last_turtle_pos.x) - last_turtle_pos.theta;
            if(ang < -M_PI)
                ang = 2*M_PI + ang;
            
            else if(ang > M_PI)
                ang = -2*M_PI + ang;

            return ang;
        }
};

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "MouseTurtleController");
    MouseTurtleController turtcont;
    ros::spin();
    return 0;
}

