#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>

#include <cmath>

int LINEAR_SPEED = 4;
int ANGULAR_SPEED = 20;

class MouseTurtleController {
 private:
  ros::NodeHandle node_;
  ros::Subscriber mouse_pos_;
  ros::Subscriber turt_pos_;
  ros::Publisher turt_vel_;
  ros::Timer turt_pub_timer_;

  turtlesim::Pose last_turtle_pos_;
  geometry_msgs::Point last_mouse_pos_;

  int SCREEN_X, SCREEN_Y;

 public:
  MouseTurtleController() {
    node_.getParam("screen_x", SCREEN_X);
    node_.getParam("screen_y", SCREEN_Y);
    mouse_pos_ = node_.subscribe(
        "/mouse_position", 1, &MouseTurtleController::mouse_pos_callback, this);
    turt_pos_ = node_.subscribe(
        "/turtle1/pose", 1, &MouseTurtleController::turtle_pos_callback, this);
    turt_vel_ = node_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    turt_pub_timer_ = node_.createTimer(
        ros::Duration(0.01), &MouseTurtleController::turt_pub_callback, this);
  }

 private:
  // CALLBACK FUNCTIONS
  void mouse_pos_callback(const geometry_msgs::Point::ConstPtr &msg) {
    last_mouse_pos_ = *msg;
  }
  void turtle_pos_callback(const turtlesim::Pose::ConstPtr &msg) {
    last_turtle_pos_ = *msg;
  }
  void turt_pub_callback(const ros::TimerEvent &event) {
    geometry_msgs::Twist pub_data;
    geometry_msgs::Point desk_to_turt = desktop_to_turtlesim(last_mouse_pos_);
    double curr_distance = get_distance(desk_to_turt);
    double curr_ang = get_ang_distance(desk_to_turt);
    if (curr_distance > 1) {
      pub_data.linear.x = curr_distance * LINEAR_SPEED;
    }

    if (abs(curr_ang) > 0.3) {
      pub_data.angular.z = curr_ang > 0 ? ANGULAR_SPEED : -ANGULAR_SPEED;
    }

    turt_vel_.publish(pub_data);
  }

  // UTILITY FUNCTIONS
  geometry_msgs::Point desktop_to_turtlesim(
      const geometry_msgs::Point &desktop_coord) {
    geometry_msgs::Point ret_point;
    ret_point.x = (desktop_coord.x * 11) / SCREEN_X;
    ret_point.y = 11 - (desktop_coord.y * 11) / SCREEN_Y;
    return ret_point;
  }

  double get_distance(const geometry_msgs::Point &goal) {
    return sqrt(pow((goal.x - last_turtle_pos_.x), 2) +
                pow((goal.y - last_turtle_pos_.y), 2));
  }

  double get_ang_distance(const geometry_msgs::Point &goal) {
    double ang =
        atan2(goal.y - last_turtle_pos_.y, goal.x - last_turtle_pos_.x) -
        last_turtle_pos_.theta;
    if (ang < -M_PI)
      ang += 2 * M_PI;

    else if (ang > M_PI)
      ang += -2 * M_PI;

    return ang;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlesim_mouse_tracker");

  // ros::param::set("screen_x", 1920);
  // ros::param::set("screen_y", 1080);
  MouseTurtleController turtcont;
  ros::spin();
  return 0;
}
