#ifndef AURA_CONTROLLER_H
#define AURA_CONTROLLER_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include "trajectory.h"

#define Twist geometry_msgs::Twist

// robot moving Controller class
class Controller {
private:
    ros::NodeHandlePtr nh;
    ros::Rate *rate;
    ros::Publisher *takeoff_pub;
    ros::Publisher *land_pub;
    ros::Publisher *parrot_cmd_pub;
    ros::Publisher *camera_cmd_pub;
    std_msgs::EmptyConstPtr empty_msg;
    trajectory::Trajectory *trajectory_saver;

    inline double to_radian(double degree) {
        return degree / 180.0 * M_PI;
    }

public:
    explicit Controller(const ros::NodeHandlePtr &nh = nullptr);

    void publish_cmd(geometry_msgs::TwistPtr twist, const uint &time);

    void publish_camera_cmd(geometry_msgs::TwistPtr twist, const uint &time);

    void publish_twist(double x, double y, double z, double degree, const uint &time);

    void publish_camera_twist(double y_degree = 0, double z_degree = 0);

    void takeoff();

    void land();

    void rotate(double degree, uint time);

    void go_forward(uint time);

    void go_backward(uint time);

    void go_left(uint time);

    void go_right(uint time);

    void go_up(uint time);

    void go_down(uint time);

    void look_up();

    void look_down();

    void look_left();

    void look_right();

    void reset_camera();
};


#endif //AURA_CONTROLLER_H
