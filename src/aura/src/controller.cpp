#include "../include/aura/controller.h"

Controller::Controller(const ros::NodeHandlePtr &nh) {
    if (nh == nullptr) {
        Controller::nh = ros::NodeHandlePtr(new ros::NodeHandle);
    } else {
        Controller::nh = nh;
    }
    Controller::trajectory_saver = new trajectory::Trajectory;
    Controller::takeoff_pub = new ros::Publisher(Controller::nh->advertise<std_msgs::Empty>("/bebop/takeoff", 1000));
    Controller::land_pub = new ros::Publisher(Controller::nh->advertise<std_msgs::Empty>("/bebop/land", 1000));
    Controller::parrot_cmd_pub = new ros::Publisher(Controller::nh->advertise<Twist>("/bebop/cmd_vel", 1000));
    Controller::camera_cmd_pub = new ros::Publisher(Controller::nh->advertise<Twist>("/bebop/camera_control", 1000));
    Controller::empty_msg = std_msgs::EmptyConstPtr(new std_msgs::Empty);
    Controller::rate = new ros::Rate(10);
}

void Controller::publish_cmd(geometry_msgs::TwistPtr twist, const uint &time) {
    uint end = time * 10;
    for (int i = 0; i < end; i++) {
        Controller::parrot_cmd_pub->publish(twist);
        Controller::rate->sleep();
    }
    twist->angular.z = 0;
    twist->linear.x = 0;
    twist->linear.y = 0;
    twist->linear.z = 0;
    for (int i = 0; i < 5; i++) {
        Controller::parrot_cmd_pub->publish(twist);
    }
}

void Controller::publish_camera_cmd(geometry_msgs::TwistPtr twist, const uint &time) {
    for (int i = 0; i < 5; i++) {
        Controller::camera_cmd_pub->publish(twist);
    }
}

void Controller::publish_twist(double x, double y, double z, double degree, const uint &time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.x = x;
    twist->linear.y = y;
    twist->linear.z = z;
    twist->angular.z = to_radian(degree);

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::rotate(double degree, uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->angular.z = Controller::to_radian(degree);

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}


void Controller::go_forward(uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.x = 0.1;

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::go_backward(uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.x = -0.1;

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::go_left(uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.y = 0.1;

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::go_right(uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.y = -0.1;

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::go_up(uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.z = 0.1;

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::go_down(uint time) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->linear.z = -0.1;

    Controller::trajectory_saver->add(time, twist);
    Controller::publish_cmd(twist, time);
}

void Controller::look_up() {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->angular.y = 200;
    Controller::publish_camera_cmd(twist, 2);
}

void Controller::look_down() {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->angular.y = -200;
    Controller::publish_camera_cmd(twist, 2);
}

void Controller::look_left() {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->angular.z = -200;
    Controller::publish_camera_cmd(twist, 2);
}

void Controller::look_right() {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->angular.z = 200;
    Controller::publish_camera_cmd(twist, 2);
}

void Controller::reset_camera() {
    geometry_msgs::TwistPtr twist(new Twist);
    Controller::publish_camera_cmd(twist, 2);
}

void Controller::publish_camera_twist(double y_degree, double z_degree) {
    geometry_msgs::TwistPtr twist(new Twist);
    twist->angular.y = y_degree;
    twist->angular.z = z_degree;
    Controller::publish_camera_cmd(twist, 3);
}

void Controller::takeoff() {
    Controller::trajectory_saver->add(0, geometry_msgs::TwistPtr(new Twist), trajectory::TAKEOFF);
    Controller::takeoff_pub->publish(Controller::empty_msg);
}

void Controller::land() {
    Controller::go_down(5);

    Controller::trajectory_saver->add(0, geometry_msgs::TwistPtr(new Twist), trajectory::LAND);
    Controller::land_pub->publish(Controller::empty_msg);
}

