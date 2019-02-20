#include <ros/ros.h>
#include <iostream>
#include "../include/aura/controller.h"
#include "../include/aura/image_process.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    Controller controller;
    int op;
    double d, d1, d2, d3;
    uint t;
    while (true) {
        std::cin >> op;
        switch (op) {
            case 1:
                controller.takeoff();
                break;
            case 2:
                controller.land();
                break;
            case 3:
                std::cin >> d >> t;
                controller.rotate(d, t);
                break;
            case 4:
                std::cin >> t;
                controller.go_forward(t);
                break;
            case 5:
                std::cin >> t;
                controller.go_backward(t);
                break;
            case 6:
                std::cin >> t;
                controller.go_left(t);
                break;
            case 7:
                std::cin >> t;
                controller.go_right(t);
                break;
            case 8:
                std::cin >> t;
                controller.go_up(t);
                break;
            case 9:
                std::cin >> t;
                controller.go_down(t);
                break;
            case 10:
                controller.look_up();
                break;
            case 11:
                controller.look_down();
                break;
            case 12:
                controller.look_right();
                break;
            case 13:
                controller.look_left();
                break;
            case 14:
                controller.reset_camera();
                break;
            case 15:
                std::cin >> d >> d1 >> d2 >> d3 >> t;
                controller.publish_twist(d, d1, d2, d3, t);
                break;
            case 16:
                std::cin >> d1 >> d2;
                controller.publish_camera_twist(d1, d2);
                break;
            default:
                break;
        }
    }
}
