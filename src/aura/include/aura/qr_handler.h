#ifndef AURA_QR_HANDLER_H
#define AURA_QR_HANDLER_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>

// class for reading and handeling QR code
class QRHandler {
private:
    std::string *qr_code;
    char *goals;
    bool goal;
    int goal_id;

public:
    explicit QRHandler(const std_msgs::StringConstPtr &qr_msg);

    // compute given QR code
    void compute();

    // get target position from given index
    char get_target(int target);

    // get goal number
    int get_goal();

    // get all targets position in given QR code
    std::string get_all_targets();

    // current QR code is goal
    bool is_goal();
};

#endif
