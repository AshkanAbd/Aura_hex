
#ifndef AURA_IMAGE_PROCESS_H
#define AURA_IMAGE_PROCESS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "controller.h"
#include "qr_handler.h"

class ImageProcess {
private:
    ros::NodeHandlePtr nh;
    ros::Subscriber *img_sub;
    std::string *qr_topic;
    cv::Scalar *black_low, *black_up, *white_low, *white_up;
    Controller *controller;
    QRHandler *qrHandler;

public:
    double angle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0);

    void detect_black(const cv::UMat &frame, cv::UMat &dst);

    void detect_while(const cv::UMat &frame, cv::UMat &dst, std::vector<cv::Point> &dst_approx);

    ImageProcess(const ros::NodeHandlePtr &nh, const std::string &image_topic, const std::string &qr_topic,
                 Controller *controller);

    void get_image(const sensor_msgs::ImageConstPtr &img_msg);

    void process(const std::vector<cv::Point> &approx, const cv::UMat &frame_white);

    bool operator()(const std::vector<cv::Point> &p1, const std::vector<cv::Point> &p2);
};


#endif //AURA_IMAGE_PROCESS_H
