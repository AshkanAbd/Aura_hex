#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include "../include/aura/qr_handler.h"
#include "../include/aura/controller.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void go_to_target(const char &c, Controller *controller);

void do_mission(const int &TARGET);

bool compare(const std::vector<cv::Point> &p1, const std::vector<cv::Point> &p2);

double angle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0);

void detect_black(const cv::UMat &frame, cv::UMat &dst);

void detect_while(const cv::UMat &frame, cv::UMat &dst, std::vector<cv::Point> &dst_approx);

cv::Scalar *black_low, *black_up, *white_low, *white_up;

int main(int argc, char **argv) {
    // Mission Target
    int TARGET = 4;

    cv::ocl::setUseOpenCL(true);
    ros::init(argc, argv, "test_image");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    Controller *controller = new Controller(nh);
    QRHandler *qr_handler;
    black_low = new cv::Scalar(0, 0, 0);
    black_up = new cv::Scalar(180, 255, 30);
    white_low = new cv::Scalar(0, 0, 50);
    white_up = new cv::Scalar(180, 100, 255);

    // 10 time in sec
    ros::Rate rate(10);

    while (true) {
        // Wait for QR code in image
        std_msgs::StringConstPtr qr_code = ros::topic::waitForMessage<std_msgs::String>("/barcode", *nh,
                                                                                        ros::Duration(1, 0));
        if (qr_code != nullptr) {
            // QR code detected
            qr_handler = new QRHandler(qr_code);
            qr_handler->compute();
            if (qr_handler->is_goal() && qr_handler->get_goal() == TARGET) {
                // Target found
                char c = qr_handler->get_target(TARGET);
                go_to_target(c, controller);
                do_mission(TARGET);
            } else {
                // Still search for target
                char c = qr_handler->get_target(TARGET);
                go_to_target(c, controller);
            }
        }
        // No qr code found
        // search for qr
        sensor_msgs::ImageConstPtr img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/bebop/image_raw", *nh,
                                                                                            ros::Duration(2, 0));
        cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
        cv::UMat frame, frame_black, frame_white;
        img_ptr->image.copyTo(frame);
        std::vector<cv::Point> approx;
        detect_black(frame, frame_black);
        detect_while(frame_black, frame_white, approx);
        if (!approx.empty()) {
            cv::Rect rect = cv::boundingRect(approx);
            if (rect.y > 320) {
                controller->go_right(1);
            } else if (rect.y < 120) {
                controller->go_left(1);
            } else {
                controller->go_forward(1);
            }
        }
        rate.sleep();
    }
}

void go_to_target(const char &c, Controller *controller) {
    if (c == 'W') {
        controller->go_forward(2);
    } else if (c == 'S') {
        controller->go_backward(2);
    } else if (c == 'E') {
        controller->rotate(-90, 1);
        controller->go_forward(2);
    } else {
        controller->rotate(90, 1);
        controller->go_forward(2);
    }
}

void do_mission(const int &TARGET) {
    if (TARGET == 4) {
        
    }
}


double angle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

bool compare(const std::vector<cv::Point> &p1, const std::vector<cv::Point> &p2) {
    double e1 = 0.05 * cv::arcLength(p1, true);
    double e2 = 0.05 * cv::arcLength(p2, true);
    std::vector<cv::Point> ap1, ap2;
    cv::approxPolyDP(p1, ap1, e1, true);
    cv::approxPolyDP(p2, ap2, e2, true);
    double area1 = cv::contourArea(ap1);
    double area2 = cv::contourArea(ap2);
    return area1 > area2;
}


void detect_black(const cv::UMat &frame, cv::UMat &dst) {
    cv::UMat frame_blur, frame_hsv, frame_thresh, frame_pyr_up, frame_pyr_down, frame_filter;
    cv::UMat frame_edge, frame1, frame_gray;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, *black_low, *black_up, frame_thresh);
    cv::bitwise_and(frame, frame, frame_filter, frame_thresh);
    cv::Canny(frame_thresh, frame_edge, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    long double biggest = -1;
    int biggest_index = -1;
    for (int i = 0; i < contours.size(); i++) {
        double epsilon = 0.02 * cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, epsilon, true);
        cv::Rect rect = cv::boundingRect(approx);
        long double area = rect.area();
        biggest = MAX(area, biggest);
        if (biggest == area) {
            biggest_index = i;
        }
    }
    frame.copyTo(frame1);
    if (biggest_index != -1) {
        cv::drawContours(frame1, contours, biggest_index, cv::Scalar(0, 255, 0), 3);
    }
    cv::Mat final_mask(frame.rows, frame.cols, frame.type(), cv::Scalar(0, 0, 0));
    cv::Rect rect = cv::boundingRect(contours[biggest_index]);
    cv::rectangle(final_mask, rect, cv::Scalar(255, 255, 255), -1);

//    cv::RotatedRect rotatedRect = cv::minAreaRect(contours[biggest_index]);
//    cv::Point2f vertices[4];
//    rotatedRect.points(vertices);
//    std::vector<cv::Point> points;
//    for (auto &vertex : vertices) {
//        points.push_back(vertex);
//    }
//    cv::fillPoly(final_mask, std::vector<std::vector<cv::Point>>{points}, cv::Scalar(255, 255, 255));

    cv::bitwise_and(frame, final_mask, dst);
    cv::imshow("detected black", dst);
}

void detect_while(const cv::UMat &frame, cv::UMat &dst, std::vector<cv::Point> &dst_approx) {
    cv::UMat frame_hsv, frame_pyr_up, frame_pyr_down, frame_thresh, frame_edge, frame_blur;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, *white_low, *white_up, frame_thresh);
    cv::Canny(frame_thresh, frame_edge, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), compare);
    for (const auto &contour : contours) {
        std::vector<cv::Point> approx;
        double epsilon = 0.05 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        if (approx.size() == 4/*true*/) {
            double maxCosine = 0;
            for (int j = 2; j < 5; j++) {
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            if (/*maxCosine < 0.3*/ true) {
//                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(255, 0, 0));;
                dst_approx = approx;
                break;
            }
        }
    }
    frame.copyTo(dst);
    cv::drawContours(frame, std::vector<std::vector<cv::Point>>{dst_approx}, -1, cv::Scalar(255, 0, 0));
    cv::imshow("white detect", frame);
}
