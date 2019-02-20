#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>

void get_image(const sensor_msgs::ImageConstPtr &img_msg);

cv::Scalar black_low(0, 0, 0, 0);
cv::Scalar black_up(180, 255, 30, 0);

cv::Scalar white_low(0, 0, 50, 0);
cv::Scalar white_up(180, 100, 255, 0);

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_cpp");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    ros::Subscriber image_sub = nh->subscribe("/bebop/image_raw", 1000, get_image);
    ros::spin();
}

double angle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

int contour_compare(const std::vector<cv::Point> &p1, const std::vector<cv::Point> &p2) {
    double e1 = 0.02 * cv::arcLength(p1, true);
    double e2 = 0.02 * cv::arcLength(p2, true);
    std::vector<cv::Point> ap1, ap2;
    cv::approxPolyDP(p1, ap1, e1, true);
    cv::approxPolyDP(p2, ap2, e2, true);

    double area1 = cv::contourArea(ap1);
    double area2 = cv::contourArea(ap2);
    return area1 > area2;
}

void detect_black1(const cv::UMat &frame, cv::UMat &dst) {
    cv::UMat frame_blur, frame_thresh, frame_pyr_up, frame_pyr_down, frame_filter;
    cv::UMat frame_edge, frame_blur1, frame_gray, frame1;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(frame_gray, frame_thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    cv::medianBlur(frame_thresh, frame_blur1, 5);
    cv::Laplacian(frame_blur1, frame_edge, -1);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    long double biggest = -1;
    int biggest_index = -1;
    for (int i = 0; i < contours.size(); i++) {
        long double area = cv::contourArea(contours[i]);
        biggest = MAX(area, biggest);
        if (biggest == area) {
            biggest_index = i;
        }
    }
    frame.copyTo(frame1);
    if (biggest_index != -1) {
        cv::drawContours(frame1, contours, biggest_index, cv::Scalar(0, 255, 0));
    }
    cv::imshow("c", frame_thresh);
    cv::imshow("b", frame_edge);
    cv::imshow("a", frame1);
    cv::Mat final_mask(frame.rows, frame.cols, frame.type(), cv::Scalar(0, 0, 0));
    cv::fillPoly(final_mask, std::vector<std::vector<cv::Point>>{contours[biggest_index]}, cv::Scalar(255, 255, 255));
    cv::bitwise_and(frame, final_mask, dst);
    cv::imshow("d", final_mask);
}

void detect_black(const cv::UMat &frame, cv::UMat &dst) {
    cv::UMat frame_blur, frame_hsv, frame_thresh, frame_pyr_up, frame_pyr_down, frame_filter;
    cv::UMat frame_edge, frame1, frame_gray;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, black_low, black_up, frame_thresh);
    cv::bitwise_and(frame, frame, frame_filter, frame_thresh);
    cv::Laplacian(frame_thresh, frame_edge, -1);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    long double biggest = -1;
    int biggest_index = -1;
    for (int i = 0; i < contours.size(); i++) {
        double epsilon = 0.02 * cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx;
//        std::vector<cv::Point> hull;
        cv::approxPolyDP(contours[i], approx, epsilon, true);
//        cv::convexHull(approx, hull);
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
    cv::imshow("a", frame_thresh);
    cv::imshow("b", frame_edge);
    cv::imshow("c", frame1);
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


//    cv::fillPoly(final_mask, std::vector<std::vector<cv::Point>>{contours[biggest_index]}, cv::Scalar(255, 255, 255));
    cv::bitwise_and(frame, final_mask, dst);
    cv::imshow("d", dst);
    cv::imshow("f", final_mask);
}

void detect_while(const cv::UMat &frame, cv::UMat &dst, std::vector<cv::Point> &dst_approx) {
    cv::UMat frame_hsv, frame_pyr_up, frame_pyr_down, frame_thresh, frame_edge, frame_blur;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, white_low, white_up, frame_thresh);
    cv::Canny(frame_thresh, frame_edge, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), contour_compare);
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
//    cv::imshow("a", frame_edge);
//    cv::imshow("b", frame_thresh);
//    cv::imshow("c", frame);
}

void get_image(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::UMat frame, frame_black, frame_white;
    std::vector<cv::Point> approx;
    image_ptr->image.copyTo(frame);
    detect_black(frame, frame_black);
    detect_while(frame_black, frame_white, approx);
    if (!approx.empty()) {
        cv::drawContours(frame_white, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 0, 255), 3);;
        cv::Rect rect = cv::boundingRect(approx);
        if (rect.x < 428) {
            std::cout << "stop" << std::endl;
            /*
             * Read QR code
             */
        } else if (rect.y < 120) {
            std::cout << "go left" << std::endl;
        } else if (rect.y + rect.width > 360) {
            std::cout << "go right" << std::endl;
        } else {
            std::cout << "continue" << std::endl;
        }
    }
    cv::imshow("e", frame_white);
    cv::waitKey(1);
}