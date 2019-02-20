#include "../include/aura/image_process.h"

ImageProcess::ImageProcess(const ros::NodeHandlePtr &nh, const std::string &image_topic, const std::string &qr_topic,
                           Controller *controller) {
    if (nh == nullptr) {
        ImageProcess::nh = ros::NodeHandlePtr(new ros::NodeHandle);
    } else {
        ImageProcess::nh = nh;
    }
    ImageProcess::controller = controller;
    ImageProcess::qr_topic = new std::string(qr_topic);
    ImageProcess::black_low = new cv::Scalar(0, 0, 0);
    ImageProcess::black_up = new cv::Scalar(180, 255, 30);
    ImageProcess::white_low = new cv::Scalar(0, 0, 50);
    ImageProcess::white_up = new cv::Scalar(180, 100, 255);
//    ImageProcess::img_sub = new ros::Subscriber(
//            ImageProcess::nh->subscribe(image_topic, 1000, &ImageProcess::get_image, this));
}

double ImageProcess::angle(const cv::Point &pt1, const cv::Point &pt2, const cv::Point &pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

bool ImageProcess::operator()(const std::vector<cv::Point> &p1, const std::vector<cv::Point> &p2) {
    double e1 = 0.02 * cv::arcLength(p1, true);
    double e2 = 0.02 * cv::arcLength(p2, true);
    std::vector<cv::Point> ap1, ap2;
    cv::approxPolyDP(p1, ap1, e1, true);
    cv::approxPolyDP(p2, ap2, e2, true);
    double area1 = cv::contourArea(ap1);
    double area2 = cv::contourArea(ap2);
    return area1 > area2;
}

void ImageProcess::detect_black(const cv::UMat &frame, cv::UMat &dst) {
    cv::UMat frame_blur, frame_hsv, frame_thresh, frame_pyr_up, frame_pyr_down, frame_filter;
    cv::UMat frame_edge, frame1, frame_gray;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, *ImageProcess::black_low, *ImageProcess::black_up, frame_thresh);
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

void ImageProcess::detect_while(const cv::UMat &frame, cv::UMat &dst, std::vector<cv::Point> &dst_approx) {
    cv::UMat frame_hsv, frame_pyr_up, frame_pyr_down, frame_thresh, frame_edge, frame_blur;
    cv::pyrDown(frame, frame_pyr_down);
    cv::pyrUp(frame_pyr_down, frame_pyr_up);
    cv::GaussianBlur(frame_pyr_up, frame_blur, cv::Size(5, 5), -1);
    cv::cvtColor(frame_blur, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, *ImageProcess::white_low, *ImageProcess::white_up, frame_thresh);
    cv::Canny(frame_thresh, frame_edge, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), *this);
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

void ImageProcess::process(const std::vector<cv::Point> &approx,const cv::UMat &frame_white) {
    if (!approx.empty()) {
        cv::drawContours(frame_white, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 0, 255), 3);;
        cv::Rect rect = cv::boundingRect(approx);
        std_msgs::StringConstPtr code = ros::topic::waitForMessage<std_msgs::String>
                (*ImageProcess::qr_topic, *ImageProcess::nh, ros::Duration(1));
        if (code != nullptr) {
            ImageProcess::qrHandler = new QRHandler(code);
            qrHandler->compute();
            if (qrHandler->is_goal()) {
                std::cout << "TARGET FOUND " << qrHandler->get_goal() << std::endl;
            }
            //todo Go to which direction???
            std::cout << qrHandler->get_all_targets() << std::endl;
        }
        if (rect.x < 428) {
            std::cout << "stop" << std::endl << "searching qr code..." << std::endl;
            std_msgs::StringConstPtr code1 = ros::topic::waitForMessage<std_msgs::String>
                    (*ImageProcess::qr_topic, *ImageProcess::nh, ros::Duration(2));
            if (code1 == nullptr) {
                std::cout << "NO QR WHAT SHOULD ID NOW?!!!" << std::endl;
                //todo move to exit this point
            } else {
                ImageProcess::qrHandler = new QRHandler(code1);
                qrHandler->compute();
                if (qrHandler->is_goal()) {
                    std::cout << "TARGET FOUND " << qrHandler->get_goal() << std::endl;
                }
                //todo Go to which direction???
                std::cout << qrHandler->get_all_targets() << std::endl;
            }
        } else if (rect.y < 120) {
            std::cout << "go left" << std::endl;
//            ImageProcess::controller->go_left(1);
        } else if (rect.y + rect.width > 360) {
            std::cout << "go right" << std::endl;
//            ImageProcess::controller->go_right(1);
        } else {
            std::cout << "continue" << std::endl;
//            ImageProcess::controller->go_forward(1);
        }
    }
}

void ImageProcess::get_image(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::UMat frame, frame_black, frame_white;
    std::vector<cv::Point> approx;
    image_ptr->image.copyTo(frame);
    detect_black(frame, frame_black);
    detect_while(frame_black, frame_white, approx);
    ImageProcess::process(approx , frame_white);
    cv::imshow("e", frame_white);
    cv::waitKey(1);
}
