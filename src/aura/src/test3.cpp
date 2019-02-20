#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <vector>
#include <sstream>

void process();

int main() {
    process();
/*  bool _cudaSupported  = false;

//    ...
// Obtain information from the OpenCV compilation
// Here is a lot of information.
    const cv::String str = cv::getBuildInformation();

// Parse looking for "Use Cuda" or the option you are looking for.
    std::istringstream strStream(str);
    
    std::string line;
    while (std::getline(strStream, line))

    {
        // Enable this to see all the options. (Remember to remove the break)
        //std::cout << line << std::endl;

        if(line.find("NO") != std::string::npos)
        {*//*
            // Trim from elft.
            line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                                  std::not1(std::ptr_fun<int, int>(std::isspace))));

            // Trim from right.
            line.erase(line.begin(), std::find_if(line.begin(), line.end(),
                                                  std::not1(std::ptr_fun<int, int>(std::isspace))));

            // Convert to lowercase may not be necessary.
            std::transform(line.begin(), line.end(), line.begin(), ::tolower);
            if (line.find("yes") != std::string::npos)
            {
                std::cout << "USE CUDA = YES" << std::endl;
                _cudaSupported = true;
                break;
            }
        *//*
            std::cout<<line<<std::endl;
        }
    }*/
}

void process() {
    cv::ocl::setUseOpenCL(true);
    cv::VideoCapture capture;
    capture.open(0);
    cv::UMat frame, frame_blur, frame_edge, frame_gray;
    while (capture.isOpened()) {
        capture >> frame;
        cv::flip(frame, frame, 1);
        cv::GaussianBlur(frame, frame_blur, cv::Size(9, 9), -1);
        //        cv::bilateralFilter(frame, frame_blur, 9, 75, 75);
        cv::Canny(frame_blur, frame_edge, 100, 200);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(frame_edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &cnt:contours) {
            double esp = 0.03 * cv::arcLength(cnt, true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(cnt, approx, esp, true);
            if (/*approx.size() == 12*/true) {
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(255, 0, 0));
                cv::putText(frame, std::to_string(approx.size()), cv::Point(cnt[0].x - 2, cnt[0].y - 2), 1, 3,
                            cv::Scalar(0, 0, 255));
            }
        }
        cv::imshow("a", frame_edge);
        cv::imshow("b", frame);
        if (cv::waitKey(1) == 27) break;
    }
}