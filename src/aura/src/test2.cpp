#include <ros/ros.h>
#include <iostream>
#include "../include/aura/controller.h"
#include "../include/aura/image_process.h"


int main(int argc, char **argv) {
    cv::ocl::setUseOpenCL(true);
    ros::init(argc, argv, "test_image");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    Controller *controller = new Controller(nh);
    ImageProcess imageProcess(nullptr, "/bebop/image_raw", "/barcode", controller);
    ros::spin();
}
