#ifndef HIK_CAMERA_NODE_HPP
#define HIK_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

#include "hik_camera_io.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
   ImagePublisher(const std::string &name);

private:
    void hikImgCallback();

    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    
    std::unique_ptr<cv::Mat> src;
    
    HkCam hk_cam_; 

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 
