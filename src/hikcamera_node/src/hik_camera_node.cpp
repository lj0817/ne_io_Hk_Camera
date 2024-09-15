#include "hik_camera_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

ImagePublisher::ImagePublisher(const std::string &name)
: Node(name),
  hk_cam_(),  
  image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10)) 
{
    RCLCPP_INFO(this->get_logger(), "%s 节点已经启动.", name.c_str());
   
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ImagePublisher::hikImgCallback, this)
    );

    hk_cam_.start();  
}



/*void ImagePublisher::hikImgCallback()
{
    cv::Mat src = cv::Mat(hk_cam_.stImageInfo.stFrameInfo.nHeight, hk_cam_.stImageInfo.stFrameInfo.nWidth, CV_8UC1);
    memcpy(src.data, hk_cam_.stImageInfo.pBufAddr, hk_cam_.stImageInfo.stFrameInfo.nWidth * hk_cam_.stImageInfo.stFrameInfo.nHeight);
    cv::Mat bgr_image;
    cv::cvtColor(src, bgr_image, cv::COLOR_BayerGR2BGR_EA);

   
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
    image_publisher_->publish(*msg); 
}*/



void ImagePublisher::hikImgCallback()
{
    // 检查图像数据是否有效
    if (hk_cam_.pData == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Image data pointer is null.");
        return;
    }

    if (hk_cam_.stImageInfo.stFrameInfo.nWidth <= 0 || hk_cam_.stImageInfo.stFrameInfo.nHeight <= 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid image dimensions: Width=%d, Height=%d",
                    hk_cam_.stImageInfo.stFrameInfo.nWidth, hk_cam_.stImageInfo.stFrameInfo.nHeight);
        return;
    }

    // 创建 OpenCV 单通道图像
    cv::Mat src(hk_cam_.stImageInfo.stFrameInfo.nHeight, hk_cam_.stImageInfo.stFrameInfo.nWidth, CV_8UC1);
    std::memcpy(src.data, hk_cam_.pData, hk_cam_.stImageInfo.stFrameInfo.nWidth * hk_cam_.stImageInfo.stFrameInfo.nHeight);

    // 检查图像是否为空
    if (src.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Source image is empty after memcpy.");
        return;
    }

    // 输出图像信息用于调试
    RCLCPP_INFO(this->get_logger(), "Source image: Width=%d, Height=%d, Channels=%d",
                src.cols, src.rows, src.channels());

    // 尝试进行颜色转换
    cv::Mat bgr_image;
    try {
        // 根据实际图像格式选择正确的颜色转换代码
        cv::cvtColor(src, bgr_image, cv::COLOR_BayerGR2BGR_EA); // 如果原始图像是 Bayer 格式
        // cv::cvtColor(src, bgr_image, cv::COLOR_GRAY2BGR);  // 如果原始图像是灰度图像
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error during color conversion: %s", e.what());
        return;
    }

    // 使用 cv_bridge 转换 OpenCV 图像到 ROS 消息
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
    image_publisher_->publish(*msg);  // 发布图像消息
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>("camera_node");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
