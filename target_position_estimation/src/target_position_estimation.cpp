#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Image.hpp"
#include "sensor_msgs/msg/CameraInfo.hpp"
#include <functional>

class TargetDetection : public rclcpp::Node {
    public:
        TargetDetection() : Node("target_detection") {
            color_image = create_subscription<std_msg::msg::String>(
                "/rqbd_camera/image",
                10,
                std::bind(
                    &TargetDetection::color_image_callback,
                    this,
                    std::placeholders::_1));

        }
    private:
        void color_image_callback(std_msgs::msg::String::ConstSharedPtr msg) {
            
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_image;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TargetDetection>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}