#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "bagger_interfaces/msg/masked_depth_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <functional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>
#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <pcl/common/centroid.h>
#include <string>


class TargetPositionEstimation : public rclcpp::Node {
    public:
        TargetPositionEstimation() : Node("target_position_estimation") {
            color_image_sub = create_subscription<bagger_interfaces::msg::MaskedDepthArray>(
                "/target_detection/masked_depths",
                10,
                std::bind(
                    &TargetPositionEstimation::color_image_callback,
                    this,
                    std::placeholders::_1));

            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            viewer->setBackgroundColor (0, 0, 0);
            viewer->addCoordinateSystem(0.1);

            viewer_timer = create_wall_timer(
                std::chrono::milliseconds(30),
                [this]() {
                    if (!viewer->wasStopped()) {
                        viewer->spinOnce(1);
                    }
                }
            );

            camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/rqbd_camera/camera_info",
                10,
                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {camera_info = msg;}
            );
        }
    private:
        void color_image_callback(const bagger_interfaces::msg::MaskedDepthArray& msg) {
            if (!camera_info) {
                return;
            }

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
            filter.setMeanK(10);
            filter.setStddevMulThresh(1.0);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZ>);

            // RCLCPP_INFO(this->get_logger(), "message arrived outside the loop");
            for (std::vector<sensor_msgs::msg::Image>::const_iterator it = msg.masked_depths.begin(); it != msg.masked_depths.end(); ++it) {
                // ros msg to cv2
                // to gray
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(*it, "32FC1");
                cloud = depth_to_cloud(depth_ptr->image, *camera_info);

                if (cloud->size() <= 20) {
                    continue;
                }


                filter.setInputCloud(cloud);
                filter.filter(*filtered_cloud);

                *combined_cloud += *filtered_cloud;

                Eigen::Vector4f center;

                if (pcl::compute3DCentroid(*filtered_cloud, center) == 0) {
                    continue;
                }

                geometry_msgs::msg::TransformStamped transform;

                transform.header = it->header;

                transform.header = it->header;

                transform.child_frame_id = "detected_cube_" + std::to_string(it - msg.masked_depths.begin());

                transform.transform.translation.x = center[0];
                transform.transform.translation.y = center[1];
                transform.transform.translation.z = center[2];

                transform.transform.rotation.w = 1.0;

                tf_broadcaster->sendTransform(transform);

                // RCLCPP_INFO(this->get_logger(), "message arrived inside the loop");
                

                // calculate mean values and save apprx. cube's coordinates

                // ByteTrack???

            }

            // RCLCPP_INFO_STREAM(this->get_logger(), (*depth_ptr).image);
            if (combined_cloud->empty()) {
                viewer->removePointCloud("combined_cloud");
            }
            else if (!viewer->updatePointCloud<pcl::PointXYZ>(combined_cloud, "combined_cloud")) {
                viewer->addPointCloud<pcl::PointXYZ>(combined_cloud, "combined_cloud");
                viewer->resetCamera();
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr depth_to_cloud(
            const cv::Mat& depth,
            const sensor_msgs::msg::CameraInfo& camera_info
        ) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

            const double fx = camera_info.k[0];
            const double fy = camera_info.k[4];
            const double cx = camera_info.k[2];
            const double cy = camera_info.k[5];

            for (int v = 0; v < depth.rows; ++v) {
                for (int u = 0; u < depth.cols; ++u) {
                    const float z = depth.at<float>(v, u);

                    if (!std::isfinite(z) || z <= 0.0f) {
                        continue;
                    }

                    pcl::PointXYZ point;
                    point.x = (u - cx) * z / fx;
                    point.y = (v - cy) * z / fy;
                    point.z = z;

                    cloud->push_back(point);
                }
            }

            return cloud;
        }

        rclcpp::Subscription<bagger_interfaces::msg::MaskedDepthArray>::SharedPtr color_image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
        rclcpp::TimerBase::SharedPtr viewer_timer;
        bool displaying = false;
        pcl::visualization::PCLVisualizer::Ptr viewer {new pcl::visualization::PCLVisualizer ("3D Viewer")};
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TargetPositionEstimation>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}