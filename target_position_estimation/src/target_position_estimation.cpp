#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "bagger_interfaces/msg/target_transform_array.hpp"
#include "bagger_interfaces/msg/masked_depth_array.hpp"


class TargetPositionEstimation : public rclcpp::Node
{
public:
    TargetPositionEstimation() : Node("target_position_estimation")
    {
        masked_depth_array_sub = create_subscription<bagger_interfaces::msg::MaskedDepthArray>(
            "/target_detection/masked_depth_array",
            10,
            std::bind(
                &TargetPositionEstimation::masked_depth_array_callback,
                this,
                std::placeholders::_1));

        camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rqbd_camera/camera_info",
            10,
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {camera_info = msg;}
        );

        detected_targets_pub = create_publisher<bagger_interfaces::msg::TargetTransformArray>(
            "/target_position_estimation/target_transform_array",
            10
        );

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem(0.1);

        viewer_timer = create_wall_timer(
            std::chrono::milliseconds(30),
            [this]()
            {
                if (!viewer->wasStopped())
                {
                    viewer->spinOnce(1);
                }
            }
        );
    }
private:
    bool displaying = false;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;

    rclcpp::Subscription<bagger_interfaces::msg::MaskedDepthArray>::SharedPtr masked_depth_array_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

    rclcpp::Publisher<bagger_interfaces::msg::TargetTransformArray>::SharedPtr detected_targets_pub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    pcl::visualization::PCLVisualizer::Ptr viewer {new pcl::visualization::PCLVisualizer ("3D Viewer")};
    
    rclcpp::TimerBase::SharedPtr viewer_timer;

    void masked_depth_array_callback(const bagger_interfaces::msg::MaskedDepthArray& msg)
    {
        if (!camera_info)
        {
            return;
        }

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
        filter.setMeanK(10);
        filter.setStddevMulThresh(1.0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        viewer->removeAllShapes();

        bagger_interfaces::msg::TargetTransformArray targets_msg;

        for (std::vector<sensor_msgs::msg::Image>::const_iterator it = msg.masked_depth.begin(); it != msg.masked_depth.end(); ++it)
        {
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(*it, "32FC1");
            cloud = depth_to_cloud(depth_ptr->image, *camera_info);

            filter.setInputCloud(cloud);
            filter.filter(*filtered_cloud);

            Eigen::Vector4f center;

            if (pcl::compute3DCentroid(*filtered_cloud, center) <= 20)
            {
                continue;
            }

            geometry_msgs::msg::TransformStamped target;

            target.header = it->header;
            target.child_frame_id = "target_" + std::to_string(it - msg.masked_depth.begin());
            target.transform.translation.x = center[0];
            target.transform.translation.y = center[1];
            target.transform.translation.z = center[2];
            target.transform.rotation.w = 1.0;

            tf_broadcaster->sendTransform(target);

            pcl::PointXYZ centroid;
            centroid.x = center[0];
            centroid.y = center[1];
            centroid.z = center[2];

            std::string marker_id = "centroid_" + std::to_string(it - msg.masked_depth.begin());

            viewer->addSphere(
                centroid,
                0.03,
                1.0, 0.0, 0.0,
                marker_id
            );

            targets_msg.target.push_back(std::move(target));

            *combined_cloud += *filtered_cloud;
        }

        detected_targets_pub->publish(targets_msg);

        if (combined_cloud->empty())
        {
            viewer->removePointCloud("combined_cloud");
        }
        else if (!viewer->updatePointCloud<pcl::PointXYZ>(combined_cloud, "combined_cloud"))
        {
            viewer->addPointCloud<pcl::PointXYZ>(combined_cloud, "combined_cloud");
            viewer->resetCamera();
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_to_cloud(
        const cv::Mat& depth,
        const sensor_msgs::msg::CameraInfo& camera_info)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        const double fx = camera_info.k[0];
        const double fy = camera_info.k[4];
        const double cx = camera_info.k[2];
        const double cy = camera_info.k[5];

        for (int v = 0; v < depth.rows; ++v)
        {
            for (int u = 0; u < depth.cols; ++u)
            {
                const float z = depth.at<float>(v, u);

                if (!std::isfinite(z) || z <= 0.0f)
                {
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TargetPositionEstimation>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}