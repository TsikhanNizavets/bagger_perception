#include <iostream>
#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>
#include <pcl-1.14/pcl/features/normal_3d.h>
#include <pcl-1.14/pcl/io/pcd_io.h>
#include <pcl-1.14/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.14/pcl/console/parse.h>
#include <pcl-1.14/pcl/search/kdtree.h>
#include <eigen3/Eigen/Dense>
#include <limits>

float read_float(const YAML::Node& data, std::size_t offset) {
    std::uint8_t bytes[4];

    for (std::size_t j = 0; j < 4; ++j) {
        bytes[j] = data[offset + j].as<unsigned int>();
    }

    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud1, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(cloud2, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, cloud1_color, "cloud1");
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, cloud2_color, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud2");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main() {
    YAML::Node cloud = YAML::LoadFile("points_cloud.yaml");

    int width = cloud["width"].as<int>();
    int height = cloud["height"].as<int>();
    int point_step = cloud["point_step"].as<int>();
    int row_step = cloud["row_step"].as<int>();

    YAML::Node data = cloud["data"];

    std::cout << data[0].as<int>() << "\n";

    struct Point {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    std::vector<Eigen::Vector3f> points;
    points.reserve(width * height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::size_t row = 0; row < height; ++row) {
        for (std::size_t col = 0; col < width; ++col) {
            const std::size_t offset = row * row_step + col * point_step;

            Eigen::Vector3f point {read_float(data, offset), read_float(data, offset + 4), read_float(data, offset + 8)};

            if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
                points.push_back(point);
            }

            pcl::PointXYZ basic_point;
            basic_point.x = read_float(data, offset);
            basic_point.y = read_float(data, offset + 4);
            basic_point.z = read_float(data, offset + 8);
            if (std::isfinite(basic_point.x) && std::isfinite(basic_point.y) && std::isfinite(basic_point.z)) {
                basic_cloud_ptr->points.push_back(basic_point);
            }
        }
    }

    points.shrink_to_fit();
    int points_num = points.size();

    float error = std::numeric_limits<float>::infinity();
    constexpr float error_threshold = 0.0005f;

    std::size_t iter_num = 1;
    std::size_t i = 0;

    Eigen::Vector3f a;
    Eigen::Vector3f b;
    Eigen::Vector3f c;

    while (i < iter_num) {
        int x = rand() % points_num;
        int y = rand() % (points_num - 1);
        int z = rand() % (points_num - 2);
        y += y >= x;
        z += z >= std::min(x, y);
        z += z >= std::max(x, y);

        float e = 0.0f;

        for (std::size_t j; j < points_num; ++j) {
            if (j != x && j != y && j != z) {


                Eigen::Vector3f ab = points[y] - points[x];
                Eigen::Vector3f ac = points[z] - points[x];
                Eigen::Vector3f n = ab.cross(ac).normalized();

                float d = abs(points[j].dot(n) - points[x].dot(n));
                if (d > error_threshold) {
                    e += d;
                }
            }
        }

        if (e < error) {
            error = e;

            a = points[x];
            b = points[y];
            c = points[z];
        }

        ++i;
    }

    std::vector<Eigen::Vector3f> points_in_plane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr red_points (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::size_t j; j < points_num; ++j) {
        Eigen::Vector3f ab = b - a;
        Eigen::Vector3f ac = c - a;
        Eigen::Vector3f n = ab.cross(ac).normalized();

        float d = abs(points[j].dot(n) - a.dot(n));
        if (d <= error_threshold) {
            points_in_plane.push_back(points[j]);
            red_points->points.push_back(basic_cloud_ptr->points[j]);
        }
    }

    std::cout << "size: " << points_in_plane.size() << "\n";
    std::cout << "error: " << error << "\n";

    /*Eigen::Vector3f a_result = points[1];
    Eigen::Vector3f b_result = points[100];
    Eigen::Vector3f c_result = points[10000];

    points.erase(points.begin() + 10000);
    points.erase(points.begin() + 100);
    points.erase(points.begin() + 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr red_points (new pcl::PointCloud<pcl::PointXYZ>);
    red_points->points.push_back(basic_cloud_ptr->points[1]);
    red_points->points.push_back(basic_cloud_ptr->points[100]);
    red_points->points.push_back(basic_cloud_ptr->points[10000]);

    basic_cloud_ptr->points.erase(basic_cloud_ptr->points.begin() + 10000);
    basic_cloud_ptr->points.erase(basic_cloud_ptr->points.begin() + 100);
    basic_cloud_ptr->points.erase(basic_cloud_ptr->points.begin() + 1);

    red_points->width = red_points->size ();
    red_points->height = 1;*/



    /*for (std::size_t i = 0; i < width * height; ++i) {
        for (std::size_t j = 0; j < width * height; ++j) {
            for (std::size_t k = 0; k < width * height; ++k) {
                if (i == j || i == k || j == k) {
                    continue;
                }
                else {
                    Eigen::Vector3f a = points[i];
                    Eigen::Vector3f b = points[j];
                    Eigen::Vector3f c = points[k];

                    Eigen::Vector3f ab = b - a;
                    Eigen::Vector3f ac = c - a;

                    Eigen::Vector3f n = ab.cross(ac).normalized();

                    float e = 0.0f;

                    for (std::size_t l = 0; l < width * height; ++l) {
                        if (l != i && l != j && l != k) {
                            e = e + abs(points[l].dot(n) - a.dot(n));
                        }
                    }

                    if (e < error) {
                        error = e;
                        
                        a_result = a;
                        b_result = b;
                        c_result = c;
                    }
                }
            }
        }
    }*/

    //std::cout << a_result << "\n" << b_result << "\n" << c_result << "\n";

    basic_cloud_ptr->width = basic_cloud_ptr->size ();
    basic_cloud_ptr->height = 1;

     pcl::visualization::PCLVisualizer::Ptr viewer = customColourVis(basic_cloud_ptr, red_points);

    using namespace std::chrono_literals;
    
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }


    /*std::ifstream file("points_cloud.yaml");
    
    if (!file.is_open()) {
        std::cerr << "Could not open the file\n";
        return 1;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::cout << line << '\n';
    }*/

    return 0;
}