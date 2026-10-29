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
#include <string>


class RANSAC_ALG{
    public:
        std::vector<Eigen::Vector3f> points;
        std::vector<std::vector<Eigen::Vector3f>> points_in_planes;

        int points_num;

        int width;
        int height;
        int point_step;
        int row_step;

        static float read_float(const YAML::Node& data, std::size_t offset) {
            std::uint8_t bytes[4];

            for (std::size_t j = 0; j < 4; ++j) {
                bytes[j] = data[offset + j].as<unsigned int>();
            }

            float value;
            std::memcpy(&value, bytes, sizeof(value));
            return value;
        }

        pcl::visualization::PCLVisualizer::Ptr PlanesVis ()
        {
            //std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds;

            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0, 0, 0);

            auto it_begin = points_in_planes.begin();
            auto it_end = points_in_planes.end();

            float step = 765.0f / (it_end - it_begin);

            if (!points_in_planes.empty()) {
                for (auto it = it_begin; it != it_end - 1; ++it) {
                    auto num = it - it_begin;

                    float hue = 6.0f * static_cast<float>(num) / static_cast<float>(points_in_planes.size() - 1);

                    int section = static_cast<int>(hue);
                    float fraction = hue - section;

                    float rising = 255.0f * fraction;
                    float falling = 255.0f * (1.0f - fraction);

                    float r = 0.0f;
                    float g = 0.0f;
                    float b = 0.0f;

                    switch (section) {
                        case 0:  // Red -> yellow
                            r = 255.0f;
                            g = rising;
                            break;

                        case 1:  // Yellow -> green
                            r = falling;
                            g = 255.0f;
                            break;

                        case 2:  // Green -> cyan
                            g = 255.0f;
                            b = rising;
                            break;

                        case 3:  // Cyan -> blue
                            g = falling;
                            b = 255.0f;
                            break;

                        case 4:  // Blue -> magenta
                            r = rising;
                            b = 255.0f;
                            break;

                        default:  // Magenta -> red
                            r = 255.0f;
                            b = falling;
                            break;
                    }

                    /*if ((it - it_begin) * step < 255.0f) {
                        r = std::max(255.0f - step * num, 0.0f);
                        g = std::min(step * num, 255.0f);
                        b = 0.0f;

                    } else if ((it - it_begin) * step < 510.0f) {
                        r = 0.0f;
                        g = std::max(510.0f - step * num, 0.0f);
                        b = std::min(step * num - 255.0f, 255.0f);

                    } else if ((it - it_begin) * step < 765.0f) {
                        r = std::min(step * num - 510.0f, 255.0f);
                        g = 0.0f;
                        b = std::max(765.0f - step * num, 0.0f);
                    }*/

                    std::cout << "run: " << num << "; color: " << r << ", " << g << ", " << b << "\n";

                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                    for (std::size_t i = 0; i < (*it).size(); ++i) {
                        pcl::PointXYZ point;
                        
                        point.x = (*it)[i][0];
                        point.y = (*it)[i][1];
                        point.z = (*it)[i][2];

                        cloud->points.push_back(point);
                    }

                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, r, g, b);
                    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color, "plane" + std::to_string(num));
                    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane" + std::to_string(num));

                }

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::size_t i = 0; i < points_in_planes.back().size(); ++i) {
                    pcl::PointXYZ point;
                    
                    point.x = points_in_planes.back()[i][0];
                    point.y = points_in_planes.back()[i][1];
                    point.z = points_in_planes.back()[i][2];

                    cloud->points.push_back(point);
                }

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 0, 0);
                viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color, "rest");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rest");

            }
            

            viewer->addCoordinateSystem (1.0);
            viewer->initCameraParameters ();
            return (viewer);
           
        }

        void load_data() {
            YAML::Node cloud = YAML::LoadFile("wall_of_cubes.yaml");

            width = cloud["width"].as<int>();
            height = cloud["height"].as<int>();
            point_step = cloud["point_step"].as<int>();
            row_step = cloud["row_step"].as<int>();

            YAML::Node data = cloud["data"];

            points.reserve(width * height);

            for (std::size_t row = 0; row < height; ++row) {
                for (std::size_t col = 0; col < width; ++col) {
                    const std::size_t offset = row * row_step + col * point_step;

                    Eigen::Vector3f point {read_float(data, offset), read_float(data, offset + 4), read_float(data, offset + 8)};

                    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
                        points.push_back(point);
                    }
                }
            }

            points.shrink_to_fit();
            points_num = points.size();
        }

        void RANSAC(float distance_error_threshold, std::size_t planes_num) {
            std::size_t iter_num = 10000;
            std::size_t h = 0;

            points_in_planes.reserve(planes_num + 1);
            for (std::size_t i = 0; i < planes_num + 1; ++i) {
                points_in_planes.push_back(std::vector<Eigen::Vector3f>());
                points_in_planes[i].reserve(points_num);
            }

            while (points.size() > 0 && h < planes_num) {
                float outliers_min = std::numeric_limits<float>::infinity();

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

                    int outliers = 0;

                    for (std::size_t j = 0; j < points_num; ++j) {
                        if (j != x && j != y && j != z) {


                            Eigen::Vector3f ab = points[y] - points[x];
                            Eigen::Vector3f ac = points[z] - points[x];
                            Eigen::Vector3f n = ab.cross(ac).normalized();

                            float d = abs(points[j].dot(n) - points[x].dot(n));
                            if (d > distance_error_threshold) {
                                outliers += 1;
                            }
                        }
                    }

                    if (outliers < outliers_min) {
                        outliers_min = outliers;

                        a = points[x];
                        b = points[y];
                        c = points[z];
                    }

                    ++i;
                }

                Eigen::Vector3f ab = b - a;
                Eigen::Vector3f ac = c - a;
                Eigen::Vector3f n = ab.cross(ac).normalized();

                //points_in_planes.push_back(std::vector<Eigen::Vector3f>());

                for (auto it = points.begin(); it != points.end(); ) {
                    float distance = abs((*it).dot(n) - a.dot(n));
                    if (distance <= distance_error_threshold) {
                        points_in_planes[h].push_back((*it));
                        it = points.erase(it);
                    }
                    else {
                        ++it;
                    }
                }

                points_in_planes[h].shrink_to_fit();
                points_num = points.size();

                ++h;
            }
            if (points.size() != 0) {
                //points_in_planes.push_back(std::vector<Eigen::Vector3f>());

                points_in_planes[h].insert(
                    points_in_planes[h].end(),
                    std::make_move_iterator(points.begin()),
                    std::make_move_iterator(points.end())
                );

                points_in_planes[h].shrink_to_fit();
            }
            points_in_planes.shrink_to_fit();
            std::cout << "num planes: " << points_in_planes.size() << "\n";
        }
};


int main() {
    RANSAC_ALG ransac;

    ransac.load_data();

    ransac.RANSAC(0.0005, 40);

    pcl::visualization::PCLVisualizer::Ptr viewer = ransac.PlanesVis();

    using namespace std::chrono_literals;

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}