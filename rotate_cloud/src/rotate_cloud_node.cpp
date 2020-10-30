#define PCL_NO_PRECOMPILE

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

struct PointXYZIR8Y
{
    PCL_ADD_POINT4D
    ; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint8_t ring; ///< laser ring number
    float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZIR8Y,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (uint8_t, ring, ring))

class rotateCloud {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

public:
    rotateCloud(ros::NodeHandle nh_) {
        nh = nh_;
        cloud_sub = nh.subscribe("/cloud_in", 1,
                                     &rotateCloud::callback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        pcl::PointCloud<PointXYZIR8Y>::Ptr
                cloud_in(new pcl::PointCloud<PointXYZIR8Y>);
        pcl::PointCloud<PointXYZIR8Y>::Ptr
                cloud_in_transformed(new pcl::PointCloud<PointXYZIR8Y>);
        pcl::fromROSMsg(*cloud_msg, *cloud_in);
        double yaw_angle = 90*M_PI/180;
        Eigen::Matrix4d transformation_mat = Eigen::Matrix4d::Identity(); // transformation_mat = new_T_old
        transformation_mat(0, 0) = cos(yaw_angle); transformation_mat(0, 1) = sin(yaw_angle); transformation_mat(0, 2) = 0;
        transformation_mat(1, 0) = -sin(yaw_angle); transformation_mat(1, 1) = cos(yaw_angle); transformation_mat(1, 2) = 0;
        transformation_mat(2, 0) = 0; transformation_mat(2, 1) = 0; transformation_mat(2, 2) = 1;
        pcl::transformPointCloud(*cloud_in, *cloud_in_transformed, transformation_mat); // X_new = new_T_old X_old
        sensor_msgs::PointCloud2 cloud_out_ros;
        pcl::toROSMsg(*cloud_in_transformed, cloud_out_ros);
        cloud_out_ros.header.stamp = ros::Time::now();
        cloud_out_ros.header.frame_id = cloud_msg->header.frame_id;
        cloud_pub.publish(cloud_out_ros);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_cloud_node");
    ros::NodeHandle nh("~");
    rotateCloud rC(nh);
    ros::spin();
}
