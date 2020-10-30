#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
         sensor_msgs::Image> SyncPolicy;

class projectionLidarLines {
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string result_str;
    std::string cam_config_file_path;
    std::string base_folder_name;

    Eigen::Matrix4d C_T_L;

    cv::Mat D, K;
    int image_width, image_height;

    std::vector<cv::Point3d> objectPoints_L;
    std::vector<cv::Point3d> objectPoints_C;

    std::vector<cv::Point2d> imagePoints;
    double fov_x, fov_y;

    cv::Mat c_R_l, tvec;
    cv::Mat rvec;
    Eigen::Matrix3d C_R_L;
    Eigen::Vector3d C_t_L;

    std::string node_name;
    double x_threshold;
    int view_no;

    std::string camera_name;

public:
    projectionLidarLines(ros::NodeHandle n) {
        node_name = ros::this_node::getName();
        nh = n;
        cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/cloud_in", 1);
        image_sub = new
                message_filters::Subscriber
                        <sensor_msgs::Image>(nh, "/image_in", 1);

        sync = new
                message_filters::Synchronizer
                        <SyncPolicy>(SyncPolicy(10),
                                     *cloud_sub, *image_sub);
        sync->registerCallback(boost::bind(&projectionLidarLines::callback,
                                           this, _1, _2));

        result_str = readParam<std::string>(nh, "result_str");
        cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");
        base_folder_name = readParam<std::string>(nh, "base_folder_name");
        x_threshold = readParam<double>(nh, "x_threshold");
        camera_name = readParam<std::string>(nh, "camera_name");
        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_cam_config.isOpened());
        K = cv::Mat::zeros(3, 3, CV_64F);
        D = cv::Mat::zeros(5, 1, CV_64F);
        fs_cam_config["image_height"] >> image_height;
        fs_cam_config["image_width"] >> image_width;
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["k3"] >> D.at<double>(4);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);

        ROS_INFO_STREAM("camMat :  \n" << K );
        ROS_INFO_STREAM("distMat :  \n" << D );
        std::cout << std::endl;

        fov_x = 2*atan2(image_width, 2*K.at<double>(0, 0))*180/CV_PI;
        fov_y = 2*atan2(image_height, 2*K.at<double>(1, 1))*180/CV_PI;

        C_T_L = Eigen::Matrix4d::Identity();

        std::ifstream myReadFile(result_str.c_str());
        std::string word;
        int i = 0;
        int j = 0;
        while (myReadFile >> word){
            C_T_L(i, j) = atof(word.c_str());
            j++;
            if(j>3) {
                j = 0;
                i++;
            }
        }

        std::cout << C_T_L << std::endl;
        C_R_L = C_T_L.block(0, 0, 3, 3);
        C_t_L = C_T_L.block(0, 3, 3, 1);

        cv::eigen2cv(C_R_L, c_R_l);
        cv::Rodrigues(c_R_l, rvec);
        cv::eigen2cv(C_t_L, tvec);
        view_no = 0;
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                  const sensor_msgs::ImageConstPtr &image_msg) {
        objectPoints_L.clear();
        objectPoints_C.clear();
        imagePoints.clear();
        cv::Mat image_in;
        try {
            image_in = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
        pcl::fromROSMsg(*cloud_msg, cloud_pcl);

        double max_range, min_range;
        max_range = -INFINITY;
        min_range = INFINITY;

        for(int i = 0; i < cloud_pcl.points.size(); i++) {

            if(cloud_pcl.points[i].x < 0 || cloud_pcl.points[i].x > x_threshold)
                continue;

            Eigen::Vector4d pointCloud_L;
            pointCloud_L[0] = cloud_pcl.points[i].x;
            pointCloud_L[1] = cloud_pcl.points[i].y;
            pointCloud_L[2] = cloud_pcl.points[i].z;
            pointCloud_L[3] = 1;

            Eigen::Vector3d pointCloud_C;
            pointCloud_C = C_T_L.block(0, 0, 3, 4) * pointCloud_L;

            double X = pointCloud_C[0];
            double Y = pointCloud_C[1];
            double Z = pointCloud_C[2];

            double Xangle = atan2(X, Z)*180/CV_PI;
            double Yangle = atan2(Y, Z)*180/CV_PI;

            if(Xangle < -fov_x/2 || Xangle > fov_x/2)
                continue;

            if(Yangle < -fov_y/2 || Yangle > fov_y/2)
                continue;

            objectPoints_L.push_back(cv::Point3d(pointCloud_L[0], pointCloud_L[1], pointCloud_L[2]));
            objectPoints_C.push_back(cv::Point3d(X, Y, Z));
            double range = sqrt(X*X + Y*Y + Z*Z);

            if(range > max_range) {
                max_range = range;
            }
            if(range < min_range) {
                min_range = range;
            }
        }

        cv::projectPoints(objectPoints_L, rvec, tvec, K, D, imagePoints, cv::noArray());
        for(int i = 0; i < imagePoints.size(); i++) {
            double X = objectPoints_C[i].x;
            double Y = objectPoints_C[i].y;
            double Z = objectPoints_C[i].z;

            double range = sqrt(X*X + Y*Y + Z*Z);
            double red_field = 255*(range - min_range)/(max_range - min_range);
            double green_field = 255*(max_range - range)/(max_range - min_range);

            cv::circle(image_in, imagePoints[i], 5,
                    cv::Scalar(0, red_field, green_field), -1, 1, 0);
        }
        if(camera_name=="basler") {
            cv::resize(image_in, image_in, cv::Size(image_in.cols/2, image_in.rows/2));
            cv::imshow(node_name + " view", image_in);
        } else {
            cv::imshow(node_name + " view", image_in);
        }
        cv::imwrite(base_folder_name+std::to_string(view_no)+".png", image_in);
        cv::waitKey(1);
        view_no++;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "projection_node");
    ros::NodeHandle nh("~");
    projectionLidarLines pLL(nh);
    ros::spin();
    return 0;
}