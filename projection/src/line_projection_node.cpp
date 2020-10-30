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

#include "line_msg/line.h"

#include <iostream>
#include <fstream>

typedef message_filters::sync_policies::ApproximateTime
        <line_msg::line,
         line_msg::line,
         line_msg::line,
         line_msg::line,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2> SyncPolicy1;

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::Image> SyncPolicy2;

class projectionLidarLines {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<line_msg::line> *line1_image_sub;
    message_filters::Subscriber<line_msg::line> *line2_image_sub;
    message_filters::Subscriber<line_msg::line> *line3_image_sub;
    message_filters::Subscriber<line_msg::line> *line4_image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line1_cloud_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line2_cloud_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line3_cloud_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line4_cloud_sub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *line1_cloud_sub2;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line2_cloud_sub2;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line3_cloud_sub2;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line4_cloud_sub2;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *edge_cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;

    message_filters::Synchronizer<SyncPolicy1> *sync1;
    message_filters::Synchronizer<SyncPolicy2> *sync2;

    std::string result_str;
    std::string cam_config_file_path;
    std::string base_folder_name;

    Eigen::Matrix4d C_T_L;

    cv::Mat D, K;
    int image_width, image_height;


    double fov_x, fov_y;

    cv::Mat c_R_l, tvec;
    cv::Mat rvec;
    Eigen::Matrix3d C_R_L;
    Eigen::Vector3d C_t_L;

    double dist_avg;
    int no_of_frames;

    std::string node_name;
    int view_no;

    std::string target_config_file_path;
    double side_len;
    std::string csv_file_name;
    std::ofstream csv_file;

    std::string camera_name;

    cv::Point2f line1_start, line1_end;
    cv::Point2f line2_start, line2_end;
    cv::Point2f line3_start, line3_end;
    cv::Point2f line4_start, line4_end;
    cv::Mat distortedImg;
    cv::Mat undistortedImg;

public:
    projectionLidarLines(ros::NodeHandle n) {
        nh = n;
        node_name = ros::this_node::getName();
        createSubscribers();


        result_str = readParam<std::string>(nh, "result_str");
        cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");
        base_folder_name = readParam<std::string>(nh, "base_folder_name");
        csv_file_name = readParam<std::string>(nh, "csv_file_name");
        camera_name = readParam<std::string>(nh, "camera_name");
        csv_file.open(csv_file_name);

        target_config_file_path = readParam<std::string>(nh, "target_config_file_path");
        cv::FileStorage fs_target_config(target_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_target_config.isOpened());
        fs_target_config["side_len"] >> side_len;

        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_cam_config.isOpened());
        K = cv::Mat::eye(3, 3, CV_64F);
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

        no_of_frames = 0;
        dist_avg = 0;
        view_no = 0;
    }
    ~projectionLidarLines() {
        csv_file.close();
    }
    void createSubscribers() {
        line1_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image1", 1);
        line2_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image2", 1);
        line3_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image3", 1);
        line4_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image4", 1);
        line1_cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line1_out", 1);
        line2_cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line2_out", 1);
        line3_cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line3_out", 1);
        line4_cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line4_out", 1);

        line1_cloud_sub2 = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line1_out", 1);
        line2_cloud_sub2 = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line2_out", 1);
        line3_cloud_sub2 = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line3_out", 1);
        line4_cloud_sub2 = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line4_out", 1);
        image_sub = new
                message_filters::Subscriber
                        <sensor_msgs::Image>(nh, "/image_in", 1);

        sync1 = new message_filters::Synchronizer
                <SyncPolicy1>(SyncPolicy1(10),
                              *line1_image_sub,
                              *line2_image_sub,
                              *line3_image_sub,
                              *line4_image_sub,
                              *line1_cloud_sub,
                              *line2_cloud_sub,
                              *line3_cloud_sub,
                              *line4_cloud_sub);
        sync1->registerCallback(boost::bind(&projectionLidarLines::callbackLines,
                                            this, _1, _2, _3, _4,
                                                  _5, _6, _7, _8));

        sync2 = new message_filters::Synchronizer
                <SyncPolicy2>(SyncPolicy2(10),
                              *line1_cloud_sub2,
                              *line2_cloud_sub2,
                              *line3_cloud_sub2,
                              *line4_cloud_sub2,
                              *image_sub);
        sync2->registerCallback(boost::bind(&projectionLidarLines::callback,
                                            this, _1, _2, _3, _4, _5));
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

    std::vector<cv::Point2d> getProjectedPts(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
        std::vector<cv::Point3d> objectPoints_L;
        std::vector<cv::Point2d> imagePoints;
        for(int i = 0; i < cloud_in.points.size(); i++) {
//            if(cloud_in.points[i].x < 0 || cloud_in.points[i].x > 3)
//                continue;
//
            Eigen::Vector4d pointCloud_L;
            pointCloud_L[0] = cloud_in.points[i].x;
            pointCloud_L[1] = cloud_in.points[i].y;
            pointCloud_L[2] = cloud_in.points[i].z;
            pointCloud_L[3] = 1;
            objectPoints_L.push_back(cv::Point3d(pointCloud_L[0], pointCloud_L[1], pointCloud_L[2]));
        }
        if(objectPoints_L.size() > 0)
            cv::projectPoints(objectPoints_L, rvec, tvec, K, cv::Mat::zeros(cv::Size(1, 5), CV_64F), imagePoints, cv::noArray());
        else
            ROS_ERROR("objectPoints_L.size() <= 0");
        return imagePoints;
    }

    cv::Vec3f getEqnOfLine(cv::Vec4f line) {
        double x_a = line[0];
        double y_a = line[1];
        double x_b = line[2];
        double y_b = line[3];

        if(x_a == y_a && x_b == y_b) {
            return cv::Vec3f(0, 0, 0);
        } else if(x_a == x_b) {
            // eqn: x = x_a or x = x_b
            return cv::Vec3f(1, 0, -x_a);
        } else if(y_a == y_b){
            // eqn: y = y_a or y = y_b
            return cv::Vec3f(0, 1, -y_a);
        } else {
            double m = (y_b - y_a)/(x_b - x_a);
            double a = m;
            double b = -1;
            double c = y_a - m*x_a;
            return cv::Vec3f(a, b, c);
        }
    }

    double distanceFromLine(cv::Vec3f eqn, cv::Point2f pt) {
        float a = eqn(0);
        float b = eqn(1);
        float c = eqn(2);
        float x_0 = pt.x;
        float y_0 = pt.y;
        double dist = fabs(a*x_0+b*y_0+c)/sqrt(a*a+b*b);
    }

    cv::Point2f getIntersection(cv::Vec3f line_1, cv::Vec3f line_2) {
        cv::Vec3f intersection = line_1.cross(line_2);
        cv::Point2f pt(intersection[0]/intersection[2],
                       intersection[1]/intersection[2]);
        return pt;
    }

    bool isWithinImage(cv::Point2f query_pt) {
        double u = query_pt.x;
        double v = query_pt.y;
        if (u < 0 || v < 0)
            return false;
        if (u > image_width-1 || v > image_height-1)
            return false;
        return true;
    }

    Eigen::Vector4d getPose(cv::Point2f pt1, cv::Point2f pt2,
                 cv::Point2f pt3, cv::Point2f pt4) {
        std::vector<cv::Point2f> imagePoints;
        imagePoints.push_back(pt1);
        imagePoints.push_back(pt2);
        imagePoints.push_back(pt3);
        imagePoints.push_back(pt4);

        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(0, 0, 0));
        objectPoints.push_back(cv::Point3f(0, side_len, 0));
        objectPoints.push_back(cv::Point3f(side_len, side_len, 0));
        objectPoints.push_back(cv::Point3f(side_len, 0, 0));

        cv::Mat rvec_cv(3, 1, CV_64FC1);
        cv::Mat tvec_cv(3, 1, CV_64FC1);

        cv::solvePnP(objectPoints, imagePoints, K, D, rvec_cv, tvec_cv);
        Eigen::Vector3d rvec_eig;
        rvec_eig(0) = rvec_cv.at<double>(0);
        rvec_eig(1) = rvec_cv.at<double>(1);
        rvec_eig(2) = rvec_cv.at<double>(2);
        double theta = rvec_eig.norm();
        double x = tvec_cv.at<double>(0);
        double y = tvec_cv.at<double>(1);
        double z = tvec_cv.at<double>(2);
        Eigen::Vector4d pose_out(x, y, z, theta);
        return pose_out;
    }

    void drawLinesOnImage() {
        ROS_INFO_STREAM("Drawing Lines");
        cv::line(undistortedImg, line1_start, line1_end, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        cv::line(undistortedImg, line2_start, line2_end, cv::Scalar(0, 0, 255), 2, cv::LINE_8);
        cv::line(undistortedImg, line3_start, line3_end, cv::Scalar(255, 255, 0), 2, cv::LINE_8);
        cv::line(undistortedImg, line4_start, line4_end, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    }

    void callbackLines(const line_msg::lineConstPtr& line1_img_msg,
                  const line_msg::lineConstPtr& line2_img_msg,
                  const line_msg::lineConstPtr& line3_img_msg,
                  const line_msg::lineConstPtr& line4_img_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line1_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line2_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line3_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line4_cloud_msg) {
        no_of_frames++;

        line1_start = cv::Point2f(line1_img_msg->a1, line1_img_msg->b1);
        line1_end = cv::Point2f(line1_img_msg->a2, line1_img_msg->b2);
        line2_start = cv::Point2f(line2_img_msg->a1, line2_img_msg->b1);
        line2_end = cv::Point2f(line2_img_msg->a2, line2_img_msg->b2);
        line3_start = cv::Point2f(line3_img_msg->a1, line3_img_msg->b1);
        line3_end = cv::Point2f(line3_img_msg->a2, line3_img_msg->b2);
        line4_start = cv::Point2f(line4_img_msg->a1, line4_img_msg->b1);
        line4_end = cv::Point2f(line4_img_msg->a2, line4_img_msg->b2);

        pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
        pcl::fromROSMsg(*line1_cloud_msg, line_1_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
        pcl::fromROSMsg(*line2_cloud_msg, line_2_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
        pcl::fromROSMsg(*line3_cloud_msg, line_3_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
        pcl::fromROSMsg(*line4_cloud_msg, line_4_pcl);

        std::vector<cv::Point2d> imagePts1 = getProjectedPts(line_1_pcl);
        std::vector<cv::Point2d> imagePts2 = getProjectedPts(line_2_pcl);
        std::vector<cv::Point2d> imagePts3 = getProjectedPts(line_3_pcl);
        std::vector<cv::Point2d> imagePts4 = getProjectedPts(line_4_pcl);

        cv::Vec3f line1 = getEqnOfLine(cv::Vec4f(line1_start.x, line1_start.y, line1_end.x, line1_end.y));
        cv::Vec3f line2 = getEqnOfLine(cv::Vec4f(line2_start.x, line2_start.y, line2_end.x, line2_end.y));
        cv::Vec3f line3 = getEqnOfLine(cv::Vec4f(line3_start.x, line3_start.y, line3_end.x, line3_end.y));
        cv::Vec3f line4 = getEqnOfLine(cv::Vec4f(line4_start.x, line4_start.y, line4_end.x, line4_end.y));

        cv::Point2f point_1 = getIntersection(line1, line2);
        cv::Point2f point_2 = getIntersection(line2, line3);
        cv::Point2f point_3 = getIntersection(line3, line4);
        cv::Point2f point_4 = getIntersection(line4, line1);



        double distance1 = 0;
        for(int i = 0; i < imagePts1.size(); i++){
            distance1 += distanceFromLine(line1, imagePts1[i]);
        }
        distance1 = distance1/imagePts1.size();

        double distance2 = 0;
        for(int i = 0; i < imagePts2.size(); i++){
            distance2 += distanceFromLine(line2, imagePts2[i]);
        }
        distance2 = distance2/imagePts2.size();

        double distance3 = 0;
        for(int i = 0; i < imagePts3.size(); i++){
            distance3 += distanceFromLine(line3, imagePts3[i]);
        }
        distance3 = distance3/imagePts3.size();

        double distance4 = 0;
        for(int i = 0; i < imagePts4.size(); i++){
            distance4 += distanceFromLine(line4, imagePts4[i]);
        }
        distance4 = distance4/imagePts4.size();

        dist_avg += (distance1 + distance2 + distance3 + distance4)/4;
        ROS_WARN_STREAM("Avg Reproj Error: " << dist_avg/no_of_frames);
        if(isWithinImage(point_1) && isWithinImage(point_2)
           && isWithinImage(point_3) && isWithinImage(point_4) || camera_name == "basler") {
            Eigen::Vector4d pose3d = getPose(point_1, point_2, point_3, point_4);
            csv_file << pose3d(0) << "," << pose3d(1) << "," << pose3d(2) << "," << pose3d(3) <<  ","<<
            (distance1 + distance2 + distance3 + distance4)/4 << "\n";
        }
//        ROS_WARN_STREAM("Avg Reproj Error: " << dist_avg);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& line1_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line2_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line3_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line4_cloud_msg,
                  const sensor_msgs::ImageConstPtr& image_msg) {
        try {
            distortedImg = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            cv::undistort(distortedImg, undistortedImg, K, D, cv::noArray());
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
        }
        pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
        pcl::fromROSMsg(*line1_cloud_msg, line_1_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
        pcl::fromROSMsg(*line2_cloud_msg, line_2_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
        pcl::fromROSMsg(*line3_cloud_msg, line_3_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
        pcl::fromROSMsg(*line4_cloud_msg, line_4_pcl);

        std::vector<cv::Point2d> imagePts1 = getProjectedPts(line_1_pcl);
        std::vector<cv::Point2d> imagePts2 = getProjectedPts(line_2_pcl);
        std::vector<cv::Point2d> imagePts3 = getProjectedPts(line_3_pcl);
        std::vector<cv::Point2d> imagePts4 = getProjectedPts(line_4_pcl);

        if(camera_name=="basler") {
            for (int i = 0; i < imagePts1.size(); i++) {
                cv::circle(undistortedImg, imagePts1[i], 20, cv::Scalar(255, 0, 0), -1, 1, 0);
            }
            for (int i = 0; i < imagePts2.size(); i++) {
                cv::circle(undistortedImg, imagePts2[i], 20, cv::Scalar(0, 0, 255), -1, 1, 0);
            }
            for (int i = 0; i < imagePts3.size(); i++) {
                cv::circle(undistortedImg, imagePts3[i], 20, cv::Scalar(255, 255, 0), -1, 1, 0);
            }
            for (int i = 0; i < imagePts4.size(); i++) {
                cv::circle(undistortedImg, imagePts4[i], 20, cv::Scalar(0, 255, 0), -1, 1, 0);
            }
            cv::resize(undistortedImg, undistortedImg, cv::Size(undistortedImg.cols/2, undistortedImg.rows/2));
            cv::imshow(node_name+" edge projection", undistortedImg);
        } else {
            for (int i = 0; i < imagePts1.size(); i++) {
                cv::circle(undistortedImg, imagePts1[i], 5, cv::Scalar(255, 0, 0), -1, 1, 0);
            }
            for (int i = 0; i < imagePts2.size(); i++) {
                cv::circle(undistortedImg, imagePts2[i], 5, cv::Scalar(0, 0, 255), -1, 1, 0);
            }
            for (int i = 0; i < imagePts3.size(); i++) {
                cv::circle(undistortedImg, imagePts3[i], 5, cv::Scalar(255, 255, 0), -1, 1, 0);
            }
            for (int i = 0; i < imagePts4.size(); i++) {
                cv::circle(undistortedImg, imagePts4[i], 5, cv::Scalar(0, 255, 0), -1, 1, 0);
            }
            drawLinesOnImage();
            cv::imshow(node_name+" edge projection", undistortedImg);
        }
        cv::waitKey(30);
        cv::imwrite(base_folder_name+std::to_string(view_no)+".png", undistortedImg);
        cv::waitKey(1);
        view_no++;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_projection_node");
    ros::NodeHandle nh("~");
    projectionLidarLines pLL(nh);
    ros::spin();
    return 0;
}
