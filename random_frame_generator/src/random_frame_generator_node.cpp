#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <iostream>

struct PointXYZIr
{
    PCL_ADD_POINT4D
    ; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t ring; ///< laser ring number
    float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
}EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZIr,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (uint16_t, ring, ring))

class frameGenerator {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher cloud_pub_;
    image_transport::Publisher image_pub_;

    std::string image_folder_name;
    std::string pointcloud_folder_name;

    std::string lidar_output_topic_name;
    std::string image_output_topic_name;

    int output_fps;
    bool randomize_frames;
    std::string lidar_frame_id;

public:
    frameGenerator(ros::NodeHandle n): it_(nh_) {
        nh_ = n;
        image_folder_name =
                readParam<std::string>(nh_, "image_folder_name");
        pointcloud_folder_name =
                readParam<std::string>(nh_, "pointcloud_folder_name");
        lidar_output_topic_name =
                readParam<std::string>(nh_, "lidar_output_topic_name");
        image_output_topic_name =
                readParam<std::string>(nh_, "image_output_topic_name");
        output_fps = readParam<int>(nh_, "output_fps");
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(lidar_output_topic_name, 1);
        image_pub_ = it_.advertise(image_output_topic_name, 1);
        lidar_frame_id = readParam<std::string>(nh_, "lidar_frame_id");
        randomize_frames = readParam<bool>(nh_, "randomize_frames");
        ROS_INFO_STREAM("Press [ENTER] to continue");
        std::cin.get();
        readData();
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else{
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void readData() {

        std::vector<cv::String> filenamesImg;
        std::vector<cv::String> filenamesPCD;

        cv::glob(image_folder_name + "/*.png", filenamesImg);
        cv::glob(pointcloud_folder_name + "/*.pcd", filenamesPCD);

        ROS_ASSERT(filenamesPCD.size() == filenamesImg.size());

        int no_of_frames = filenamesImg.size();
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist6(0,
                no_of_frames-1);
        std::vector<int> random_numbers;

        if(randomize_frames) {
            int count = 0;
            while(count < no_of_frames) {
                int random_number = dist6(rng);
                int flag = 0;
                for(int i = 0; i < random_numbers.size(); i++) {
                    if(random_numbers[i] == random_number) {
                        flag = 1;
                        break;
                    }
                }
                if(flag != 1) {
                    random_numbers.push_back(random_number);
                    count ++;
                }
            }
        } else {
            for(int i = 0; i < no_of_frames; i++) {
                random_numbers.push_back(i);
            }
        }

        ROS_WARN_STREAM("RANDOM no size: " << random_numbers.size());
        ROS_WARN_STREAM("Output fps: " << output_fps);
        int i = 0;
        ros::Time present_time, previous_time;
        ROS_INFO_STREAM("No of frames: " << no_of_frames);

        ros::Rate loop_rate(output_fps);


        while (ros::ok() && i < no_of_frames) {
            int query_integer = random_numbers[i];
            cv::Mat image_in = cv::imread(filenamesImg[query_integer]);
            pcl::PointCloud<PointXYZIr>::Ptr
                    cloud (new pcl::PointCloud<PointXYZIr>);
            pcl::io::loadPCDFile<PointXYZIr>(filenamesPCD[query_integer], *cloud);
            sensor_msgs::ImagePtr msg_ros =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                       image_in).toImageMsg();
            ros::Time current_time = ros::Time::now();
            msg_ros->header.stamp = current_time;
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(*cloud, cloud_ros);
            cloud_ros.header.frame_id = lidar_frame_id;
            cloud_ros.header.stamp = current_time;
            image_pub_.publish(msg_ros);
            cloud_pub_.publish(cloud_ros);
            if (i!=0) {
                double time_taken = current_time.toSec() - previous_time.toSec();
                ROS_INFO_STREAM("Publishing Data at " << 1/time_taken << " [Hz]");
            }
            loop_rate.sleep();
            i++;
            previous_time = current_time;
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "frame_generator");
    ros::NodeHandle nh("~");
    frameGenerator fG(nh);
    ros::spin();
    return 0;
}
