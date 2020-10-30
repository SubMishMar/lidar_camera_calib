#include <random>
#include <chrono>
#include <ctime>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include "normal_msg/normal.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <calibration_error_term.h>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ceres/rotation.h"
#include "ceres/covariance.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "line_msg/line.h"

#include <iostream>

#include <boost/filesystem.hpp>

struct dataFrame {
    pcl::PointCloud<pcl::PointXYZ> lidar_pts;
    Eigen::Vector3d normal;
    Eigen::Vector3d tvec;
    Eigen::Vector3d rvec;
    double noise;
};

typedef message_filters::sync_policies::ApproximateTime
       <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        normal_msg::normal,
        normal_msg::normal,
        normal_msg::normal,
        normal_msg::normal> SyncPolicy1;

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
         normal_msg::normal,
         normal_msg::normal> SyncPolicy2;

class calib {
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line1_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line2_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line3_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line4_sub;
    message_filters::Subscriber<normal_msg::normal> *normal1_sub;
    message_filters::Subscriber<normal_msg::normal> *normal2_sub;
    message_filters::Subscriber<normal_msg::normal> *normal3_sub;
    message_filters::Subscriber<normal_msg::normal> *normal4_sub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *chkrbrdplane_sub;

    message_filters::Subscriber<normal_msg::normal> *normal_sub;
    message_filters::Subscriber<normal_msg::normal> *tvec_sub;

    message_filters::Synchronizer<SyncPolicy1> *sync1;
    message_filters::Synchronizer<SyncPolicy2> *sync2;

    int no_of_line_views, max_no_of_line_views;
    int no_of_plane_views, max_no_of_plane_views;

    Eigen::Matrix3d Rotn;
    Eigen::Vector3d axis_angle;
    Eigen::Vector3d translation;
    Eigen::VectorXd R_t;
    Eigen::VectorXd R_t_init;

    std::vector<dataFrame> plane_data;
    std::vector<dataFrame> line1_data;
    std::vector<dataFrame> line2_data;
    std::vector<dataFrame> line3_data;
    std::vector<dataFrame> line4_data;

    std::string result_str;
    std::string cam_config_file_path;
    std::string target_config_file_path;
    double side_len;

    cv::Mat D, K;
    int image_width, image_height;
    Eigen::Matrix3d K_eig;
    Eigen::Matrix3d K_eig_T;

    bool useLines;
    bool usePlane;
    bool jointSol;

    Eigen::Vector3d r3_old;

    std::string initializations_file;
    std::string results_file;

    double fov_x, fov_y;

    std::ofstream init_file;
    std::ofstream res_file;

    int no_of_diff_initializations;

    bool generate_debug_data;
    std::string debug_data_basefilename;
    std::string lidar_line1_file_name;
    std::string lidar_line2_file_name;
    std::string lidar_line3_file_name;
    std::string lidar_line4_file_name;
    std::string camera_line1_file_name;
    std::string camera_line2_file_name;
    std::string camera_line3_file_name;
    std::string camera_line4_file_name;
    std::string lidar_plane_file_name;
    std::string cam_plane_file_name;

    double plane_selection_threshold;

    std::vector<Eigen::Vector3d> qualified_r3;

    Eigen::Vector4d centroid_old;

    std::string node_name;


public:
    calib(ros::NodeHandle n) {
        nh = n;
        node_name = ros::this_node::getName();
        useLines = readParam<bool>(nh, "useLines");
        usePlane = readParam<bool>(nh, "usePlane");

        if(!useLines && !usePlane) {
            ROS_ERROR("You have to atleast use Lines or use Plane, shouldn't set both to false");
            ros::shutdown();
        }

        no_of_line_views = 0;
        no_of_plane_views = 0;

        if(useLines) {
            max_no_of_line_views = readParam<int>(nh, "max_no_of_line_views");
            line1_data.reserve(max_no_of_line_views);
            line2_data.reserve(max_no_of_line_views);
            line3_data.reserve(max_no_of_line_views);
            line4_data.reserve(max_no_of_line_views);
            line1_sub = new
                    message_filters::Subscriber
                            <sensor_msgs::PointCloud2>(nh, "/line1_out", 1);
            line2_sub = new
                    message_filters::Subscriber
                            <sensor_msgs::PointCloud2>(nh, "/line2_out", 1);
            line3_sub = new
                    message_filters::Subscriber
                            <sensor_msgs::PointCloud2>(nh, "/line3_out", 1);
            line4_sub = new
                    message_filters::Subscriber
                            <sensor_msgs::PointCloud2>(nh, "/line4_out", 1);
            normal1_sub = new
                    message_filters::Subscriber
                            <normal_msg::normal>(nh, "/normal1", 1);
            normal2_sub = new
                    message_filters::Subscriber
                            <normal_msg::normal>(nh, "/normal2", 1);
            normal3_sub = new
                    message_filters::Subscriber
                            <normal_msg::normal>(nh, "/normal3", 1);
            normal4_sub = new
                    message_filters::Subscriber
                            <normal_msg::normal>(nh, "/normal4", 1);
            sync1 = new message_filters::Synchronizer<SyncPolicy1>(SyncPolicy1(10),
                                                                   *line1_sub, *line2_sub, *line3_sub, *line4_sub,
                                                                   *normal1_sub, *normal2_sub, *normal3_sub, *normal4_sub);
            sync1->registerCallback(boost::bind(&calib::callbackLines, this, _1, _2, _3,
                                                _4, _5, _6,
                                                _7, _8));
        } else {
            max_no_of_line_views = 0;
        }

        if(usePlane) {
            max_no_of_plane_views = readParam<int>(nh, "max_no_of_plane_views");
            plane_data.reserve(max_no_of_plane_views);
            plane_selection_threshold = readParam<double>(nh, "plane_selection_threshold");
            chkrbrdplane_sub = new
                    message_filters::Subscriber
                            <sensor_msgs::PointCloud2>(nh, "/points/plane", 1);
            normal_sub = new
                    message_filters::Subscriber
                            <normal_msg::normal>(nh, "/normal_plane", 1);
            tvec_sub = new
                    message_filters::Subscriber
                            <normal_msg::normal>(nh, "/tvec_plane", 1);
            sync2 = new message_filters::Synchronizer<SyncPolicy2>(SyncPolicy2(10),
                                                                   *chkrbrdplane_sub,
                                                                   *normal_sub,
                                                                   *tvec_sub);
            sync2->registerCallback(boost::bind(&calib::callbackPlane, this, _1, _2, _3));
        } else {
            max_no_of_plane_views = 0;
        }

        if (useLines && usePlane) {
            jointSol = readParam<bool>(nh, "jointSol");
        }

        r3_old = Eigen::Vector3d(0, 0, 0);

        Rotn = Eigen::Matrix3d::Zero();

        result_str = readParam<std::string>(nh, "result_str");
        cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");

        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_cam_config.isOpened());
        K = cv::Mat::eye(3, 3, CV_64F);
        D = cv::Mat::zeros(5, 1, CV_64F);
//        fs_cam_config["image_height"] >> image_height;
//        fs_cam_config["image_width"] >> image_width;
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["k3"] >> D.at<double>(4);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);
        cv::cv2eigen(K, K_eig);
        K_eig_T = K_eig.transpose();
        std::cout << K_eig_T << std::endl;
        fov_x = 2*atan2(image_width, 2*K.at<double>(0, 0))*180/CV_PI;
        fov_y = 2*atan2(image_height, 2*K.at<double>(1, 1))*180/CV_PI;

        initializations_file = readParam<std::string>(nh, "initializations_file");
        results_file = readParam<std::string>(nh, "results_file");
        no_of_diff_initializations = readParam<int>(nh, "no_of_diff_initializations");

        generate_debug_data = readParam<bool>(nh, "generate_debug_data");

        if (generate_debug_data) {
            debug_data_basefilename = readParam<std::string>(nh, "debug_data_basefilename");
            if (usePlane) {
                std::string folder_name_lidar = debug_data_basefilename +"/plane/lidar/";
                boost::filesystem::remove_all(folder_name_lidar);
                boost::filesystem::create_directory(folder_name_lidar);
                std::string folder_name_camera = debug_data_basefilename +"/plane/camera/";
                boost::filesystem::remove_all(folder_name_camera);
                boost::filesystem::create_directory(folder_name_camera);
            }
            if (useLines) {
                std::string folder_name_lidar = debug_data_basefilename +"/lines/lidar/";
                boost::filesystem::remove_all(folder_name_lidar);
                boost::filesystem::create_directory(folder_name_lidar);
                std::string folder_name_camera = debug_data_basefilename +"/lines/camera/";
                boost::filesystem::remove_all(folder_name_camera);
                boost::filesystem::create_directory(folder_name_camera);
            }
        }
        centroid_old = Eigen::Vector4d(0, 0, 0, 0);

        target_config_file_path = readParam<std::string>(nh, "target_config_file_path");
        cv::FileStorage fs_target_config(target_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_target_config.isOpened());
        fs_target_config["side_len"] >> side_len;
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("[ "<< node_name << " ]: " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void addGaussianNoise(Eigen::Matrix4d &transformation) {
        std::vector<double> data_rot = {0, 0, 0};
        const double mean_rot = 0.0;
        std::default_random_engine generator_rot;
        generator_rot.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<double> dist(mean_rot, 90);

        // Add Gaussian noise
        for (auto& x : data_rot) {
            x = x + dist(generator_rot);
        }

        double roll = data_rot[0]*M_PI/180;
        double pitch = data_rot[1]*M_PI/180;
        double yaw = data_rot[2]*M_PI/180;

        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        std::vector<double> data_trans = {0, 0, 0};
        const double mean_trans = 0.0;
        std::default_random_engine generator_trans;
        generator_trans.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<double> dist_trans(mean_trans, 0.5);

        // Add Gaussian noise
        for (auto& x : data_trans) {
            x = x + dist_trans(generator_trans);
        }

        Eigen::Vector3d trans;
        trans(0) = data_trans[0];
        trans(1) = data_trans[1];
        trans(2) = data_trans[2];

        Eigen::Matrix4d trans_noise = Eigen::Matrix4d::Identity();
        trans_noise.block(0, 0, 3, 3) = m;
        trans_noise.block(0, 3, 3, 1) = trans;
        transformation = transformation*trans_noise;
    }

    void solvePlaneOptimization() {
        init_file.open(initializations_file);
        res_file.open(results_file);

        time_t tstart, tend;
        tstart = time(0);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);
        ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
        translation = transformation_matrix.block(0, 3, 3, 1);
        R_t = Eigen::VectorXd(6);
        R_t(0) = axis_angle(0);
        R_t(1) = axis_angle(1);
        R_t(2) = axis_angle(2);
        R_t(3) = translation(0);
        R_t(4) = translation(1);
        R_t(5) = translation(2);

        R_t_init = R_t;

        ceres::Problem problem;

        problem.AddParameterBlock(R_t.data(), 6);
        ceres::LossFunction *loss_function = NULL;
        for(int k = 0; k < plane_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
            Eigen::Vector3d r3 = plane_data[k].normal;
            Eigen::Vector3d tvec = plane_data[k].tvec;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                            lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                            (new CalibrationErrorTermPlane(point_3d, r3, tvec, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        tend = time(0);
        ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration:"<< difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                  << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
        res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                 << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << "\n";

        init_file.close();
        res_file.close();
    }

    void solveLineOptimization() {
        ROS_INFO_STREAM("Solving Line Optimization Only");
        init_file.open(initializations_file);
        res_file.open(results_file);

        time_t tstart, tend;
        tstart = time(0);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);
        Rotn(0, 0) = 0; Rotn(0, 1) = -1; Rotn(0, 2) = 0;
        Rotn(1, 0) = 0; Rotn(1, 1) = 0; Rotn(1, 2) = -1;
        Rotn(2, 0) = 1; Rotn(2, 1) = 0; Rotn(2, 2) = 0;
        transformation_matrix(0, 3) = 0;
        transformation_matrix(1, 3) = 0;
        transformation_matrix(2, 3) = 0;
        ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
        translation = transformation_matrix.block(0, 3, 3, 1);
        R_t = Eigen::VectorXd(6);
        R_t(0) = axis_angle(0);
        R_t(1) = axis_angle(1);
        R_t(2) = axis_angle(2);
        R_t(3) = translation(0);
        R_t(4) = translation(1);
        R_t(5) = translation(2);

        R_t_init = R_t;

        ceres::Problem problem;

        problem.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < line1_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
            Eigen::Vector3d normal = line1_data[k].normal;
            ceres::LossFunction *loss_function = NULL;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        for(int k = 0; k < line2_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
            Eigen::Vector3d normal = line2_data[k].normal;
            ceres::LossFunction *loss_function = NULL;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        for(int k = 0; k < line3_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
            Eigen::Vector3d normal = line3_data[k].normal;
            ceres::LossFunction *loss_function = NULL;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        for(int k = 0; k < line4_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
            Eigen::Vector3d normal = line4_data[k].normal;
            ceres::LossFunction *loss_function = NULL;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                    problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        tend = time(0);
        ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration: "<< difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                      << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
        res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                     << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << "\n";

        init_file.close();
        res_file.close();
    }

    void solvePlaneThenLine() {
        init_file.open(initializations_file);
        res_file.open(results_file);
        double avg_time_taken = 0;
        for(int counter = 0; counter < no_of_diff_initializations; counter++) {
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
            Rotn = transformation_matrix.block(0, 0, 3, 3);
            ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
            translation = transformation_matrix.block(0, 3, 3, 1);
            Eigen::Vector3d rpy_init = Rotn.eulerAngles(0, 1, 2)*180/M_PI;
            Eigen::Vector3d tran_init = transformation_matrix.block(0, 3, 3, 1);

            R_t = Eigen::VectorXd(6);
            R_t(0) = axis_angle(0);
            R_t(1) = axis_angle(1);
            R_t(2) = axis_angle(2);
            R_t(3) = translation(0);
            R_t(4) = translation(1);
            R_t(5) = translation(2);

            R_t_init = R_t;

            ceres::LossFunction *loss_function = NULL;

            ceres::Problem problem1;
            problem1.AddParameterBlock(R_t.data(), 6);
            for(int k = 0; k < plane_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
                Eigen::Vector3d r3 = plane_data[k].normal;
                Eigen::Vector3d tvec = plane_data[k].tvec;
                double pi_sqrt = 1/sqrt((double)lidar_pts.size());
                for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                            (new CalibrationErrorTermPlane(point_3d, r3, tvec, pi_sqrt));
                    problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
            }
            ceres::Solver::Options options1;
            options1.minimizer_type = ceres::MinimizerType::TRUST_REGION;
            options1.max_num_iterations = 200;
            options1.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options1.minimizer_progress_to_stdout = false;

            ceres::Solver::Summary summary1;
            ros::Time time_begin1, time_end1;
            time_begin1 = ros::Time::now();
            ceres::Solve(options1, &problem1, &summary1);
            time_end1 = ros::Time::now();
            double time_taken1 = time_end1.toSec()-time_begin1.toSec();
            ROS_WARN_STREAM("Time taken 1: " << time_taken1);
            std::cout << summary1.FullReport() << "\n";
            ceres::Problem problem2;

            problem2.AddParameterBlock(R_t.data(), 6);
            for(int k = 0; k < line1_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
                Eigen::Vector3d normal = line1_data[k].normal;
                Eigen::Vector3d rvec_eig = line1_data[k].rvec;
                Eigen::Vector3d tvec = line1_data[k].tvec;
                cv::Mat rvec_cv;
                cv::eigen2cv(rvec_eig, rvec_cv);
                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                Eigen::Vector3d r3;
                r3(0) = R_cv.at<double>(0, 2);
                r3(1) = R_cv.at<double>(1, 2);
                r3(2) = R_cv.at<double>(2, 2);
                double pi_sqrt = 1/sqrt((double)lidar_pts.size());
                for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function1 = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                    problem2.AddResidualBlock(cost_function1, loss_function, R_t.data());
                }
            }

            for(int k = 0; k < line2_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
                Eigen::Vector3d normal = line2_data[k].normal;
                Eigen::Vector3d rvec_eig = line2_data[k].rvec;
                Eigen::Vector3d tvec = line2_data[k].tvec;
                cv::Mat rvec_cv;
                cv::eigen2cv(rvec_eig, rvec_cv);
                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                Eigen::Vector3d r3;
                r3(0) = R_cv.at<double>(0, 2);
                r3(1) = R_cv.at<double>(1, 2);
                r3(2) = R_cv.at<double>(2, 2);
                double pi_sqrt = 1/sqrt((double)lidar_pts.size());
                for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function1 = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                    problem2.AddResidualBlock(cost_function1, loss_function, R_t.data());
                }
            }

            for(int k = 0; k < line3_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
                Eigen::Vector3d normal = line3_data[k].normal;
                Eigen::Vector3d rvec_eig = line3_data[k].rvec;
                Eigen::Vector3d tvec = line3_data[k].tvec;
                cv::Mat rvec_cv;
                cv::eigen2cv(rvec_eig, rvec_cv);
                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                Eigen::Vector3d r3;
                r3(0) = R_cv.at<double>(0, 2);
                r3(1) = R_cv.at<double>(1, 2);
                r3(2) = R_cv.at<double>(2, 2);
                double pi_sqrt = 1/sqrt((double)lidar_pts.size());
                for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function1 = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                    problem2.AddResidualBlock(cost_function1, loss_function, R_t.data());
                }
            }

            for(int k = 0; k < line4_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
                Eigen::Vector3d normal = line4_data[k].normal;
                Eigen::Vector3d rvec_eig = line4_data[k].rvec;
                Eigen::Vector3d tvec = line4_data[k].tvec;
                cv::Mat rvec_cv;
                cv::eigen2cv(rvec_eig, rvec_cv);
                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                Eigen::Vector3d r3;
                r3(0) = R_cv.at<double>(0, 2);
                r3(1) = R_cv.at<double>(1, 2);
                r3(2) = R_cv.at<double>(2, 2);
                double pi_sqrt = 1/sqrt((double)lidar_pts.size());
                for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function1 = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                            (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                    problem2.AddResidualBlock(cost_function1, loss_function, R_t.data());
                }
            }
            ceres::Solver::Options options2;
            options2.minimizer_type = ceres::MinimizerType::TRUST_REGION;
            options2.max_num_iterations = 200;
            options2.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
            options2.minimizer_progress_to_stdout = false;

            ceres::Solver::Summary summary2;
            ros::Time time_begin2, time_end2;
            time_begin2 = ros::Time::now();
            ceres::Solve(options2, &problem2, &summary2);
            time_end2 = ros::Time::now();
            double time_taken2 = time_end2.toSec()-time_begin2.toSec();
            ROS_WARN_STREAM("Time taken 2: " << time_taken2);
            ROS_WARN_STREAM("Time taken: " << time_taken1+time_taken2);
            avg_time_taken += time_taken1 + time_taken2;
            std::cout << summary2.FullReport() << "\n";

            ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
            Eigen::MatrixXd C_T_L(3, 4);
            C_T_L.block(0, 0, 3, 3) = Rotn;
            C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
            init_file << rpy_init(0) << "," << rpy_init(1) << "," << rpy_init(2) << ","
                      << tran_init(0) << "," << tran_init(1) << "," << tran_init(2) << "\n";
            init_file << Rotn.eulerAngles(0, 1, 2)(0)*180/M_PI << "," << Rotn.eulerAngles(0, 1, 2)(1)*180/M_PI << "," << Rotn.eulerAngles(0, 1, 2)(2)*180/M_PI << ","
                      << R_t[3] << "," << R_t[4] << "," << R_t[5] << "\n";
        }
        avg_time_taken /= no_of_diff_initializations;
        ROS_WARN_STREAM("Avg time taken = " << avg_time_taken);
        init_file.close();
    }

    void solveJointly() {
        init_file.open(initializations_file);
        res_file.open(results_file);


        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);

        ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
        translation = transformation_matrix.block(0, 3, 3, 1);

        R_t = Eigen::VectorXd(6);
        R_t(0) = axis_angle(0);
        R_t(1) = axis_angle(1);
        R_t(2) = axis_angle(2);
        R_t(3) = translation(0);
        R_t(4) = translation(1);
        R_t(5) = translation(2);

        R_t_init = R_t;

        ceres::LossFunction *loss_function = NULL;

        ceres::Problem problem;
        problem.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < plane_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
            Eigen::Vector3d r3 = plane_data[k].normal;
            Eigen::Vector3d tvec = plane_data[k].tvec;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                        (new CalibrationErrorTermPlane(point_3d, r3, tvec, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        problem.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < line1_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
            Eigen::Vector3d normal = line1_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        for(int k = 0; k < line2_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
            Eigen::Vector3d normal = line2_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        for(int k = 0; k < line3_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
            Eigen::Vector3d normal = line3_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        for(int k = 0; k < line4_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
            Eigen::Vector3d normal = line4_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        ceres::Solver::Options options;
        options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        time_t tstart, tend;
        tstart = time(0);
        ceres::Solve(options, &problem, &summary);
        tend = time(0);
        ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration: " << difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                  << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
        res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                 << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << "\n";

        init_file.close();
        res_file.close();
    }

    void generateCSVFileFromLIDAR(std::string filename,
                                  pcl::PointCloud<pcl::PointXYZ> cloud_data_pcl) {
        std::ofstream csv_file;
        csv_file.open(filename);
        for(int i = 0; i < cloud_data_pcl.points.size(); i++) {
            double X = cloud_data_pcl.points[i].x;
            double Y = cloud_data_pcl.points[i].y;
            double Z = cloud_data_pcl.points[i].z;
            csv_file << X << "," << Y << "," << Z << "\n";
        }
        csv_file.close();

    }

    void generateCSVFileFromCamera(std::string filename,
                                   cv::Mat R, cv::Mat t) {
        std::ofstream csv_file;
        csv_file.open(filename);
        csv_file << R.at<double>(0, 0) << "," << R.at<double>(0, 1) << "," << R.at<double>(0, 2) << "," << t.at<double>(0) << "\n"
                 << R.at<double>(1, 0) << "," << R.at<double>(1, 1) << "," << R.at<double>(1, 2) << "," << t.at<double>(1) << "\n"
                 << R.at<double>(2, 0) << "," << R.at<double>(2, 1) << "," << R.at<double>(2, 2) << "," << t.at<double>(2) << "\n";
        csv_file.close();
    }

    void generateCSVFileFromCameraLine(std::string filename, Eigen::Vector3d rvec) {
        std::ofstream csv_file;
        csv_file.open(filename);
        csv_file << rvec(0) << "," << rvec(1) << "," << rvec(2);
        csv_file.close();
    }

    void callbackPlane(const sensor_msgs::PointCloud2ConstPtr &plane_msg,
                       const normal_msg::normalConstPtr &norm_msg,
                       const normal_msg::normalConstPtr &tvec_msg) {
        ROS_INFO_STREAM("At Plane Callback");
        if(usePlane && no_of_plane_views < max_no_of_plane_views) {
            pcl::PointCloud<pcl::PointXYZ> plane_pcl;
            pcl::fromROSMsg(*plane_msg, plane_pcl);
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            rvec.at<double>(0) = norm_msg->a;
            rvec.at<double>(1) = norm_msg->b;
            rvec.at<double>(2) = norm_msg->c;
            cv::Mat C_R_W;
            cv::Rodrigues(rvec, C_R_W);

            Eigen::Vector3d r3(C_R_W.at<double>(0, 2),
                               C_R_W.at<double>(1, 2),
                               C_R_W.at<double>(2, 2));

            Eigen::Vector3d c_t_w(tvec_msg->a, tvec_msg->b, tvec_msg->c);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
            tvec.at<double>(0) = tvec_msg->a;
            tvec.at<double>(1) = tvec_msg->b;
            tvec.at<double>(2) = tvec_msg->c;

            if(r3.dot(r3_old) < plane_selection_threshold) {
                dataFrame plane_datum;
                plane_datum.lidar_pts = plane_pcl;
                plane_datum.normal = r3;
                plane_datum.tvec = c_t_w;
                plane_datum.noise = tvec_msg->w + norm_msg->w;
                plane_data.push_back(plane_datum);
//                qualified_r3.push_back(r3);
                if (generate_debug_data) {
                    lidar_plane_file_name = debug_data_basefilename
                            +"/plane/lidar/lidar_plane_view"
                            +std::to_string(no_of_plane_views)+".csv";
                    cam_plane_file_name = debug_data_basefilename
                            +"/plane/camera/camera_plane_view"
                            +std::to_string(no_of_plane_views)+".csv";
                    generateCSVFileFromLIDAR(lidar_plane_file_name, plane_pcl);
                    generateCSVFileFromCamera(cam_plane_file_name, C_R_W, tvec);
                }
                ROS_INFO_STREAM("[ "<< node_name << " ]: " << " No of plane views: " << ++no_of_plane_views);
                r3_old = r3;
            }
            checkStatus();
        }
    }

    cv::Point2f getIntersection(Eigen::Vector3d line1, Eigen::Vector3d line2) {
        Eigen::Vector3d intersection = line1.cross(line2);
        cv::Point2f pt(intersection(0)/intersection(2),
                       intersection(1)/intersection(2));
        return pt;
    }

    Eigen::VectorXd getPose(cv::Point2f pt1, cv::Point2f pt2,
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
        Eigen::Vector3d tvec_eig;
        tvec_eig(0) = tvec_cv.at<double>(0);
        tvec_eig(1) = tvec_cv.at<double>(1);
        tvec_eig(2) = tvec_cv.at<double>(2);


        Eigen::VectorXd rvec_tvec_eig(6);
        rvec_tvec_eig(0) = rvec_eig(0);
        rvec_tvec_eig(1) = rvec_eig(1);
        rvec_tvec_eig(2) = rvec_eig(2);
        rvec_tvec_eig(3) = tvec_eig(0);
        rvec_tvec_eig(4) = tvec_eig(1);
        rvec_tvec_eig(5) = tvec_eig(2);

        return rvec_tvec_eig;
    }

    void callbackLines(const sensor_msgs::PointCloud2ConstPtr &line1_msg,
                       const sensor_msgs::PointCloud2ConstPtr &line2_msg,
                       const sensor_msgs::PointCloud2ConstPtr &line3_msg,
                       const sensor_msgs::PointCloud2ConstPtr &line4_msg,
                       const normal_msg::normalConstPtr &norm1_msg,
                       const normal_msg::normalConstPtr &norm2_msg,
                       const normal_msg::normalConstPtr &norm3_msg,
                       const normal_msg::normalConstPtr &norm4_msg) {
        ROS_INFO_STREAM("At Line Callback");
        if(useLines && no_of_line_views < max_no_of_line_views) {
            pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
            pcl::fromROSMsg(*line1_msg, line_1_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
            pcl::fromROSMsg(*line2_msg, line_2_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
            pcl::fromROSMsg(*line3_msg, line_3_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
            pcl::fromROSMsg(*line4_msg, line_4_pcl);

            double no_pts_line1 = line_1_pcl.points.size();
            double no_pts_line2 = line_2_pcl.points.size();
            double no_pts_line3 = line_3_pcl.points.size();
            double no_pts_line4 = line_4_pcl.points.size();
            if (no_pts_line1 >= 4 && no_pts_line2 >= 4 &&
                no_pts_line3 >= 4 && no_pts_line4 >= 4) {

                Eigen::Vector3d normal1 = Eigen::Vector3d(norm1_msg->a,
                                                          norm1_msg->b,
                                                          norm1_msg->c);
                Eigen::Vector3d normal2 = Eigen::Vector3d(norm2_msg->a,
                                                          norm2_msg->b,
                                                          norm2_msg->c);
                Eigen::Vector3d normal3 = Eigen::Vector3d(norm3_msg->a,
                                                          norm3_msg->b,
                                                          norm3_msg->c);
                Eigen::Vector3d normal4 = Eigen::Vector3d(norm4_msg->a,
                                                          norm4_msg->b,
                                                          norm4_msg->c);

                Eigen::Vector3d line1_image = K_eig_T.inverse()*normal1;
                Eigen::Vector3d line2_image = K_eig_T.inverse()*normal2;
                Eigen::Vector3d line3_image = K_eig_T.inverse()*normal3;
                Eigen::Vector3d line4_image = K_eig_T.inverse()*normal4;

                cv::Point2f p1 = getIntersection(line1_image, line2_image);
                cv::Point2f p2 = getIntersection(line2_image, line3_image);
                cv::Point2f p3 = getIntersection(line3_image, line4_image);
                cv::Point2f p4 = getIntersection(line4_image, line1_image);

                Eigen::VectorXd rvec_tvec_eig = getPose(p1, p2, p3, p4);
                Eigen::Vector3d rvec(rvec_tvec_eig(0), rvec_tvec_eig(1), rvec_tvec_eig(2));
                Eigen::Vector3d tvec(rvec_tvec_eig(3), rvec_tvec_eig(4), rvec_tvec_eig(5));

                Eigen::Vector4d centroid_l1;
                pcl::compute3DCentroid(line_1_pcl, centroid_l1);
                Eigen::Vector4d centroid_l2;
                pcl::compute3DCentroid(line_2_pcl, centroid_l2);
                Eigen::Vector4d centroid_l3;
                pcl::compute3DCentroid(line_3_pcl, centroid_l3);
                Eigen::Vector4d centroid_l4;
                pcl::compute3DCentroid(line_4_pcl, centroid_l4);
                Eigen::Vector4d centroid_allLines = 0.25*(centroid_l1+centroid_l2+centroid_l3+centroid_l4);
                double distance = (centroid_allLines.transpose() - centroid_old.transpose()).norm();

                if (distance >= 0.0) {
                    dataFrame line1_datum;
                    line1_datum.lidar_pts = line_1_pcl;
                    line1_datum.normal = normal1/normal1.norm();
                    line1_datum.rvec = rvec;
                    line1_datum.tvec = tvec;
                    line1_data.push_back(line1_datum);

                    dataFrame line2_datum;
                    line2_datum.lidar_pts = line_2_pcl;
                    line2_datum.normal = normal2/normal2.norm();
                    line2_datum.rvec = rvec;
                    line2_datum.tvec = tvec;
                    line2_data.push_back(line2_datum);

                    dataFrame line3_datum;
                    line3_datum.lidar_pts = line_3_pcl;
                    line3_datum.normal = normal3/normal3.norm();
                    line3_datum.rvec = rvec;
                    line3_datum.tvec = tvec;
                    line3_data.push_back(line3_datum);

                    dataFrame line4_datum;
                    line4_datum.lidar_pts = line_4_pcl;
                    line4_datum.normal = normal4/normal4.norm();
                    line4_datum.rvec = rvec;
                    line4_datum.tvec = tvec;
                    line4_data.push_back(line4_datum);

                    centroid_old = centroid_allLines;
                    if(generate_debug_data) {
                        lidar_line1_file_name = debug_data_basefilename +
                                                "/lines/lidar/line1_"
                                                +std::to_string(no_of_line_views)+".csv";
                        lidar_line2_file_name = debug_data_basefilename +
                                                "/lines/lidar/line2_"
                                                +std::to_string(no_of_line_views)+".csv";
                        lidar_line3_file_name = debug_data_basefilename +
                                                "/lines/lidar/line3_"
                                                +std::to_string(no_of_line_views)+".csv";
                        lidar_line4_file_name = debug_data_basefilename +
                                                "/lines/lidar/line4_"
                                                +std::to_string(no_of_line_views)+".csv";
                        camera_line1_file_name = debug_data_basefilename
                                              +"/lines/camera/camera_line1_view"
                                              +std::to_string(no_of_line_views)+".csv";
                        camera_line2_file_name = debug_data_basefilename
                                                 +"/lines/camera/camera_line2_view"
                                                 +std::to_string(no_of_line_views)+".csv";
                        camera_line3_file_name = debug_data_basefilename
                                                 +"/lines/camera/camera_line3_view"
                                                 +std::to_string(no_of_line_views)+".csv";
                        camera_line4_file_name = debug_data_basefilename
                                                 +"/lines/camera/camera_line4_view"
                                                 +std::to_string(no_of_line_views)+".csv";

                        generateCSVFileFromLIDAR(lidar_line1_file_name, line_1_pcl);
                        generateCSVFileFromCameraLine(camera_line1_file_name, normal1/normal1.norm());
                        generateCSVFileFromLIDAR(lidar_line2_file_name, line_2_pcl);
                        generateCSVFileFromCameraLine(camera_line2_file_name, normal2/normal2.norm());
                        generateCSVFileFromLIDAR(lidar_line3_file_name, line_3_pcl);
                        generateCSVFileFromCameraLine(camera_line3_file_name, normal3/normal3.norm());
                        generateCSVFileFromLIDAR(lidar_line4_file_name, line_4_pcl);
                        generateCSVFileFromCameraLine(camera_line4_file_name, normal4/normal4.norm());
                    }
                    ROS_INFO_STREAM("[ "<< node_name << " ]: " << " No of line views: " << ++no_of_line_views);
                    checkStatus();
                } else {
                    ROS_WARN_STREAM("[ "<< node_name << " ]: " << " The frames are too close!");
                }
            } else {
                ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Insufficient points in line");
            }
        }
    }

    void checkStatus() {
        bool lineOnlyCond = !usePlane && useLines && no_of_line_views >= max_no_of_line_views;
        bool planeOnlyCond = usePlane && !useLines && no_of_plane_views >= max_no_of_plane_views;
        bool bothLineAndPlane = usePlane && useLines &&
                                no_of_plane_views >= max_no_of_plane_views &&
                                no_of_line_views >= max_no_of_line_views;

        if(bothLineAndPlane) {
            if(jointSol) {
                ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Solving Jointly");
                solveJointly();
                logOutput();
            }
            else {
                ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Solving Serially");
                solvePlaneThenLine();
                logOutput();
            }
            logOutput();
        } else if(lineOnlyCond) {
            ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Solving Line Optimization Only");
            solveLineOptimization();
            logOutput();
        } else if(planeOnlyCond) {
            ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Solving Plane Optimization Only");
            solvePlaneOptimization();
            logOutput();
        } else {
            if(useLines)
                ROS_INFO_STREAM("[ "<< node_name << " ]: " << " No of line views: " << no_of_line_views);
            if(usePlane)
                ROS_INFO_STREAM("[ "<< node_name << " ]: " << " No of plane views: " << no_of_plane_views);
        }
    }

    void logOutput() {
        /// Printing and Storing C_T_L in a file
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        std::cout << "RPY = " << Rotn.eulerAngles(0, 1, 2)*180/M_PI << std::endl;
        std::cout << "t = " << C_T_L.block(0, 3, 3, 1) << std::endl;
        ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Writing the result");
        std::ofstream results;
        results.open(result_str);
        results << C_T_L;
        results.close();
        ROS_WARN_STREAM("[ "<< node_name << " ]: " << " Wrote result to: " << result_str);
        ros::shutdown();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_node");
    ros::NodeHandle nh("~");
    calib cL(nh);
    ros::spin();
    return 0;
}

