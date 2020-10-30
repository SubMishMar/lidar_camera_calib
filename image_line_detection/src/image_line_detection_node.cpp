#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <string>

#include <eigen3/Eigen/Eigen>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "normal_msg/normal.h"
#include "line_msg/line.h"

//#define FX 6.4372590342756985e+02
//#define FY 6.4372590342756985e+02
//#define CX 3.9534097290039062e+02
//#define CY 3.0199901199340820e+02
//
double fx, fy, cx, cy;
double k1, k2, p1, p2, k3;
int image_width, image_height;
std::string cam_config_file_path;
std::string target_config_file_path;
std::string camera_name;

double side_len;
struct labelledLine {
    cv::Vec4f line;
    double slope;
    char labelX;
    char labelY;
};
std::vector<labelledLine> lls;

ros::Publisher normal_pub_lt;
ros::Publisher normal_pub_rt;
ros::Publisher normal_pub_rb;
ros::Publisher normal_pub_lb;
ros::Publisher normal_pub_chkrbrd;
ros::Publisher tvec_pub_chkrbrd;
ros::Publisher line_pub_lt;
ros::Publisher line_pub_rt;
ros::Publisher line_pub_rb;
ros::Publisher line_pub_lb;

std_msgs::Header global_header;
int view_no = 0;
int line_no = 0;
int line_length_threshold;
double canny_threshold;
double draw_all_lines = false;
double draw_best_lines = false;

cv::Mat camMat, distMat;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
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
void readCameraParams() {
    cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
    ROS_ASSERT(fs_cam_config.isOpened());
    fs_cam_config["image_height"] >> image_height;
    fs_cam_config["image_width"] >> image_width;
    fs_cam_config["k1"] >> k1;
    fs_cam_config["k2"] >> k2;
    fs_cam_config["p1"] >> p1;
    fs_cam_config["p2"] >> p2;
    fs_cam_config["k3"] >> k3;
    fs_cam_config["fx"] >> fx;
    fs_cam_config["fy"] >> fy;
    fs_cam_config["cx"] >> cx;
    fs_cam_config["cy"] >> cy;

    camMat = cv::Mat::zeros(3, 3, CV_64FC1);
    camMat.at<double>(0, 0) = fx;
    camMat.at<double>(1, 1) = fy;
    camMat.at<double>(0, 2) = cx;
    camMat.at<double>(1, 2) = cy;
    camMat.at<double>(2, 2) = 1.0;

    distMat = cv::Mat::zeros(1, 5, CV_64FC1);
    distMat.at<double>(0) = k1;
    distMat.at<double>(1) = k2;
    distMat.at<double>(2) = p1;
    distMat.at<double>(3) = p2;
    distMat.at<double>(4) = k3;

    ROS_INFO_STREAM("camMat :  \n" << camMat );
    ROS_INFO_STREAM("distMat :  \n" << distMat );
    std::cout << std::endl;
    cv::FileStorage fs_target_config(target_config_file_path, cv::FileStorage::READ);
    ROS_ASSERT(fs_target_config.isOpened());
    fs_target_config["side_len"] >> side_len;
}


cv::Vec3f getEqnOfPlane(cv::Vec3f line) {
    cv::Mat line_vec = cv::Mat::zeros(3, 1, CV_64FC1);
//    std::cout << K << std::endl;
    line_vec.at<double>(0) = line(0);
    line_vec.at<double>(1) = line(1);
    line_vec.at<double>(2) = line(2);
    cv::Mat camMat_transpose;
    cv::transpose(camMat, camMat_transpose);
    cv::Mat normal_c = camMat_transpose*line_vec;
    cv::Vec3f normal_vec(normal_c.at<double>(0),
                         normal_c.at<double>(1),
                         normal_c.at<double>(2));
//    ROS_INFO_STREAM("Normal Equation: " << cv::normalize(normal_vec));
//    return cv::normalize(normal_vec);
    return normal_vec;
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

double getAngle(double slope1, double slope2) {
    double angle = atan2(slope1-slope2, 1+slope1*slope2);
    return angle;
}

double getDistance(cv::Vec4f line1, cv::Vec4f line2) {
    cv::Point2f l1_pta = cv::Point2f(line1[0], line1[1]);
    cv::Point2f l1_ptb = cv::Point2f(line1[2], line1[3]);
    cv::Point2f midPoint1 = 0.5*(l1_pta+l1_ptb);

    cv::Point2f l2_pta = cv::Point2f(line2[0], line2[1]);
    cv::Point2f l2_ptb = cv::Point2f(line2[2], line2[3]);
    cv::Point2f midPoint2 = 0.5*(l2_pta+l2_ptb);
    double distance = cv::norm(midPoint1-midPoint2);
    return distance;
}

cv::Point2f getIntersection(cv::Vec4f line_1, cv::Vec4f line_2) {
    cv::Vec3f line1 = getEqnOfLine(line_1);
    cv::Vec3f line2 = getEqnOfLine(line_2);
    cv::Vec3f intersection = line1.cross(line2);
    cv::Point2f pt(intersection[0]/intersection[2],
                   intersection[1]/intersection[2]);
    return pt;
}

Eigen::Vector2d gettraceRT(cv::Mat jacobian, std::vector<double> reprojection_errors) {
    double sum = std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0);
    double mean = sum / reprojection_errors.size();

    std::vector<double> diff(reprojection_errors.size());
    std::transform(reprojection_errors.begin(), reprojection_errors.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), mean));

    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev_rperr = std::sqrt(sq_sum / (reprojection_errors.size()-1));

    cv::Mat J = jacobian;
    cv::Mat Sigma = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6)).inv();
    cv::Mat std_dev;
    sqrt(Sigma.diag(), std_dev);
    cv::Mat standard_deviation = std_dev*stdev_rperr;
    Eigen::VectorXd varianceRT_eig(6);
    varianceRT_eig(0) = standard_deviation.at<double>(0)*
                        standard_deviation.at<double>(0);
    varianceRT_eig(1) = standard_deviation.at<double>(1)*
                        standard_deviation.at<double>(1);
    varianceRT_eig(2) = standard_deviation.at<double>(2)*
                        standard_deviation.at<double>(2);
    varianceRT_eig(3) = standard_deviation.at<double>(3)*
                        standard_deviation.at<double>(3);
    varianceRT_eig(4) = standard_deviation.at<double>(4)*
                        standard_deviation.at<double>(4);
    varianceRT_eig(5) = standard_deviation.at<double>(5)*
                        standard_deviation.at<double>(5);
    double traceR = varianceRT_eig(0)+varianceRT_eig(1)+varianceRT_eig(2);
    double traceT = varianceRT_eig(3)+varianceRT_eig(4)+varianceRT_eig(5);
    Eigen::Vector2d traceRT(traceR, traceT);
    return traceRT;
}

std::vector<cv::Point2f> getPose(cv::Point2f pt1,
                                 cv::Point2f pt2,
                                 cv::Point2f pt3,
                                 cv::Point2f pt4) {
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

    cv::Mat K = cv::Mat::zeros(3, 3, CV_64FC1);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1.0;

    cv::Mat D = cv::Mat::zeros(4, 1, CV_64FC1);

    cv::Mat rvec(3, 1, CV_64FC1);
    cv::Mat tvec(3, 1, CV_64FC1);

    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);
    std::vector<cv::Point2f> imagePoints_proj;
    cv::Mat jacobian;
    cv::projectPoints(objectPoints, rvec, tvec, K, D, imagePoints_proj, jacobian, 0);
    assert(objectPoints.size() == imagePoints_proj.size());
    assert(imagePoints_proj.size() == imagePoints.size());
    assert(imagePoints.size() == 4);

    Eigen::MatrixXd jacobian_eig(jacobian.rows, jacobian.cols);
    cv::cv2eigen(jacobian, jacobian_eig);

    cv::Mat error_uv = cv::Mat::zeros(cv::Size(2, 4), CV_64F);
    std::vector<double> reproj_errs;
    for(int i = 0; i < objectPoints.size(); i++) {
        float e_u = imagePoints[i].x - imagePoints_proj[i].x;
        float e_v = imagePoints[i].y - imagePoints_proj[i].y;
        error_uv.at<double>(i, 0) = e_u;
        error_uv.at<double>(i, 1) = e_v;
        reproj_errs.push_back(e_u);
        reproj_errs.push_back(e_v);
    }
    Eigen::Vector2d traceRT = gettraceRT(jacobian, reproj_errs);
    cv::Mat mu;
    cv::Mat cov;
    cv::calcCovarMatrix(error_uv, cov, mu, CV_COVAR_NORMAL | CV_COVAR_ROWS);
    cov = cov/(error_uv.rows - 1);
    Eigen::Matrix2d cov_eig = Eigen::Matrix2d::Identity();
    cv::cv2eigen(cov, cov_eig);
    Eigen::MatrixXd jacobian_1 = jacobian_eig.block(0, 0, 2, 6);
    Eigen::MatrixXd jacobian_2 = jacobian_eig.block(0, 2, 2, 6);
    Eigen::MatrixXd jacobian_3 = jacobian_eig.block(0, 4, 2, 6);
    Eigen::MatrixXd jacobian_4 = jacobian_eig.block(0, 6, 2, 6);
    Eigen::MatrixXd info_mat_1 = jacobian_1.transpose() * cov_eig.inverse() * jacobian_1;
    Eigen::MatrixXd info_mat_2 = jacobian_2.transpose() * cov_eig.inverse() * jacobian_2;
    Eigen::MatrixXd info_mat_3 = jacobian_3.transpose() * cov_eig.inverse() * jacobian_3;
    Eigen::MatrixXd info_mat_4 = jacobian_4.transpose() * cov_eig.inverse() * jacobian_4;
    Eigen::MatrixXd info_mat = info_mat_1 + info_mat_2 + info_mat_3 + info_mat_4;
    Eigen::MatrixXd cov_matRT = info_mat.inverse();
    Eigen::MatrixXd cov_matR = cov_matRT.block(0, 0, 3, 3);
    Eigen::MatrixXd cov_matT = cov_matRT.block(3, 3, 3, 3);

    // Careful here, I am publishing the whole rvec instead of publishing C_R_W(:, 3)
    cv::Mat C_R_W_cv;
    Eigen::Matrix3d C_R_W_eig = Eigen::Matrix3d::Identity();
    cv::Rodrigues(rvec, C_R_W_cv);
//    cv::cv2eigen(C_R_W_cv, C_R_W_eig);
//    Eigen::Vector3d euler_angles = C_R_W_eig.eulerAngles(0, 1, 2)*(180/M_PI);
//    std::cout << euler_angles.transpose() << std::endl;
    normal_msg::normal n_plane;
    n_plane.header.stamp = global_header.stamp;
    n_plane.a = rvec.at<double>(0);
    n_plane.b = rvec.at<double>(1);
    n_plane.c = rvec.at<double>(2);
    n_plane.w = traceRT(0);
    normal_pub_chkrbrd.publish(n_plane);

    normal_msg::normal tvec_plane;
    tvec_plane.header.stamp = global_header.stamp;
    tvec_plane.a = tvec.at<double>(0);
    tvec_plane.b = tvec.at<double>(1);
    tvec_plane.c = tvec.at<double>(2);
    tvec_plane.w = traceRT(1);
    tvec_pub_chkrbrd.publish(tvec_plane);

    // This was for some debugging, can be removed
    if(view_no == 151) {
        ROS_WARN_STREAM("View recorded!!");
        std::cout << "C_R_W " << "\n" << C_R_W_cv << std::endl;
        std::cout << "C_t_W " << "\n" << tvec << std::endl << std::endl;
    }

    return imagePoints_proj;
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

void drawAndPublishLineSegments(cv::Mat image_in) {
    bool cam_flag = camera_name == "basler";
    if(lls.size() == 4) {
        std::vector<double> slopes_ordered(4);
        std::vector<cv::Vec4f> lines_ordered(4);
        normal_msg::normal n_lt; line_msg::line l_lt;
        normal_msg::normal n_rt; line_msg::line l_rt;
        normal_msg::normal n_rb; line_msg::line l_rb;
        normal_msg::normal n_lb; line_msg::line l_lb;
        for(size_t i = 0; i < 4; i++) {
            char labelX = lls[i].labelX;
            char labelY = lls[i].labelY;
            if(labelX == 'l' && labelY == 't') {
                slopes_ordered[0] = lls[i].slope;
                lines_ordered[0] = lls[i].line;
            }

            if(labelX == 'r' && labelY == 't') {
                slopes_ordered[1] = lls[i].slope;
                lines_ordered[1] = lls[i].line;
            }

            if(labelX == 'r' && labelY == 'b') {
                slopes_ordered[2] = lls[i].slope;
                lines_ordered[2] = lls[i].line;
            }

            if(labelX == 'l' && labelY == 'b') {
                slopes_ordered[3] = lls[i].slope;
                lines_ordered[3] = lls[i].line;
            }
        }
        double angle01 =
                fabs(getAngle(slopes_ordered[0], slopes_ordered[1]))*180/M_PI;
        double angle12 =
                180.0f-fabs(getAngle(slopes_ordered[1], slopes_ordered[2]))*180/M_PI;
        double angle23 =
                fabs(getAngle(slopes_ordered[2], slopes_ordered[3]))*180/M_PI;
        double angle30 =
                180.0f-fabs(getAngle(slopes_ordered[3], slopes_ordered[0]))*180/M_PI;
//        double angle02 =
//                fabs(getAngle(slopes_ordered[0], slopes_ordered[2]))*180/M_PI;
//        double angle13 =
//                fabs(getAngle(slopes_ordered[1], slopes_ordered[3]))*180/M_PI;

        double dist02 = getDistance(lines_ordered[0], lines_ordered[2]);
        double dist13 = getDistance(lines_ordered[1], lines_ordered[3]);

//        ROS_WARN_STREAM("Angle 1: " << angle01);
//        ROS_WARN_STREAM("Angle 2: " << angle12);
//        ROS_WARN_STREAM("Angle 3: " << angle23);
//        ROS_WARN_STREAM("Angle 4: " << angle30);
//        ROS_WARN_STREAM("Sum of angles: " << angle01 + angle12 + angle23 + angle30);
//        ROS_WARN_STREAM("dist 02: " << dist02);
//        ROS_WARN_STREAM("dist 13: " << dist13);

        if(angle01 > 30 && angle12 > 30 && angle23 > 30 && angle30 > 30 &&
           dist02 > 100 && dist13 > 100) {
            cv::Point2f pt1 = getIntersection(lines_ordered[0], lines_ordered[1]);
            cv::Point2f pt2 = getIntersection(lines_ordered[1], lines_ordered[2]);
            cv::Point2f pt3 = getIntersection(lines_ordered[2], lines_ordered[3]);
            cv::Point2f pt4 = getIntersection(lines_ordered[3], lines_ordered[0]);


            // check if reprojected points are within the image if not, dont publish
	    


            if(isWithinImage(pt1) &&
               isWithinImage(pt2) &&
               isWithinImage(pt3) &&
               isWithinImage(pt4) || cam_flag) {

                ROS_INFO_STREAM("[drawAndPublishLineSegments]: Publishing " << camera_name << " Lines: " << ++line_no);
                std::vector<cv::Point2f> re_projected_pts = getPose(pt1, pt2, pt3, pt4);

                cv::circle(image_in, pt1, 7,
                           cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
                cv::circle(image_in, pt2, 7,
                           cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
                cv::circle(image_in, pt3, 7,
                           cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
                cv::circle(image_in, pt4, 7,
                           cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);

                cv::circle(image_in, re_projected_pts[0], 7,
                           cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
                cv::circle(image_in, re_projected_pts[1], 7,
                           cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
                cv::circle(image_in, re_projected_pts[2], 7,
                           cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_8);
                cv::circle(image_in, re_projected_pts[3], 7,
                           cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_8);

                for(size_t i = 0; i < 4; i++) {
                    cv::Vec4f line_i = lls[i].line;

                    cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
                    cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);
                    cv::Point2f mid_pt = 0.5*(start_pt + end_pt);

                    cv::Scalar line_color = cv::Scalar(0, 0, 0);
                    line_color = cv::Scalar(255, 0, 0);
                    std::string line_txt;

                    char labelX = lls[i].labelX;
                    char labelY = lls[i].labelY;

                    if(labelX == 'l' && labelY == 't') {
                        line_txt = "lt";
                        line_color = cv::Scalar(255, 0, 0);
                        slopes_ordered[0] = lls[i].slope;
                        cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[0]);
                        cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
                        n_lt.header.stamp = global_header.stamp;
                        n_lt.a = normal_eqn(0);
                        n_lt.b = normal_eqn(1);
                        n_lt.c = normal_eqn(2);
                        normal_pub_lt.publish(n_lt);

                        l_lt.header.stamp = global_header.stamp;
                        l_lt.a1 = lls[i].line(0);
                        l_lt.b1 = lls[i].line(1);
                        l_lt.a2 = lls[i].line(2);
                        l_lt.b2 = lls[i].line(3);
                        line_pub_lt.publish(l_lt);
                    } else if(labelX == 'r' && labelY == 't') {
                        line_txt = "rt";
                        line_color = cv::Scalar(0, 0, 255);
                        slopes_ordered[1] = lls[i].slope;
                        cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[1]);
                        cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
                        n_rt.header.stamp = global_header.stamp;
                        n_rt.a = normal_eqn(0);
                        n_rt.b = normal_eqn(1);
                        n_rt.c = normal_eqn(2);
                        normal_pub_rt.publish(n_rt);

                        l_rt.header.stamp = global_header.stamp;
                        l_rt.a1 = lls[i].line(0);
                        l_rt.b1 = lls[i].line(1);
                        l_rt.a2 = lls[i].line(2);
                        l_rt.b2 = lls[i].line(3);
                        line_pub_rt.publish(l_rt);
                    } else if(labelX == 'r' && labelY == 'b') {
                        line_txt = "rb";
                        line_color = cv::Scalar(255, 255, 0);
                        slopes_ordered[2] = lls[i].slope;
                        cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[2]);
                        cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
                        n_rb.header.stamp = global_header.stamp;
                        n_rb.a = normal_eqn(0);
                        n_rb.b = normal_eqn(1);
                        n_rb.c = normal_eqn(2);
                        normal_pub_rb.publish(n_rb);

                        l_rb.header.stamp = global_header.stamp;
                        l_rb.a1 = lls[i].line(0);
                        l_rb.b1 = lls[i].line(1);
                        l_rb.a2 = lls[i].line(2);
                        l_rb.b2 = lls[i].line(3);
                        line_pub_rb.publish(l_rb);
                    } else if(labelX == 'l' && labelY == 'b') {
                        line_txt = "lb";
                        line_color = cv::Scalar(0, 255, 0);
                        slopes_ordered[3] = lls[i].slope;
                        cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[3]);
                        cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
                        n_lb.header.stamp = global_header.stamp;
                        n_lb.a = normal_eqn(0);
                        n_lb.b = normal_eqn(1);
                        n_lb.c = normal_eqn(2);
                        normal_pub_lb.publish(n_lb);

                        l_lb.header.stamp = global_header.stamp;
                        l_lb.a1 = lls[i].line(0);
                        l_lb.b1 = lls[i].line(1);
                        l_lb.a2 = lls[i].line(2);
                        l_lb.b2 = lls[i].line(3);
                        line_pub_lb.publish(l_lb);
                    } else {
                        ROS_WARN_STREAM("[image_line_detection]: No label found");
                    }

                    cv::Scalar start_point_color = cv::Scalar(0, 255, 255);
                    cv::Scalar end_point_color = cv::Scalar(255, 0, 255);
                    cv::line(image_in, start_pt, end_pt, line_color, 2, cv::LINE_8);
                    cv::circle(image_in, start_pt, 3,
                               start_point_color, cv::FILLED, cv::LINE_8);
                    cv::circle(image_in, end_pt, 3,
                               end_point_color, cv::FILLED, cv::LINE_8);
                    cv::putText(image_in,
                                line_txt,
                                mid_pt, cv::FONT_HERSHEY_DUPLEX,
                                1, cv::Scalar(0, 143, 143), 2);
                }
            } else {
                ROS_WARN_STREAM("[image_line_detection]: one of the corners is outside the image, skip publishing");
            }
        } else {
            ROS_WARN_STREAM("[drawAndPublishLineSegments]: Not Publishing Lines");
        }
	if(cam_flag) {
		cv::resize(image_in, image_in, cv::Size(image_width/2, image_height/2));
		cv::imshow(camera_name+" view edges", image_in);
	} else
	    cv::imshow(camera_name+" view edges", image_in);
        cv::waitKey(1);
    } else {
        ROS_WARN_STREAM("[drawAndPublishLineSegments]: Less than 4 valid lines..");
    }
}

double getSlope(cv::Vec4f line) {

    double x_a = line[0];
    double y_a = line[1];
    double x_b = line[2];
    double y_b = line[3];

    double m = (y_b - y_a)/(x_b - x_a);
    return m;
}

double getAngle(cv::Point2f start_pt,
                cv::Point2f end_pt) {
    double x_start = start_pt.x;
    double y_start = start_pt.y;

    double x_end = end_pt.x;
    double y_end = end_pt.y;

    double angle_rad = atan2(y_end - y_start, x_end - x_start);
    double angle_deg = angle_rad*180/M_PI;
    return angle_deg;
}

void chooseBestLines(std::vector<cv::Vec4f> lines, cv::Mat image_in) {
    lls.clear();
    std::vector<cv::Vec4f> angle_filtered_lines;
    std::vector<cv::Vec4f> best_lines;
    std::vector<std::pair<double, int>> lengths;
    size_t i;
    size_t j = 0;

    for(i = 0; i < lines.size(); i++) {
        cv::Vec4f line_i = lines[i];

        cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
        cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);

        double angle_deg = getAngle(start_pt, end_pt);
        double angle_eps = 30;
        if(fabs(angle_deg - 90) < angle_eps)
            continue;
        if(fabs(angle_deg + 90) < angle_eps)
            continue;
        if(fabs(angle_deg) < angle_eps)
            continue;
        if(fabs(angle_deg-180) < angle_eps)
            continue;
        if(fabs(angle_deg+180) < angle_eps)
            continue;

        double length = cv::norm(start_pt-end_pt);
//        std::cout << "Slope: " << angle_deg << std::endl;
        lengths.push_back(std::make_pair(length, j++));
        angle_filtered_lines.push_back(line_i);
    }

//    std::cout << angle_filtered_lines.size() << "\t" << lengths.size() << std::endl;
    if(lengths.size() >= 4) {
        std::sort(lengths.begin(), lengths.end(),
                  std::greater<std::pair<double, int>>());
        // Pick the 4 best lines
        cv::Mat image_best_lines = image_in.clone();
        for (int i = 0; i < 4; i++) {
            labelledLine ll;
            ll.line = angle_filtered_lines[lengths[i].second];
            ll.slope = getSlope(ll.line);
            lls.push_back(ll);
            best_lines.push_back(angle_filtered_lines[lengths[i].second]);
            cv::Vec4f line_i = angle_filtered_lines[lengths[i].second];
            cv::line(image_best_lines,
                     cv::Point2f(line_i[0], line_i[1]),
                     cv::Point2f(line_i[2], line_i[3]),
                     cv::Scalar(0, 255, 0),
                     2, cv::LINE_8);
            cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
            cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);
            double angle_deg = getAngle(start_pt, end_pt);
//            std::cout << "Slope: " << angle_deg << std::endl;
        }
//        cv::imshow("image_best_lines", image_best_lines);
//        cv::waitKey(1);
    }
}

void labelLines(char axis) {
    if(lls.size() == 4) {
        std::vector<cv::Vec4f> sorted_lines;
        std::vector<std::pair<double, int>> dists_of_midpt;
        for(int i = 0; i < 4; i++) {
            cv::Vec4f line_i = lls[i].line;
            cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
            cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);
            cv::Point2f mid_pt = 0.5*(start_pt + end_pt);
            if(axis == 'x')
                dists_of_midpt.push_back(std::make_pair(mid_pt.x, i));
            else if(axis == 'y')
                dists_of_midpt.push_back(std::make_pair(mid_pt.y, i));
            else
                ROS_ASSERT(axis == 'x' || axis == 'y');
        }
        ROS_ASSERT(dists_of_midpt.size() == 4);
        std::sort(dists_of_midpt.begin(),
                  dists_of_midpt.end(),
                  std::greater<std::pair<double, int>>());
        for (int i = 0; i < 4; i++) {
            if(axis == 'x') {
                if(i <= 1)
                    lls[dists_of_midpt[i].second].labelX = 'r';
                else
                    lls[dists_of_midpt[i].second].labelX = 'l';
            } else if (axis == 'y') {
                if(i <= 1)
                    lls[dists_of_midpt[i].second].labelY = 'b';
                else
                    lls[dists_of_midpt[i].second].labelY = 't';
            } else {
                ROS_ASSERT(axis == 'x' || axis == 'y');
            }
        }
    } else {
        ROS_WARN_STREAM("Less than 4 valid lines..");
    }
}

void detectLines(cv::Mat image_in) {
    cv::Mat image_gray;
    cv::cvtColor(image_in, image_gray, CV_RGB2GRAY);
    float distance_threshold = 1.41421356f;
//    float distance_threshold = 1;
    double canny_th1 = canny_threshold;
    double canny_th2 = canny_threshold;
    int canny_aperture_size = 3;
    bool do_merge = true;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =
            cv::ximgproc::createFastLineDetector(line_length_threshold,
                                                     distance_threshold,
                                                     canny_th1, canny_th2,
                                                     canny_aperture_size,
                                                     do_merge);
    std::vector<cv::Vec4f> lines_fld;
    // detects all lines
    fld->detect(image_gray, lines_fld);
    if(draw_all_lines) {
        cv::Mat image_lines = image_in.clone();
        for(int l = 0; l < lines_fld.size(); l++) {
            cv::Vec4f line_l = lines_fld[l];
            cv::Point2f start_pt = cv::Point2f(line_l[0], line_l[1]);
            cv::Point2f end_pt = cv::Point2f(line_l[2], line_l[3]);
            cv::line(image_lines, start_pt, end_pt, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        }
        cv::imshow("all lines", image_lines);
        cv::waitKey(1);
    }
    if(lines_fld.size() >=4) {
        chooseBestLines(lines_fld, image_in);
        labelLines('x');
        labelLines('y');
        drawAndPublishLineSegments(image_in);
    } else {
//        cv::imshow("view", image_in);
//        cv::waitKey(1);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    view_no++;
    try{
        global_header.stamp = msg->header.stamp;
        cv::Mat distortedImg = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat undistortedImg;
        cv::undistort(distortedImg, undistortedImg, camMat, distMat, cv::noArray());
        detectLines(undistortedImg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_line_detector");
    ros::NodeHandle nh("~");
    cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");
    target_config_file_path = readParam<std::string>(nh, "target_config_file_path");
    line_length_threshold = readParam<int>(nh, "line_length_threshold");
    canny_threshold = readParam<double>(nh, "canny_threshold");
    camera_name = readParam<std::string>(nh, "camera_name");
    readCameraParams();
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
    normal_pub_lt = nh.advertise<normal_msg::normal>("/"+camera_name+"/normal1", 1);
    normal_pub_rt = nh.advertise<normal_msg::normal>("/"+camera_name+"/normal2", 1);
    normal_pub_rb = nh.advertise<normal_msg::normal>("/"+camera_name+"/normal3", 1);
    normal_pub_lb = nh.advertise<normal_msg::normal>("/"+camera_name+"/normal4", 1);
    line_pub_lt = nh.advertise<line_msg::line>("/"+camera_name+"/line_image1", 1);
    line_pub_rt = nh.advertise<line_msg::line>("/"+camera_name+"/line_image2", 1);
    line_pub_rb = nh.advertise<line_msg::line>("/"+camera_name+"/line_image3", 1);
    line_pub_lb = nh.advertise<line_msg::line>("/"+camera_name+"/line_image4", 1);
    normal_pub_chkrbrd = nh.advertise<normal_msg::normal>("/"+camera_name+"/normal_plane", 1);
    tvec_pub_chkrbrd = nh.advertise<normal_msg::normal>("/"+camera_name+"/tvec_plane", 1);
    ros::spin();
}
