//
// Created by subodh on 9/23/19.
//

#ifndef CALIBRATION_CALIBRATION_ERROR_TERM_H
#define CALIBRATION_CALIBRATION_ERROR_TERM_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_cost_function.h>

class CalibrationErrorTermLine {
private:
    const Eigen::Vector3d laser_line_point_;
    const Eigen::Vector3d normal_to_bp_plane_;
    const double sqrt_wt_;

public:
    CalibrationErrorTermLine(const Eigen::Vector3d& laser_line_point,
                         const Eigen::Vector3d& normal_to_bp_plane,
                         const double& sqrt_wt):
                         laser_line_point_(laser_line_point),
                         normal_to_bp_plane_(normal_to_bp_plane),
                         sqrt_wt_(sqrt_wt)
                         {}

    template <typename T>
    bool operator () (const T* const R_t,
                      T* residual) const {
        T l_pt_L[3] = {T(laser_line_point_(0)),
                       T(laser_line_point_(1)),
                       T(laser_line_point_(2))};
        T n_C[3] = {T(normal_to_bp_plane_(0)),
                    T(normal_to_bp_plane_(1)),
                    T(normal_to_bp_plane_(2))};
        T l_pt_C[3];
        ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
        l_pt_C[0] += R_t[3];
        l_pt_C[1] += R_t[4];
        l_pt_C[2] += R_t[5];
        Eigen::Matrix<T, 3, 1> laser_point_C(l_pt_C);
        Eigen::Matrix<T, 3, 1> normal_C(n_C);
        residual[0] = normal_C.normalized().dot(laser_point_C);
        residual[0] = sqrt_wt_ * residual[0];
        return true;
    }
};

class CalibrationErrorTermPlane {
private:
    const Eigen::Vector3d laser_point_;
    const Eigen::Vector3d r3_;
    const Eigen::Vector3d tvec_;
    const double sqrt_wt_;

public:
    CalibrationErrorTermPlane(const Eigen::Vector3d& laser_point,
                              const Eigen::Vector3d& r3,
                              const Eigen::Vector3d& tvec,
                              const double& sqrt_wt):
            laser_point_(laser_point),
            r3_(r3),
            tvec_(tvec),
            sqrt_wt_(sqrt_wt)
    {}

    template  <typename  T>
    bool operator() (const T* const R_t,
                     T* residual) const {
        T l_pt_L[3] = {T(laser_point_(0)), T(laser_point_(1)), T(laser_point_(2))};
        T r_3[3] = {T(r3_(0)), T(r3_(1)), T(r3_(2))};
        T t_vec[3] = {T(tvec_(0)), T(tvec_(1)), T(tvec_(2))};

        T l_pt_C[3];
        ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
        l_pt_C[0] += R_t[3];
        l_pt_C[1] += R_t[4];
        l_pt_C[2] += R_t[5];

        Eigen::Matrix<T, 3, 1> laser_point_C(l_pt_C);
        Eigen::Matrix<T, 3, 1> r3_C(r_3);
        Eigen::Matrix<T, 3, 1> tvec_C(t_vec);
        residual[0] = r3_C.dot(laser_point_C - tvec_C);
        residual[0] = sqrt_wt_ * residual[0];
        return true;
    }
};
#endif //CALIBRATION_CALIBRATION_ERROR_TERM_H
