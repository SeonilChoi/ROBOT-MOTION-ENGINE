#include <cmath>

#include "robot_motion_engine/robotics.hpp"

inline constexpr double NEAR_ZERO = 1e-7;
inline constexpr double MAX_IK_ITER = 100;
inline constexpr double M_PI = 3.14159265358979323846;

bool micros::is_near_zero(const double val) {
    return (std::abs(val) < NEAR_ZERO);
}

Eigen::MatrixXd micros::normalize(Eigen::MatrixXd vec) {
    vec.normalize();
    return vec;
}

Eigen::Matrix3d micros::vec_to_so3(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d so3;
    so3 <<       0, -vec(2),  vec(1),
            vec(2),       0, -vec(0),
           -vec(1),  vec(0),       0;
    return so3;
}

Eigen::Vector3d micros::so3_to_vec(const Eigen::Matrix3d& so3) {
    Eigen::Vector3d vec;
    vec << so3(2, 1), so3(0, 2), so3(1, 0);
    return vec;
}

Eigen::Vector4d micros::axis_angle_3(const Eigen::Vector3d& vec) {
    Eigen::Vector4d axis_angle;
    axis_angle << normalize(vec), vec.norm();
    return axis_angle;
}

Eigen::Matrix3d micros::exp_3(const Eigen::Matrix3d& mat) {
    Eigen::Vector3d omg_theta = so3_to_vec(mat);

    Eigen::Matrix3d exp_mat = Eigen::Matrix3d::Identity();
    if (is_near_zero(omg_theta.norm())) {
        return exp_mat;
    } else {
        double theta = (axis_angle_3(omg_theta))(3);
        Eigen::Matrix3d omg_mat = mat / theta;
        return exp_mat + std::sin(theta) * omg_mat + (1 - std::cos(theta)) * (omg_mat * omg_mat);
    }
}

Eigen::Matrix3d micros::log_3(const Eigen::Matrix3d& rot) {
    double arc_cos_input = (rot.trace() - 1) / 2.0;
    if (arc_cos_input >= 1.0) {
        return Eigen::Matrix3d::Zero();
    } else if (arc_cos_input <= -1.0) {
        Eigen::Vector3d omg;
        if (!is_near_zero(1.0 + rot(2, 2))) {
            omg = (1.0 / std::sqrt(2.0 * (1.0 + rot(2, 2)))) * Eigen::Vector3d(rot(0, 2), rot(1, 2), 1.0 + rot(2, 2));
        } else if (!is_near_zero(1.0 + rot(1, 1))) {
            omg = (1.0 / std::sqrt(2.0 * (1.0 + rot(1, 1)))) * Eigen::Vector3d(rot(0, 1), 1.0 + rot(1, 1), rot(2, 1));
        } else {
            omg = (1.0 / std::sqrt(2.0 * (1.0 + rot(0, 0)))) * Eigen::Vector3d(1.0 + rot(0, 0), rot(1, 0), rot(2, 0))
        }
        return vec_to_so3(omg * M_PI);
    } else {
        double theta = std::acos(arc_cos_input);
        return theta / 2.0 / std::sin(theta) * (rot - rot.transpose());
    }
}

Eigen::Matrix4d micros::Rp_to_T(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos) {
    Eigen::Matrix4d T;
    T.block<3, 3>(0, 0) = rot;
    T.block<3, 1>(0, 3) = pos;
    T.row(3) << 0, 0, 0, 1;
    return T;
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> micros::T_to_Rp(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d rot = T.block<3, 3>(0, 0);
    Eigen::Vector3d pos = T.block<3, 1>(0, 3);
    return std::make_pair(rot, pos);
}

Eigen::Matrix4d micros::vec_to_se3(const Eigen::VectorXd& vec) {
    Eigen::Vector3d exp_angular_vec(vec.head<3>());
    Eigen::Vector3d exp_linear_vec(vec.tail<3>());

    Eigen::Matrix4d se3;
    se3.block<3, 3>(0, 0) = vec_to_so3(exp_angular_vec);
    se3.block<3, 1>(0, 3) = exp_linear_vec;
    se3.row(3) << 0, 0, 0, 0;
}

Eigen::MatrixXd micros::adjoint(const Eigen::Matrix4d& T) {
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> Rp = T_to_Rp(T);
    
    Eigen::MatrixXd adjoint = Eigen::MatrixXd::Zero(6, 6);
    adjoint.block<3, 3>(0, 0) = Rp.first;
    adjoint.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    adjoint.block<3, 3>(3, 0) = Rp.first * vec_to_so3(Rp.second);
    adjoint.block<3, 3>(3, 3) = Rp.first;
    return adjoint;
}

Eigen::Matrix4d micros::exp_6(const Eigen::Matrix4d& mat) {
    Eigen::Matrix3d so3mat = mat.block<3, 3>(0, 0);
    Eigen::Vector3d omg_theta = so3_to_vec(so3mat);

    Eigen::Matrix4d exp_mat;
    if (is_near_zero(omg_theta.norm())) {
        exp_mat.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        exp_mat.block<3, 1>(0, 3) = mat.block<3, 1>(0, 3);
        exp_mat.row(3) << 0, 0, 0, 1;
    } else {
        double theta = (axis_angle_3(omg_theta))(3);
        exp_mat.block<3, 3>(0, 0) = exp_3(so3mat);
        exp_mat.block<3, 1>(0, 3) = Eigen::Matrix3d::Identity() * theta + \
                                    (1 - std::cos(theta)) * mat.block<3, 3>(0, 0) / theta + \
                                    (theta - std::sin(theta)) * ((mat.block<3, 3>(0, 0) / theta) * (mat.block<3, 3>(0, 0) / theta));
        exp_mat.row(3) << 0, 0, 0, 1;
    }
    return exp_mat;
}

Eigen::Matrix4d micros::log_6(const Eigen::Matrix4d& mat) {
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> Rp = T_to_Rp(mat);
    Eigen::Matrix3d so3mat = log_3(Rp.first);
    
    Eigen::Matrix4d log_mat;
    if (is_near_zero(so3mat.norm())) {
        log_mat.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
        log_mat.block<3, 1>(0, 3) = Rp.second;
        log_mat.row(3) << 0, 0, 0, 0;
    } else {
        double theta = std::acos((Rp.first.trace() - 1.0) / 2.0);
        log_mat.block<3, 3>(0, 0) = so3mat;
        log_mat.block<3, 1>(0, 3) = ((Eigen::Matrix3d::Identity() - so3mat / 2.0) + \
                                     ((1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2.0) * (so3mat * so3mat) / theta)) * Rp.second;
        log_mat.row(3) << 0, 0, 0, 0;
    }
    return log_mat;
}

Eigen::Matrix4d micros::forward_kinematics_space(const Eigen::Matrix4d& M, const Eigen::MatrixXd& S_list, const Eigen::VectorXd& theta_list) {
    Eigen::Matrix4d T = M;
    for (int i = theta_list.size() - 1; i > -1; i--) {
        T = exp_6(vec_to_se3(S_list.col(i) * theta_list(i))) * T;
    }
    return T;
}

Eigen::Matrix4d micros::forward_kinematics_body(const Eigen::Matrix4d& M, const Eigen::MatrixXd& B_list, const Eigen::VectorXd& theta_list) {
    Eigen::Matrix4d T = M;
    for (int i = 0; i < theta_list.size(); i++) {
        T = T * exp_6(vec_to_se3(B_list.col(i) * theta_list(i)));
    }
    return T;
}

Eigen::MatrixXd micros::jacobian_space(const Eigen::MatrixXd& S_list, const Eigen::VectorXd& theta_list) {
    int n = theta_list.size();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 1; i < n; i++) {
        T = T * exp_6(vec_to_se3(S_list.col(i - 1) * theta_list(i - 1)));
        J.col(i) = adjoint(T) * S_list.col(i);
    }
    return J;
}

Eigen::MatrixXd micros::jacobian_body(const Eigen::MatrixXd& B_list, const Eigen::VectorXd& theta_list) {
    int n = theta_list.size();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = n - 2; i >= 0; i++) {
        T = T * exp_6(vec_to_se3(-1 * B_list.col(i + 1) * theta_list(i + 1)));
        J.col(i) = adjoint(T) * B_list.col(i);
    }
    return J;
}

Eigen::Matrix4d micros::inv_T(const Eigen::Matrix4d& T) {
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> Rp = T_to_Rp(T);
    
    Eigen::Matrix4d inv_T;
    inv_T.block<3, 3>(0, 0) = Rp.first.transpose();
    inv_T.block<3, 1>(0, 3) = -1 * Rp.first.transpose() * Rp.second;
    inv_T.row(3) << 0, 0, 0, 1;
    return inv_T;
}

Eigen::Matrix3d micros::inv_R(const Eigen::Matrix3d& R) {
    return R.transpose();
}

Eigen::VectorXd micros::screw_to_axis(Eigen::Vector3d q, Eigen::Vector3d s, double h) {
    Eigen::VectorXd axis = Eigen::VectorXd::Zero(6);
    axis.head<3>() = s;
    axis.tail<3>() = q.cross(s) + (h * s);
    return axis;
}

Eigen::VectorXd micros::axis_ang_6(const Eigen::VectorXd& vec) {
    Eigen::VectorXd axis_distance = Eigen::VectorXd::Zero(7);
    double theta = vec.head<3>().norm();
    if (is_near_zero(theta)) {
        theta = vec.tail<3>().norm();
    }
    axis_distance.head<6>() = vec / theta;
    axis_distance(6) = theta;
    return axis_distance;
}

Eigen::Matrix3d micros::project_to_SO3(const Eigen::Matrix3d& mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) {
        R.col(2) = -1 * R.col(2);
    }
    return R;
}

Eigen::Matrix4d micros::project_to_SE3(const Eigen::Matrix4d& mat) {
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> Rp = T_to_Rp(mat);
    Eigen::Matrix4d T = Rp_to_T(project_to_SO3(Rp.first), Rp.second);
    return T;
}

double micros::distance_to_SO3(const Eigen::Matrix3d& mat) {
    if (mat.determinant() > 0) {
        return (mat.transpose() * mat - Eigen::Matrix3d::Identity()).norm();
    } else {
        return 1e9;
    }
}

double micros::distance_to_SE3(const Eigen::Matrix4d& mat) {
    Eigen::Matrix3d R = T_to_Rp(mat).first;
    if (R.determinant() > 0) {
        Eigen::Matrix4d M;
        M.block<3, 3>(0, 0) = R.transpose() * R;
        M.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
        M.row(3) = mat.row(3);
        return (M - Eigen::Matrix4d::Identity()).norm();
    } else {
        return 1e9;
    }
}

bool micros::is_valid_SO3(const Eigen::Matrix3d& mat) {
    return std::abs(distance_to_SO3(mat)) < 1e-3;
}

bool micros::is_valid_SE3(const Eigen::Matrix4d& mat) {
    return std::abs(distance_to_SE3(mat)) < 1e-3;
}

std::pair<bool, Eigen::VectorXd> micros::inverse_kinematics_space(const Eigen::MatrixXd& M, const Eigen::MatrixXd& S_list, const Eigen::MatrixXd& T, const Eigen::VectorXd& theta_list_int, double e_ori, double e_pos) {
    
}

std::pair<bool, Eigen::VectorXd> micros::inverse_kinematics_body(const Eigen::MatrixXd& M, const Eigen::MatrixXd& B_list, const Eigen::MatrixXd& T, const Eigen::VectorXd& theta_list_int, double e_ori, double e_pos) {
    
}