#ifndef ROBOTICS_HPP_
#define ROBOTICS_HPP_

#include <vector>
#include <utility>

#include <Eigen/Dense>

namespace micros {

/**
 * Determine if a value is near zero.
 * @param val The value to check.
 * @return True if the value is near zero, false otherwise.
 */
bool is_near_zero(const double val);

/** 
 * Normalize a matrix. MatrixXd is used instead of VectorXd for the case of row vectors.
 * @param mat The matrix to normalize.
 * @return The normalized matrix.
 */
Eigen::MatrixXd normalize(Eigen::MatrixXd vec);

/**
 * Convert a angular velocity vector to a skew-symmetric matrix.
 * @param vec The angular velocity vector to convert.
 * @return The skew-symmetric matrix.
 */
Eigen::Matrix3d vec_to_so3(const Eigen::Vector3d& vec);

/**
 * Convert a skew-symmetric matrix to a angular velocity vector.
 * @param so3 The skew-symmetric matrix to convert.
 * @return The angular velocity vector.
 */
Eigen::Vector3d so3_to_vec(const Eigen::Matrix3d& so3);

/**
 * Convert a vector of exponential coordinates to a axis-angle vector.
 * @param vec The vector of exponential coordinates to convert.
 * @return The axis-angle vector.
 */
Eigen::Vector4d axis_angle_3(const Eigen::Vector3d& vec);

/**
 * Compute the matrix exponential of a matrix in so(3).
 * @param mat A 3x3 skew-symmetric matrix.
 * @return The matrix exponential of the input matrix.
 */
Eigen::Matrix3d exp_3(const Eigen::Matrix3d& mat);

/**
 * Compute the matrix logarithm of a rotation matrix.
 * @param rot A 3x3 rotation matrix.
 * @return The matrix logarithm of the input matrix.
*/
Eigen::Matrix3d log_3(const Eigen::Matrix3d& rot);

/**
 * Convert a rotation matrix and a position vector to a homogeneous transformation matrix.
 * @param rot The rotation matrix.
 * @param pos The position vector.
 * @return The homogeneous transformation matrix.
 */
Eigen::Matrix4d Rp_to_T(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos);

/**
 * Convert a homogeneous transformation matrix to a rotation matrix and a position vector.
 * @param T The homogeneous transformation matrix.
 * @return The rotation matrix and the position vector.
 */
std::pair<Eigen::Matrix3d, Eigen::Vector3d> T_to_Rp(const Eigen::Matrix4d& T);

/**
 * Convert a spatial velocity vector into a matrix in se(3).
 * @param vec The spatial velocity vector to convert.
 * @return The transformation matrix in se(3).
 */
Eigen::Matrix4d vec_to_se3(const Eigen::VectorXd& vec);

/** 
 * Convert a matrix in se(3) to a spatial velocity vector.
 * @param se3 The transformation matrix in se(3) to convert.
 * @return The spatial velocity vector.
 */
Eigen::VectorXd se3_to_vec(const Eigen::MatrixXd& se3);

/**
 * Compute the adjoint representation of a homogeneous transformation matrix.
 * @param T The homogeneous transformation matrix.
 * @return The adjoint representation.
 */
Eigen::MatrixXd adjoint(const Eigen::Matrix4d& T);

/**
 * Compute the matrix exponential of a matrix in se(3).
 * @param mat A 4x4 matrix in se(3).
 * @return The matrix exponential of the input matrix.
 */
Eigen::Matrix4d exp_6(const Eigen::Matrix4d& mat);

/**
 * Compute the matrix logarithm of a homogeneous transformation matrix.
 * @param T A homogeneous transformation matrix.
 * @return The matrix logarithm of the input matrix.
 */
Eigen::Matrix4d log_6(const Eigen::Matrix4d& mat);

/** 
 * Compute the forward kinematics of a robot in the space frame.
 * @param M The home configuration of the end-effector.
 * @param S_list The screw axes in the space frame.
 * @param theta_list The joint angles.
 * @return A homogeneous transformation matrix representing the end-effector position and orientation in the space frame.
*/
Eigen::Matrix4d forward_kinematics_space(const Eigen::Matrix4d& M, const Eigen::MatrixXd& S_list, const Eigen::VectorXd& theta_list);

/**
 * Compute the forward kinematics of a robot in the body frame.
 * @param M The home configuration of the end-effector.
 * @param B_list The screw axes in the body frame.
 * @param theta_list The joint angles.
 * @return A homogeneous transformation matrix representing the end-effector position and orientation in the body frame.
*/
Eigen::Matrix4d forward_kinematics_body(const Eigen::Matrix4d& M, const Eigen::MatrixXd& B_list, const Eigen::VectorXd& theta_list);

/**
 * Compute the Jacobian of a robot in the space frame.
 * @param S_list The screw axes in the space frame.
 * @param theta_list The joint angles.
 * @return The Jacobian of the robot in the space frame.
 */
Eigen::MatrixXd jacobian_space(const Eigen::MatrixXd& S_list, const Eigen::VectorXd& theta_list);

/**
 * Compute the Jacobian of a robot in the body frame.
 * @param B_list The screw axes in the body frame.
 * @param theta_list The joint angles.
 * @return The Jacobian of the robot in the body frame.
 */
Eigen::MatrixXd jacobian_body(const Eigen::MatrixXd& B_list, const Eigen::VectorXd& theta_list);

/**
 * Compute the inverse of a homogeneous transformation matrix.
 * @param T The homogeneous transformation matrix.
 * @return The inverse of the input matrix.
 */
Eigen::Matrix4d inv_T(const Eigen::Matrix4d& T);

/**
 * Compute the inverse of a rotation matrix.
 * @param R The rotation matrix.
 * @return The inverse of the input matrix.
 */
Eigen::Matrix3d inv_R(const Eigen::Matrix3d& R);

/**
 * Covert a parametric description of a screw axis into a normalized screw axis.
 * @param q A point on the screw axis.
 * @param s A unit vector in the direction of the screw axis.
 * @param h The pitch of the screw axis.
 * @return The normalized screw axis.
*/
Eigen::VecotrXd screw_to_axis(Eigen::Vector3d q, Eigen::Vector3d s, double h);

/**
* Convert a 6-vector of exponential coordinates into a screw axis-angle vector.
* @param vec The 6-vector of exponential coordinates to convert.
* @return The screw axis-angle vector and the distance traveled along the screw axis.
*/
Eigen::VectorXd axis_ang_6(const Eigen::VectorXd& vec);

/**
 * Project a matrix onto the special orthogonal group SO(3).
 * @param mat The matrix to project.
 * @return The projected matrix.
*/
Eigen::Matrix3d project_to_SO3(const Eigen::Matrix3d& mat);

/**
 * Project a matrix onto the special Euclidean group SE(3).
 * @param mat The matrix to project.
 * @return The projected matrix.
*/
Eigen::Matrix4d project_to_SE3(const Eigen::Matrix4d& mat);

/**
* Return the Frobenius norm to describe the distance of mat from the SO(3).
* @param mat The matrix to compute the distance.
* @return The distance.
*/
double distance_to_SO3(const Eigen::Matrix3d& mat);

/**
* Return the Frobenius norm to describe the distance of mat from the SE(3).
* @param mat The matrix to compute the distance.
* @return The distance.
*/
double distance_to_SE3(const Eigen::Matrix4d& mat);

/**
* Check if the matrix is close to or on the SO(3).
* @param mat The matrix to check.
* @return True if the matrix is close to or on the SO(3), false otherwise.
*/
bool is_valid_SO3(const Eigen::Matrix3d& mat);

/**
* Check if the matrix is close to or on the SE(3).
* @param mat The matrix to check.
* @return True if the matrix is close to or on the SE(3), false otherwise.
*/
bool is_valid_SE3(const Eigen::Matrix4d& mat);

/**
* Compute the inverse kinematics of a robot in the space frame.
* @param M The home configuration of the end-effector.
* @param S_list The screw axes in the space frame.
* @param T The desired configuration of the end-effector.
* @param theta_list_int The initial joint angles.
* @param e_ori The desired orientation error.
* @param e_pos The desired position error.
* @return True if the inverse kinematics is successful, false otherwise.
* @return The joint angles that achieve the desired configuration.
*/
std::pair<bool, Eigen::VectorXd> inverse_kinematics_space(const Eigen::MatrixXd& M, const Eigen::MatrixXd& S_list, const Eigen::MatrixXd& T, const Eigen::VectorXd& theta_list_int, double e_ori, double e_pos);

/**
* Compute the inverse kinematics of a robot in the body frame.
* @param M The home configuration of the end-effector.
* @param B_list The screw axes in the body frame.
* @param T The desired configuration of the end-effector.
* @param theta_list_int The initial joint angles.
* @param e_ori The desired orientation error.
* @param e_pos The desired position error.
* @return True if the inverse kinematics is successful, false otherwise.
* @return The joint angles that achieve the desired configuration.
*/
std::pair<bool, Eigen::VectorXd> inverse_kinematics_body(const Eigen::MatrixXd& M, const Eigen::MatrixXd& B_list, const Eigen::MatrixXd& T, const Eigen::VectorXd& theta_list_int, double e_ori, double e_pos);

/**
* Calculate the 6x6 adjoint matrix of the given 6-vector.
* @param vec The 6-vector to calculate the adjoint matrix.
* @return The 6x6 adjoint matrix.
*/
Eigen::MatrixXd ad(Eigen::VectorXd vec);

/**
* Compute the mass matrix of an open chain robot from the given configuration.
* @param theta_list The joint angles.
* @param M_list List of link frames i relative to i-1 at the home position.
* @param G_list Spatial inertia matrices Gi of the links.
* @param S_list Screw axes Si of the joints in a space frame.
* @return The numerical mass matrix of an n-joints open chain robot at the given configuration and joint angles.
*/
Eigen::MatrixXd mass_matrix(const Eigen::VectorXd& theta_list, const std::vector<Eigen::MatrixXd>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list);

/**
* Compute the Coriolis and centripetal terms in the inverse dynamics equation.
* @param theta_list The joint angles.
* @param dtheta_list The joint velocities.
* @param M_list List of link frames i relative to i-1 at the home position.
* @param G_list Spatial inertia matrices Gi of the links.
* @param S_list Screw axes Si of the joints in a space frame.
* @return The Coriolis and centripetal terms in the inverse dynamics equation.
*/
Eigen::VectorXd coriolis_force(const Eigen::VectorXd& theta_list, const Eigen::VectorXd& dtheta_list, const std::vector<Eigen::MatrixXd>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list);

/**
* Compute the gravity force an open chain robot requires to overcome gravity at its configuration.
* @param theta_list The joint angles.
* @param g The gravity vector.
* @param M_list List of link frames i relative to i-1 at the home position.
* @param G_list Spatial inertia matrices Gi of the links.
* @param S_list Screw axes Si of the joints in a space frame.
* @return The joint forces or torques required to overcome gravity at the given configuration.
*/
Eigen::VectorXd gravity_force(const Eigen::VectorXd& theta_list, const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list);

/**
* Compute the joint forces or torques required to achieve a desired end-effector force.
* @param theta_list The joint angles.
* @param f_tip Spatial force applied by the end-effector expressed in frame {n+1}.
* @param M_list List of link frames i relative to i-1 at the home position.
* @param G_list Spatial inertia matrices Gi of the links.
* @param S_list Screw axes Si of the joints in a space frame.
* @return The joint forces or torques required to achieve the desired end-effector force or torque.
*/
Eigen::VectorXd end_effector_force(const Eigen::VectorXd& theta_list, const Eigen::VectorXd& f_tip, const std::vector<Eigen::MatrixXd>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list);

/**
 * Compute forward dynamics in the space frame for an open chain robot.
 * @param theta_list The joint angles.
 * @param dtheta_list The joint velocities.
 * @param tau_list The joint torques.
 * @param g The gravity vector.
 * @param f_tip Spatial force applied by the end-effector expressed in frame {n+1}.
 * @param M_list List of link frames i relative to i-1 at the home position.
 * @param G_list Spatial inertia matrices Gi of the links.
 * @param S_list Screw axes Si of the joints in a space frame.
 * @return The n-vector of joint accelerations [JTF].
*/
Eigen::VectorXd forward_dynamics(const Eigen::VectorXd& theta_list, const Eigen::VectorXd& dtheta_list, const Eigen::VectorXd& tau_list, const Eigen::VectorXd& g, const Eigen::VectorXd& f_tip,
                                 const std::vector<Eigen::MatrixXd>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list);

/**
 * Compute inverse dynamics in the space frame for an open chain robot.
 * @param theta_list The joint angles.
 * @param dtheta_list The joint velocities.
 * @param ddtheta_list The joint accelerations.
 * @param g The gravity vector.
 * @param f_tip Spatial force applied by the end-effector expressed in frame {n+1}.
 * @param M_list List of link frames i relative to i-1 at the home position.
 * @param G_list Spatial inertia matrices Gi of the links.
 * @param S_list Screw axes Si of the joints in a space frame.
 * @return The n-vector of required joint forces or torques.
*/
Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd& theta_list, const Eigen::VectorXd& dtheta_list, const Eigen::VectorXd& ddtheta_list, const Eigen::Vector3d& g, const Eigen::VectorXd& f_tip,
                                 const std::vector<Eigen::Matrix4d>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list);

/**
* Update the joint angles and velocities using the Euler method.
* @param theta_list The joint angles.
* @param dtheta_list The joint velocities.
* @param ddtheta_list The joint accelerations.
* @param dt The time step.
*/
void euler_step(Eigen::VectorXd& theta_list, Eigen::VectorXd& dtheta_list, const Eigen::VectorXd& ddtheta_list, const double dt);

/**
* Compute the joint control torques at a particular time instant.
* @param theta_list The joint angles.
* @param dtheta_list The joint velocities.
* @param e_int The integral of the error.
* @param g The gravity vector.
* @param M_list List of link frames i relative to i-1 at the home position.
* @param G_list Spatial inertia matrices Gi of the links.
* @param S_list Screw axes Si of the joints in a space frame.
* @param theta_list_ref The reference joint angles.
* @param dtheta_list_ref The reference joint velocities.
* @param ddtheta_list_ref The reference joint accelerations.
* @param Kp The proportional gain.
* @param Ki The integral gain.
* @param Kd The derivative gain.
* @return The joint control torques.
*/
Eigen::VectorXd compute_torque(const Eigen::VectorXd& theta_list, const Eigen::VectorXd& dtheta_list, const Eigen::VectorXd& e_int, const Eigen::VectorXd& g,
                               const std::vector<Eigen::MatrixXd>& M_list, const std::vector<Eigen::MatrixXd>& G_list, const Eigen::MatrixXd& S_list,
                               const Eigen::VectorXd& theta_list_ref, const Eigen::VectorXd& dtheta_list_ref, const Eigen::VectorXd& ddtheta_list_ref, const double Kp, const double Ki, const double Kd);

}

#endif // ROBOTICS_HPP_