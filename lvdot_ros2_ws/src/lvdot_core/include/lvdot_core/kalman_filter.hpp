/*
 * FILE: kalman_filter.hpp
 * -------------------------------------------------------------------------
 * Ported verbatim from the ROS1 LV-DOT onboard_detector/kalmanFilter.h.
 */
#ifndef LVDOT_CORE_KALMAN_FILTER_HPP
#define LVDOT_CORE_KALMAN_FILTER_HPP

#include <Eigen/Dense>

namespace onboardDetector {

class kalman_filter
{
private:
    bool is_initialized;
    Eigen::MatrixXd states;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd H;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

public:
    kalman_filter();

    void setup(const Eigen::MatrixXd& states,
               const Eigen::MatrixXd& A,
               const Eigen::MatrixXd& B,
               const Eigen::MatrixXd& H,
               const Eigen::MatrixXd& P,
               const Eigen::MatrixXd& Q,
               const Eigen::MatrixXd& R);

    void setA(const Eigen::MatrixXd& A);
    void estimate(const Eigen::MatrixXd& z, const Eigen::MatrixXd& u);
    double output(int state_index);
};

}  // namespace onboardDetector

#endif  // LVDOT_CORE_KALMAN_FILTER_HPP
