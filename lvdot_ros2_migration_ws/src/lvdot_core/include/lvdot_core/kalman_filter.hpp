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
    // §3.3 adaptive noise: cache the original Q, R from setup() so that
    // setNoiseScales() can reapply scaling each frame without compounding.
    Eigen::MatrixXd Q_base;
    Eigen::MatrixXd R_base;

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
    // Scale Q and R relative to the base values captured at setup().  Used by
    // tracking_filter when QC-GAF quality vector triggers §3.3 noise adaptation.
    // q_scale, r_scale ≥ 0; 1.0 means no scaling.
    void setNoiseScales(double q_scale, double r_scale);
    void estimate(const Eigen::MatrixXd& z, const Eigen::MatrixXd& u);
    double output(int state_index);
};

}  // namespace onboardDetector

#endif  // LVDOT_CORE_KALMAN_FILTER_HPP
