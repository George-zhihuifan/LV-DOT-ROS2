/*
 * FILE: kalman_filter.cpp
 * -------------------------------------------------------------------------
 * Ported verbatim from the ROS1 LV-DOT onboard_detector/kalmanFilter.cpp.
 */
#include "lvdot_core/kalman_filter.hpp"

namespace onboardDetector {

kalman_filter::kalman_filter()
{
    this->is_initialized = false;
}

void kalman_filter::setup(const Eigen::MatrixXd& states,
                          const Eigen::MatrixXd& A,
                          const Eigen::MatrixXd& B,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& P,
                          const Eigen::MatrixXd& Q,
                          const Eigen::MatrixXd& R)
{
    this->states = states;
    this->A = A;
    this->B = B;
    this->H = H;
    this->P = P;
    this->Q = Q;
    this->R = R;
    this->Q_base = Q;
    this->R_base = R;
    this->is_initialized = true;
}

void kalman_filter::setA(const Eigen::MatrixXd& A)
{
    this->A = A;
}

void kalman_filter::setNoiseScales(double q_scale, double r_scale)
{
    if (this->Q_base.size() == 0 || this->R_base.size() == 0) {
        return;
    }
    if (q_scale < 0.0) q_scale = 0.0;
    if (r_scale < 0.0) r_scale = 0.0;
    this->Q = this->Q_base * q_scale;
    this->R = this->R_base * r_scale;
}

void kalman_filter::estimate(const Eigen::MatrixXd& z, const Eigen::MatrixXd& u)
{
    // predict
    this->states = this->A * this->states + this->B * u;
    this->P = this->A * this->P * this->A.transpose() + this->Q;

    // update
    Eigen::MatrixXd S = this->R + this->H * this->P * this->H.transpose();
    Eigen::MatrixXd K = this->P * this->H.transpose() * S.inverse();

    this->states = this->states + K * (z - this->H * this->states);
    this->P = (Eigen::MatrixXd::Identity(this->P.rows(), this->P.cols()) - K * this->H) * this->P;
}

double kalman_filter::output(int state_index)
{
    if (this->is_initialized) {
        return this->states(state_index, 0);
    } else {
        return 0;
    }
}

}  // namespace onboardDetector
