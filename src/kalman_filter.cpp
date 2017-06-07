#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
    TODO:
      * predict the state
    */
    x_ = F_ * x_;
    MatrixXd f_transpose = F_.transpose();
    P_ = F_ * P_ * f_transpose + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(2, 2);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    const double &px = x_(0);
    const double &py = x_(1);
    const double &vx = x_(2);
    const double &vy = x_(3);

    double rho = sqrt(px * px + py * py);
    double theta = atan2(py, px);
    double rho_dot = (px * vx + py * vy) / rho;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    VectorXd z_pred = VectorXd(3);
    z_pred << rho, theta, rho_dot;

    VectorXd y = z - z_pred;
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(2, 2);
    P_ = (I - K * H_) * P_;

    VectorXd u = VectorXd(2);
    u << 0, 0;
    x_ = F_ * x_ + u;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}
