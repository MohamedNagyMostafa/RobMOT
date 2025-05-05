//
// Created by nagy on ٣٠‏/٩‏/٢٠٢٣.
//

#include "KalmanFilter.h"
#include "../util.cpp"

Eigen::MatrixXd KalmanFilter::F() const {
    Eigen::Matrix<double, 6, 6> f;
    double&& t      = _dt;
    double&& t_2    = pow(_dt, 2) / 2.;

    f << 1, 0, t, 0, t_2, 0,
         0, 1, 0, t, 0, t_2,
         0, 0, 1, 0, t, 0,
         0, 0, 0, 1, 0, t,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    return std::move(f);
}


Eigen::MatrixXd KalmanFilter::Q() const {
    Eigen::Matrix<double, 6, 6> q;

    double&& delta_k1 = 1/20. * pow(_dt, 5) * _qa  + 1/3. * pow(_dt, 3) * _qv;
    double&& delta_k2 = 1/8.  * pow(_dt, 4) * _qa  + 1/2. * pow(_dt, 2) * _qv;
    double&& delta_k3 = 1/6.  * pow(_dt, 3) * _qa;
    double&& delta_k5 = 1/3.  * pow(_dt, 3) * _qa + _dt * _qv;
    double&& delta_k6 = 1/2.  * pow(_dt, 2) * _qa;
    double&& delta_k8 = _dt * _qa;


    q <<
            delta_k1,        0, delta_k2,        0, delta_k3,        0,
                   0, delta_k1,        0, delta_k2,        0, delta_k3,
            delta_k2,        0, delta_k5,        0, delta_k6,        0,
                   0, delta_k2,        0, delta_k5,        0, delta_k6,
            delta_k3,        0, delta_k6,        0, delta_k8,        0,
                   0, delta_k3,        0, delta_k6,        0, delta_k8;


    return std::move(q);
}

Eigen::MatrixXd KalmanFilter::H() const {
    Eigen::Matrix<double, 2, 6> h;

    h <<    1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;

    return std::move(h);
}

void KalmanFilter::prediction(Eigen::VectorXd& x, Eigen::Matrix<double, 6, 6>& P, const Object* object) {
    std::cout << "x = " << x << std::endl;

    x   =  F() * x; // F_6x6 x_6,1
    P   = F() * P * F().transpose() + Q(); // F_6x6, P_6x6, Q_6x6
}

void KalmanFilter::update(Eigen::VectorXd & x, Eigen::Matrix<double, 6, 6>& P, const Eigen::Vector2d & z, long& trackId, int& streamId, double& altZ, Object* object) {

    Eigen::VectorXd&& residual                  = z -  H() *  x; // z_2x1 H_2x6  x_6x1


    Eigen::Matrix<double, 2, 2>&& co_residual   = H() * P * H().transpose() + _D; // H_2x6 P_6x6 (H^T)_6x2 R_2x2

    Eigen::Matrix<double, 6, 2>&& K             = P * H().transpose() * co_residual.inverse(); // P_6x6 * (H^T)_6x2 co_r_2x2

    x   = x +  K * residual;

    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(2, 2);

    P   = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * H()) * P; // K_6x2 H_2x6

}
