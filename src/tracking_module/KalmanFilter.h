//
// Created by nagy on ٣٠‏/٩‏/٢٠٢٣.
//

#ifndef TRACKING_3D_KALMANFILTER_H
#define TRACKING_3D_KALMANFILTER_H
#include "../datatype/data_structure.cpp"


class KalmanFilter {
public:
    KalmanFilter(const float& qa, const float& qv, const float& dt, Eigen::Matrix<double,2, 2>& D):_qa(qa), _qv(qv),_dt(dt), _D(D){}; // TODO: dt should be computed based on the frequency.
    ~KalmanFilter() = default;


    void prediction(Eigen::VectorXd& x, Eigen::Matrix<double, 6, 6>& P, const Object* object);
    void update(Eigen::VectorXd & x, Eigen::Matrix<double, 6, 6>& P, const Eigen::Vector2d & z, long& trackId, int& streamId, double& altZ, Object* object);
private:
    const float& _dt;
    const float& _qa, _qv;
    const Eigen::Matrix<double, 2, 2>& _D;

    Eigen::MatrixXd F() const;
    Eigen::MatrixXd Q() const;
    Eigen::MatrixXd H() const;
};


#endif //TRACKING_3D_KALMANFILTER_H
