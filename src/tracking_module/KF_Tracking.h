//
// Created by nagy on ٧‏/٦‏/٢٠٢٢.
//

#ifndef TRACKING_3D_KF_TRACKING_H
#define TRACKING_3D_KF_TRACKING_H

#include "../datatype/data_structure.cpp"
#include "KalmanFilter.h"
#include "../calibration_module/PointCloudCalibration.h"

class KF_Tracking {
private:

    KalmanFilter    _kalmanFilter;
    /**
     * apply kalman filter in 3D LiDAR space, a 3D bounding box, on the given object, and update
     * state estimation.
     * @param object object to apply KF.
     */
    void kalmanFilter3D_LiDAR(Object* object, PointCloudCalibration&, int&);


public:

    /**
     * initialize kalman filter gain parameters and state transition and projection matrices.
     * @param alpha parameter for object position
     * @param beta  parameter for object velocity
     * @param gamma parameter for object acceleration
     */
    KF_Tracking(float& dt, float& qa, float& qv, Eigen::Matrix<double, 2, 2>& d): _kalmanFilter(qa, qv, dt, d)
    {

    };

    ~KF_Tracking() = default;

    /**
     * Types of kalman filter to perform, 2D space (image) or 3D space (LiDAR).
     */
    enum Type
    {
        KALMAN_FILTER_3D_LIDAR
    };

    /**
     * perform Kalman Filter operations on either 2D, 3D or both.
     * @param types     type of kalman filter to perform, the chosen types should be from @Type.
     * @param objects   objects to perform the operation.
     */
    void run(std::vector<KF_Tracking::Type>&& types, std::vector<Object *>& objects, PointCloudCalibration&, int& );
};


#endif //TRACKING_3D_KF_TRACKING_H
