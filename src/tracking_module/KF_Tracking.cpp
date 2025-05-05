//
// Created by nagy on ٧‏/٦‏/٢٠٢٢.
//

#include "KF_Tracking.h"


/**
 * apply kalman filter in 3D LiDAR space, a 3D bounding box, on the given object, and update
 * state estimation.
 * @param object object to apply KF.
 */
void KF_Tracking::kalmanFilter3D_LiDAR(Object *object, PointCloudCalibration& pointCloudCalibration, int& streamID) {
    auto pre_est = object->_3dBoxLiDAR->stateEstimation;

    // update state.
    if (object->_3dBoxLiDAR->found) {
        object->_3dBoxLiDAR->estimatedLocation  = object->_3dBoxLiDAR->location;
        Eigen::Vector2d z;
        // check first observation
        if (object->_3dBoxLiDAR->observationNumber == 1) {

            object->_3dBoxLiDAR->stateEstimation = object->_3dBoxLiDAR->stateEstimation.setZero();

            object->_3dBoxLiDAR->stateEstimation(0) = object->_3dBoxLiDAR->location(0);
            object->_3dBoxLiDAR->stateEstimation(1) = object->_3dBoxLiDAR->location(1);

        } else {

            z(0) = object->_3dBoxLiDAR->location(0);
            z(1) = object->_3dBoxLiDAR->location(1);

                _kalmanFilter.update(
                         object->_3dBoxLiDAR->stateEstimation,
                         object->getCovariance(),
                         z,
                         object->trackId,
                         streamID,
                         object->_3dBoxLiDAR->location(2),
                         object
                 );
        }
    }
    else
    {

        object->_3dBoxLiDAR->estimatedLocation << object->_3dBoxLiDAR->stateEstimation(0), object->_3dBoxLiDAR->stateEstimation(1), object->_3dBoxLiDAR->location(2);
    }
    // prediction state.
    _kalmanFilter.prediction(object->_3dBoxLiDAR->stateEstimation, object->getCovariance(), object);

}

void KF_Tracking::run(std::vector<KF_Tracking::Type>&& types, std::vector<Object* >& objects, PointCloudCalibration& pointCloudCalibration, int& streamId)
{
    for(auto type: types)
    {
        switch(type)
        {

            case Type::KALMAN_FILTER_3D_LIDAR:
                for(Object* object: objects)
                {
                    if(object->_3dBoxLiDAR) kalmanFilter3D_LiDAR(object, pointCloudCalibration, streamId);
                }
                break;
        }
    }
}
