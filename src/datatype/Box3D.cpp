//
// Created by mohamed-nagy on 1/26/25.
//

#include "Box3D.h"

Box3D::Box3D()
{
    stateEstimation = Eigen::VectorXd(6);
    stateEstimation = stateEstimation.setOnes() * -1;

    found = true;
    observationNumber = 0;
    lastObservation   = 0;
}
/**
 * Initialize bounding box coordinates by picking up minimum and maximum point
 * in 3D space.
 * @param minPoint
 * @param maxPoint
 */
Box3D::Box3D(Eigen::Vector3d location, Eigen::Vector4d dim, Eigen::Matrix<double, 8, 3> corners)
{
    this->location  = std::move(location);
    this->corners   = std::move(corners);
    this->dim  = std::move(dim);

    stateEstimation = Eigen::VectorXd(6);
    stateEstimation = stateEstimation.setOnes() * -1;


    found = true;
    observationNumber = 0;
    lastObservation   = 0;
}