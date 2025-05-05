//
// Created by mohamed-nagy on 1/26/25.
//

#ifndef BOX3D_H
#define BOX3D_H

#include <Eigen/Dense>


class Box3D {
public:
    Eigen::Vector3d location;
    Eigen::Vector3d estimatedLocation;
    Eigen::Matrix<double, 8, 3> corners;
    Eigen::Vector4d boundingBox;
    Eigen::Vector4d dim;

    Eigen::VectorXd stateEstimation; /// (px, py, vx, vy, ax, ay)

    bool found;
    int observationNumber;
    int lastObservation;

    Box3D();
    /**
     * Initialize bounding box coordinates by picking up minimum and maximum point
     * in 3D space.
     * @param minPoint
     * @param maxPoint
     */
    Box3D(Eigen::Vector3d location, Eigen::Vector4d dim, Eigen::Matrix<double, 8, 3> corners);
};


#endif //BOX3D_H
