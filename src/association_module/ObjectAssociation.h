//
// Created by nagy on ٣٠‏/٣‏/٢٠٢٢.
//

#ifndef TRACKING_3D_OBJECTASSOCIATION_H
#define TRACKING_3D_OBJECTASSOCIATION_H


#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "../datatype/data_structure.cpp"
#include "../tracking_module/KF_Tracking.h"

class ObjectAssociation{

public:

    enum AssociationType
    {
        LIDAR_3D_ASSOCIATION,
    };

    static void runAssociation(std::vector<Object*>& currentObservation,
                                     std::vector<Object*> previousObservation,
                                     std::map<int, int>& matches,
                                     std::vector<AssociationType> associationTypes,
                                     std::vector<double> distThreshold,
                                     bool visual);

private:

    /**
     * Associate previous objects with the current detected ones based on distance from estimated state
     * obtained from Kalman Filter for prior objects with current ones.
     * @param currentObservation    current detected objects.
     * @param previousObservation   stored objects in the memory.
     * @param matches               a vector holds index of the association result.
     * @param distThreshold         a maximum distance threshold to associated two objects, if the maximum is
     *                              exceeded, the objects will not be associated.
     * @param visual                to visualize the process.
     */
    static Eigen::MatrixXd associationBy3D_KalmanFilterUsingLiDAR(std::vector<Object *>& currentObservation,
                                                       std::vector<Object *> previousObservation,
                                                       std::map<int, int>& matches,
                                                       double distThreshold,
                                                       bool visual);

    static void matchingByMatrix(std::vector<Object *> &currentObservation,
                                                    std::vector<Object *> previousObservation,
                                                    Eigen::MatrixXd associationMatrix,
                                                    std::map<int, int>& matches,
                                                    double distThreshold,
                                                    bool visual);

};


#endif //TRACKING_3D_OBJECTASSOCIATION_H
