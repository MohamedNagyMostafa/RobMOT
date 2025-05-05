//
// Created by nagy on ٣٠‏/٣‏/٢٠٢٢.
//

#include "ObjectAssociation.h"


void ObjectAssociation::runAssociation(std::vector<Object*>& currentObservation,
                                       std::vector<Object*> previousObservation,
                                       std::map<int, int>& matches,
                                       std::vector<AssociationType> associationTypes,
                                       std::vector<double> distThreshold,
                                       bool visual) {

    double distThr;
    Eigen::MatrixXd associationMatrix(currentObservation.size(), previousObservation.size()),
    associationMatrixOut(currentObservation.size(), previousObservation.size());

    associationMatrix = associationMatrix.setZero();
    // Skip if there is no previous objects
    if(previousObservation.size() < 1) return;

    int i = 0;
    // Comparing stage.
    for(AssociationType type: associationTypes) {
        switch (type) {
            case AssociationType::LIDAR_3D_ASSOCIATION:
                distThr = distThreshold.at(i++);
                associationMatrixOut = associationBy3D_KalmanFilterUsingLiDAR(
                        currentObservation,
                        previousObservation,
                        matches,
                        distThr,
                        visual
                );
                associationMatrix +=  associationMatrixOut/distThr;
                break;
        }
    }

    // Matching stage.
    matchingByMatrix(currentObservation,
                     previousObservation,
                     associationMatrix/(associationTypes.size()),
                     matches,
                     1,
                     visual);

}

void ObjectAssociation::matchingByMatrix(std::vector<Object *> &currentObservation,
                                         std::vector<Object *> previousObservation,
                                         Eigen::MatrixXd associationMatrix,
                                         std::map<int, int>& matches,
                                         double distThreshold,
                                         bool visual)
{

    if(visual)  std::cout<<"Begin association matrix  processing " <<std::endl;


    while(true)
    {
        if(associationMatrix.size() < 1) break;

        // Pick up the minimum value from association matrix.
        double minValue = distThreshold;

//        std::cout<<associationMatrix<<std::endl;
        std::ptrdiff_t i, j;
        minValue = associationMatrix.minCoeff(&i,&j);
        auto x  = i;
        auto y  = j;

        if(minValue >= distThreshold) break;
//
        if(visual)  std::cout<<"Associate vehicle id " << previousObservation.at(y)->trackId << " from previous car to "<<
                             currentObservation.at(x)->trackId<<" with distance: " <<std::endl;

        matches.insert(std::pair<int, int>(x, y));

        // TODO: Speed up this procedures by avoid searching within eliminated objects.
        // Add zeros for all corresponding rows/cols of the matched objects.

        associationMatrix.block(0,y,associationMatrix.rows(), 1) = associationMatrix.block(0,y,associationMatrix.rows(), 1).setOnes() *  distThreshold;

        associationMatrix.block(x,0,1, associationMatrix.cols()) =associationMatrix.block(x,0,1, associationMatrix.cols()).setOnes() * distThreshold;


    }
}
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
Eigen::MatrixXd ObjectAssociation::associationBy3D_KalmanFilterUsingLiDAR(std::vector<Object *>& currentObservation,
                                                                          std::vector<Object *> previousObservation,
                                                                          std::map<int, int>& matches,
                                                                          double distThreshold,
                                                                          bool visual) {

    Eigen::MatrixXd associationMatrix(currentObservation.size(), previousObservation.size());
    associationMatrix = associationMatrix.setOnes() * distThreshold;

    bool&& preObjectsRecorded = false;

    for(int currObjectIdx =  0; currObjectIdx < currentObservation.size(); currObjectIdx++)
    {
        for(int prevObjectIdx =  0; prevObjectIdx < previousObservation.size(); prevObjectIdx++)
        {

            auto prevObject = previousObservation.at(prevObjectIdx);
            auto currObject = currentObservation.at(currObjectIdx);

            // Check whether object has 3D bounding box.
            if(!prevObject->_3dBoxLiDAR) continue;
            if(!currObject->_3dBoxLiDAR) continue;

            // Get 3D centroid of previous object from the estimation of KF.

            auto kf3D_Estimation = prevObject->_3dBoxLiDAR->stateEstimation;

            double centroidXLoc_preObject = kf3D_Estimation(0);
            double centroidYLoc_preObject = kf3D_Estimation(1);
            // Get 3D centroid of current object from the 3D observation.

            double distance = sqrt(
                    pow(centroidXLoc_preObject - currObject->_3dBoxLiDAR->location[0], 2) +
                    pow(centroidYLoc_preObject - currObject->_3dBoxLiDAR->location[1], 2)
            );

            if(visual)  cout<<"Object successfully associated, from current frame "<< currentObservation.at(currObjectIdx)->trackId
                            << " to prev frame "<< previousObservation.at(prevObjectIdx)->trackId<<" With distance "<< distance  << " With confidance is " <<  currentObservation.at(currObjectIdx)->confidenceScore<<std::endl;

            associationMatrix(currObjectIdx, prevObjectIdx) = (distance > distThreshold)? distThreshold:distance;

        }
        preObjectsRecorded = true;
    }

    return associationMatrix;
}

