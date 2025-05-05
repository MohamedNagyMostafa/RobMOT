//
// Created by nagy on ٢٩‏/٣‏/٢٠٢٢.
//

#include "MemoryManagement.h"
#include "../util.cpp"
#include "../config/config.h"
MemoryManagement::MemoryManagement(bool bVisual)
{
    visual = bVisual;
    nextId = 0;
}

void MemoryManagement::updateAssociatedObjects(const std::map<int, int> &matches, std::vector<Object *> &currentObservation, int streamId)
{
    // Case 1: First frame, no saved objects.
    if(shortTermMemory.empty())
    {
        for(auto object : currentObservation)
        {
            object->trackId = nextId;
            nextId++;
            object->validationScore = 0;
            object->intervalTime    = 0;
            object->validationScore = 0;
            object->lastObservation = streamId;
//                std::cout<<"passed with confidance " << object->confidenceScore << std::endl;
            shortTermMemory.push_back(object);
        }
    }
    else
    {
        // Case 2: Association matched objects
        for(auto match: matches)
        {

            // Current observation
            Object* currentObject   = currentObservation.at(match.first);

            // Previous Observation
            Object* previousObject  = shortTermMemory.at(match.second);

            // Copy previous track id & KF estimation
            currentObject->trackId = (*previousObject).trackId;

            if(currentObject->_3dBoxLiDAR)
            {
                if((*previousObject)._3dBoxLiDAR)
                {
                    currentObject->_3dBoxLiDAR->stateEstimation = (*previousObject)._3dBoxLiDAR->stateEstimation;
                    currentObject->setCovariance(std::move((*previousObject).getCovariance()));

                    currentObject->_3dBoxLiDAR->observationNumber += (*previousObject)._3dBoxLiDAR->observationNumber;
                    // For confidence
                    currentObject->intervalTime     = streamId - previousObject->lastObservation - 1;
                    currentObject->validationScore =  previousObject->validationScore;

                    currentObject->lastObservation  = streamId;
                    currentObject->isConfirmed      = previousObject->isConfirmed;
                }

                currentObject->_3dBoxLiDAR->found = true;
                currentObject->_3dBoxLiDAR->lastObservation = 0;
            }
            else
            {
                throw "matching without having 3D!";
                if((*previousObject)._3dBoxLiDAR)
                {
                    currentObject->_3dBoxLiDAR = previousObject->_3dBoxLiDAR;
                    currentObject->_3dBoxLiDAR->found = false;
                    currentObject->setCovariance(std::move((*previousObject).getCovariance()));


                }
            }

            if(currentObject->_2dBox)
            {

                if((*previousObject)._2dBox) {
                    currentObject->_2dBox->stateEstimation = (*previousObject)._2dBox->stateEstimation;
                    currentObject->_2dBox->observationNumber        += (*previousObject)._2dBox->observationNumber;
                }

                currentObject->_2dBox->found        = true;

                currentObject->_2dBox->lastObservation           = 0;
            }
            else
            {
                if((*previousObject)._2dBox)
                {
                    currentObject->_2dBox = previousObject->_2dBox;
                    currentObject->_2dBox->found = false;
                }
            }


            // Copy setting information
            delete shortTermMemory.at(match.second);
            shortTermMemory.at(match.second) = currentObject;
        }

        // Add objects that doesn't match any
        for(int index = 0; index < currentObservation.size(); index++)
        {
            // To skip associated objects
            bool accept = true;
            if(!matches.empty())
            {
                for(auto match: matches) if (index == match.first){ accept = false; break;}
                if(!accept) continue;
            }
            // Get object information
            Object* object = currentObservation.at(index);

            object->lastObservation = streamId;
            object->validationScore = 0;
            object->intervalTime = 0;
            // Setup tracking id.
            object->trackId = nextId;
            object->_3dBoxLiDAR->found = true;
            nextId++;
            // Add to memory.
            shortTermMemory.push_back(object);

        }
    }

}

void MemoryManagement::updateObjectStatus(int& streamId, float confirmedThr, std::vector<int>& confirmedObjects)
{
    for(int i =0; i < shortTermMemory.size(); i++)
    {
        auto object = shortTermMemory.at(i);
        Helper::trajectoryValidationUpdate(*object);

        if(!object->isConfirmed &&  object->validationScore > confirmedThr)
        {
            object->isConfirmed = true;
            confirmedObjects.push_back(object->trackId);
        }
        if(object->getCovariance()(0, 0) > parameters::selectedMotionModel->getMemCov() || object->getCovariance()(1, 1) > parameters::selectedMotionModel->getMemCov())

    {
            shortTermMemory.erase(shortTermMemory.begin() + i);
            delete object;
            i--;
            continue;
        }
        if(object->_3dBoxIMU)   object->_3dBoxIMU->found    = false;
        if(object->_3dBoxLiDAR)
        {
            object->_3dBoxLiDAR->found  = false;
            object->_3dBoxLiDAR->lastObservation++;
        }
        if(object->_2dBox)
        {
            object->_2dBox->found            = false;
            object->_2dBox->lastObservation++;
        }

    }
}

std::vector<Object *>& MemoryManagement::getSavedObjects()
{
    return shortTermMemory;
}

MemoryManagement::~MemoryManagement()
{
    for(auto object: shortTermMemory) {
        delete object;
        object = nullptr;
    }
    for(auto object: longTermMemory) delete object;

}
