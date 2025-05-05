//
// Created by nagy on ١٢‏/٣‏/٢٠٢٢.
//

#include "PointCloudCalibration.h"

/**
 * @PointCloudCalibration constructor to load calibration parameters from files, and prepare
 * calibration matr
 * ices.
 * @param calibrationPath calibration files path.
 */
PointCloudCalibration::PointCloudCalibration(const std::string &calibrationPath, const std::string& posePath) : poseFile(posePath) {

    std::fstream calFile(calibrationPath);

    std::cout<<posePath<<std::endl;

    std::string data;

    std::string pRectStr    = "P2:";
    std::string rRectStr    = "R_rect";
    std::string rtStr       = "Tr_velo_cam";

    while(calFile >> data)
    {
        if(data == pRectStr)
        {
            calFile >> data;
            P_rect_00(0,0) = stod(data); calFile >> data;
            P_rect_00(0,1) = stod(data); calFile >> data;
            P_rect_00(0,2) = stod(data); calFile >> data;
            P_rect_00(0,3) = stod(data); calFile >> data;
            P_rect_00(1,0) = stod(data); calFile >> data;
            P_rect_00(1,1) = stod(data); calFile >> data;
            P_rect_00(1,2) = stod(data); calFile >> data;
            P_rect_00(1,3) = stod(data); calFile >> data;
            P_rect_00(2,0) = stod(data); calFile >> data;
            P_rect_00(2,1) = stod(data); calFile >> data;
            P_rect_00(2,2) = stod(data); calFile >> data;
            P_rect_00(2,3) = stod(data);
        }
        else if(data == rRectStr)
        {
            calFile >> data;
            R_rect_00(0,0) = stod(data); calFile >> data;
            R_rect_00(0,1) = stod(data); calFile >> data;
            R_rect_00(0,2) = stod(data); calFile >> data;
            R_rect_00(0,3) = 0.0;
            R_rect_00(1,0) = stod(data); calFile >> data;
            R_rect_00(1,1) = stod(data); calFile >> data;
            R_rect_00(1,2) = stod(data); calFile >> data;
            R_rect_00(1,3) = 0.0;
            R_rect_00(2,0) = stod(data); calFile >> data;
            R_rect_00(2,1) = stod(data); calFile >> data;
            R_rect_00(2,2) = stod(data);
            R_rect_00(2,3) = 0.0;
            R_rect_00(3,0) = 0.;
            R_rect_00(3,1) = 0.;
            R_rect_00(3,2) = 0.;
            R_rect_00(3,3) = 1.;
        }
        else if(data == rtStr)
        {
            calFile >> data;
            RT(0,0) = stod(data); calFile >> data;
            RT(0,1) = stod(data); calFile >> data;
            RT(0,2) = stod(data); calFile >> data;
            RT(0,3) = stod(data); calFile >> data;
            RT(1,0) = stod(data); calFile >> data;
            RT(1,1) = stod(data); calFile >> data;
            RT(1,2) = stod(data); calFile >> data;
            RT(1,3) = stod(data); calFile >> data;
            RT(2,0) = stod(data); calFile >> data;
            RT(2,1) = stod(data); calFile >> data;
            RT(2,2) = stod(data); calFile >> data;
            RT(2,3) = stod(data);
            RT(3,0) = 0.0;
            RT(3,1) = 0.0;
            RT(3,2) = 0.0;
            RT(3,3) = 1.0;

        }
    }

    calFile.close();
}

void PointCloudCalibration::updateWorldMat(std::string currentTimeStamp)
{
    poseFile.clear();
    poseFile.seekg(0, std::ios::beg);

    std::string data;
    poseFile >> data;

    while (data != currentTimeStamp) {

        for (int i = 0; i < 13; i++) {
            poseFile >> data;

        }
    }
    poseFile >> data;
    WorldMat(0,0) = stod(data); poseFile >> data;
    WorldMat(0,1) = stod(data); poseFile >> data;
    WorldMat(0,2) = stod(data); poseFile >> data;
    WorldMat(0,3) = stod(data); poseFile >> data;
    WorldMat(1,0) = stod(data); poseFile >> data;
    WorldMat(1,1) = stod(data); poseFile >> data;
    WorldMat(1,2) = stod(data); poseFile >> data;
    WorldMat(1,3) = stod(data); poseFile >> data;
    WorldMat(2,0) = stod(data); poseFile >> data;
    WorldMat(2,1) = stod(data); poseFile >> data;
    WorldMat(2,2) = stod(data); poseFile >> data;
    WorldMat(2,3) = stod(data);
    WorldMat(3,0) = 0.0;
    WorldMat(3,1) = 0.0;
    WorldMat(3,2) = 0.0;
    WorldMat(3,3) = 1.0;
    std::cout<<WorldMat<<std::endl;
}


void PointCloudCalibration::cam2LiDAR(Eigen::Vector3d& point)
{
    Eigen::Vector4d pointHomo;
    pointHomo << point(0) , point(1) , point(2) , 1.;

    pointHomo   = R_rect_00.inverse() * pointHomo;

    pointHomo   = RT.inverse()  * pointHomo;

    point << pointHomo(0), pointHomo(1), pointHomo(2);

}


void PointCloudCalibration::LiDAR2cam(Eigen::Vector3d& point)
{
    Eigen::Vector4d pointHomo;
    pointHomo << point(0) , point(1) , point(2) , 1.;

    pointHomo   = RT  * pointHomo;

    pointHomo   = R_rect_00 * pointHomo;

    point << pointHomo(0), pointHomo(1), pointHomo(2);

}

void PointCloudCalibration::boundingBoxProjection(std::vector<Object*>& objects)
{


    for(auto object: objects)
    {

        if(!object->_3dBoxLiDAR) continue;

        std::vector<std::pair<double, double>> projectedCorners;

        // 4x1 3D Lidar PointCloud vector
        Eigen::Vector4d X;
        // 3x1 Projected PointCloud to 2D vector
        Eigen::Vector4d Y;
        Eigen::Vector3d pointCam;

        double minX = -1;
        double maxX = -1;
        double minY = -1;
        double maxY = -1;

        Eigen::Vector3d locWorld, locLidar;
        locWorld << object->_3dBoxLiDAR->estimatedLocation(0), object->_3dBoxLiDAR->estimatedLocation(1), object->_3dBoxLiDAR->estimatedLocation(2);
        world2Lidar(locWorld, locLidar);

        for(int i =0; i < 8; i++)
        {

            X(0, 0) = object->_3dBoxLiDAR->corners(i, 0) + locLidar(0);
            X(1, 0) = object->_3dBoxLiDAR->corners(i,1) + locLidar(1);
            X(2, 0) = object->_3dBoxLiDAR->corners(i, 2);
            X(3, 0) = 1;

            // To camera coordinate
            Y = R_rect_00 * (RT * X);

            pointCam << Y(0), Y(1), Y(2);
            // object rotation in camera coordinate.
            // Transform to image.
            Y << pointCam(0), pointCam(1), pointCam(2), 1;
            pointCam = P_rect_00 * Y;

            std::pair<double, double> pt;

            // pixel point
            pt.first = round(pointCam(0) / pointCam(2));
            pt.second = round(pointCam(1) / pointCam(2));

            projectedCorners.push_back(pt);

            if(i == 0)
            {
                minX = pt.first;
                maxX = pt.first;
                minY = pt.second;
                maxY = pt.second;
            }

            if(minX > pt.first) minX = pt.first;
            if(maxX < pt.first) maxX = pt.first;
            if(minY > pt.second) minY = pt.second;
            if(maxY < pt.second) maxY = pt.second;

        }

        object->_3dBoxLiDAR->boundingBox(0)  = minX;
        object->_3dBoxLiDAR->boundingBox(1)  = minY;
        object->_3dBoxLiDAR->boundingBox(2)  = maxX - minX;
        object->_3dBoxLiDAR->boundingBox(3)  = maxY - minY;

    }

}

void PointCloudCalibration::lidar2World(const Eigen::Vector3d &point, Eigen::Vector3d &worldPoint)
{
    Eigen::Vector4d homo;
    homo << point, 1.0;
    Eigen::Vector4d  homoW = WorldMat * homo;
    worldPoint << homoW(0), homoW(1), homoW(2);
}


void PointCloudCalibration::world2Lidar(const Eigen::Vector3d& worldPoint, Eigen::Vector3d& lidarPoint)
{
    Eigen::Vector4d homo;
    homo << worldPoint, 1.0;

    Eigen::Vector4d  homoW = WorldMat.inverse() * homo;

    lidarPoint << homoW(0), homoW(1), homoW(2);
}

void PointCloudCalibration::objectsProjectionToWorld(std::vector<Object *> &objects)
{
    for(auto object: objects)
    {
        if(object->_3dBoxLiDAR->found)
        {
            lidar2World(object->_3dBoxLiDAR->location, object->_3dBoxLiDAR->location);

        }

    }
}