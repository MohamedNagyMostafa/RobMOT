#ifndef TRACKING_3D_POINTCLOUDCALIBRATION_H
#define TRACKING_3D_POINTCLOUDCALIBRATION_H

#include <iostream>
#include <fstream>

#include "../datatype/data_structure.cpp"

/**
 * @class @PointCloudCalibration responsible for calibration between Camera and LiDAR sensors,
 *  and assign PointClouds for 2D detected objects.
 */
class PointCloudCalibration {

private:
    /// Calibration Matrices ///
    Eigen::Matrix<double, 3, 4> P_rect_00;
    Eigen::Matrix<double, 4, 4> R_rect_00;
    Eigen::Matrix<double, 4, 4> RT;
    Eigen::Matrix<double, 4, 4> WorldMat;
    std::fstream poseFile;

public:
    /**
     * @PointCloudCalibration constructor to load calibration parameters from files, and prepare
     * calibration matrices.
     * @param calibrationPath calibration files path.
     */
    explicit PointCloudCalibration(const std::string& calibrationPath, const std::string& posePath);
    ~PointCloudCalibration()
    {
        poseFile.close();
    };

    /**
     * Launch calibration between LiDAR and Camera given 3D PointClouds, and assign individual PointClouds for
     * detected objects based of certain shrink factor.
     * @param pointClouds PointCloud to be projected into 2D dimension.
     * @param objects Detected objects to assign PointClouds to them.
     * @param shrinkFactor Shrink factor to avoid other objects located in the detected bounding box.
     */
    void run(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, std::vector<Object*>& objects, float shrinkFactor = 0);

    /**
     * Transform into 2D Bounding box
     */

    void boundingBoxProjection(std::vector<Object*>& objects);

    void cam2LiDAR(Eigen::Vector3d& point);

    void LiDAR2cam(Eigen::Vector3d& point);

    void lidar2World(const Eigen::Vector3d& point, Eigen::Vector3d& worldPoint);


    void world2Lidar(const Eigen::Vector3d& worldPoint, Eigen::Vector3d& lidarPoint);

    void objectsProjectionToWorld(std::vector<Object *>& objects);

    void updateWorldMat(std::string currentTimeStamp);
};


#endif //TRACKING_3D_POINTCLOUDCALIBRATION_H
