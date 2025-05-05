
#ifndef LIST_H_
#define LIST_H_
#include "../config/config.h"

#include <opencv2/core/mat.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <memory>

using namespace parameters;

/**
 * @Box2D datatype describes 2D bounding box by maximum and minimum of each coordinates system. (x/y).
 */
struct Box2D
{
    Eigen::Vector<double, 4> boundingBox;

    Eigen::VectorXd stateEstimation;

    bool found;

    int observationNumber;
    int lastObservation;
    /**
     * Initialize bounding box coordinates by picking up minimum and maximum point
     * in 2D space.
     */
    Box2D()
    {
        found   = true;
        observationNumber = 0;
        lastObservation   = 0;

        stateEstimation     = Eigen::VectorXd (6);
        stateEstimation     = stateEstimation.setOnes() * -1;

    }

};
/**
 * @Object datatype represents objects detected by sensors.
 * @param id            unique id to identify the object.
 * @param classId       id refers to the type class of the object.
 * @param detectionId   unique detection id for the object.
 * @param trackId       unique id to track the object.
 * @param confidence    confidence score for the detected object
 * @param _2dBox        2D-Bounding box of the object in 2D image frame.
 * @param _3dBox        3D-Bounding box of the object in 3D LiDAR PointCloud.
 */
struct Object{
    int id;
    int classId;
    int detectionId;
    int appearance;
    bool isConfirmed;
    std::string objectStatus;
    float score;
    float alpha;
    float rt;
    float lastObservation;
    long trackId;
    float confidenceScore;
    float validationScore;
    float intervalTime;

    Box2D*  _2dBox;
    Box3D*  _3dBoxLiDAR;
    Box3D*  _3dBoxIMU;


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds;
    std::vector<cv::Point> pointCloudsInPixels;

    Object()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_cp(new pcl::PointCloud<pcl::PointXYZ>);
        // Only if it is a dynamic motion model.
        pointClouds         = pointCloud_cp;
        _3dBoxLiDAR         = nullptr;
        _3dBoxIMU           = nullptr;
        _2dBox              = nullptr;
        trackId             = random();
        appearance          = 0;
        score               = 0;
        validationScore     = 0;
        alpha               = -10;
        rt                  = -10;
        lastObservation     = -1;
        isConfirmed         = false;

        initialCovariance();
    }

    ~Object()
    {
        if(!_3dBoxLiDAR)        delete _3dBoxLiDAR;
        if(!_3dBoxIMU)          delete _3dBoxIMU;
        if(!_2dBox)             delete _2dBox;
    }

    Eigen::Matrix<double, 6, 6>& getCovariance()
    {
        return estimationCovariance;
    }

    void setCovariance(Eigen::Matrix<double, 6, 6>&& covariance)
    {
        estimationCovariance = covariance;
    }
private:
    void initialCovariance()
    {
        double&& xVar       = selectedMotionModel->getPosVar(); // best 0.05
        double&& yVar       = selectedMotionModel->getPosVar();
        double&& velXVar    = selectedMotionModel->getVelVar(); // best 28
        double&& velYVar    = selectedMotionModel->getVelVar();
        double&& accXVar    = selectedMotionModel->getAccVar(); //best 2
        double&& accYVar    = selectedMotionModel->getAccVar();
        double&& jerkXVar    = selectedMotionModel->getJerkVar(); //best 1.
        double&& jerkYVar    = selectedMotionModel->getJerkVar();

        estimationCovariance <<
                             xVar, 0, 0, 0, 0, 0,
                            0,  yVar, 0, 0, 0, 0,
                            0, 0, velXVar, 0, 0, 0,
                            0, 0, 0, velYVar, 0, 0,
                            0, 0, 0, 0, accXVar,0,
                            0, 0, 0, 0, 0, accYVar;

    }

    Eigen::Matrix<double, 6, 6> estimationCovariance;

};

/**
 * Data holder for an individual stream. Corresponding image is stored alongside
 * corresponding PointClouds inside a vector of @LidarPoint.
 */
struct DataFrame{
public:
    cv::Mat* imageFrame;                                    // Image for a given stream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds;        // PointClouds for a given stream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudsFiltered;// Filtered PointClouds.
    std::vector<Object* > objects;                            // Holding detected objects in current frame.

    DataFrame()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_cp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered_cp(new pcl::PointCloud<pcl::PointXYZ>);

        pointClouds = pointCloud_cp;
        pointCloudsFiltered = pointCloudFiltered_cp;

        imageFrame          = new cv::Mat();
    }

    ~DataFrame()
    {
        delete imageFrame;
        for(auto object: objects) delete object;
    }

    /**
     * Clear data for next stream
     */
    void reset()
    {
        pointClouds->clear();
        pointCloudsFiltered->clear();
        objects.clear();

    }
};



#endif

