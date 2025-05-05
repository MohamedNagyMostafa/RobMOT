#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <sys/stat.h>
#include "datatype/data_structure.cpp"
#include "calibration_module/PointCloudCalibration.h"
#include "config/config.h"
#include "memory_module/MemoryManagement.h"
#include <filesystem>

using namespace std;

struct Helper{
private:
    const float RAD2ANG_RATIO = 3.14159/180;

    static double computeIoU(cv::Rect rect1, cv::Rect rect2) {
        double rect1Area        = rect1.width * rect1.height;
        double rect2Area        = rect2.width * rect2.height;
        cv::Rect overlapped     = rect1 & rect2;
        double overlappedArea   = overlapped.width * overlapped.height;

        overlappedArea = (overlappedArea < 0)? 0.: overlappedArea;

        return overlappedArea/(rect1Area + rect2Area - overlappedArea);
    }

    static void matchingByMatrix(Eigen::MatrixXd associationMatrix,
                                 std::map<int, int>& matches,
                                 double distThreshold)
    {
        std::cout<<"association matrix " << associationMatrix<<std::endl;

        while(true)
        {

            double maxValue = distThreshold;
            cv::Point maxLoc;
            for(int i =  0; i < associationMatrix.rows(); i++)
            {

                for(int j =  0; j < associationMatrix.cols(); j++)
                {
                    if(associationMatrix(i, j) > maxValue)
                    {

                        maxLoc.x   = i;
                        maxLoc.y   = j;

                        maxValue    = associationMatrix(i, j);
                    }
                }
            }
            if(maxValue <= distThreshold) break;

            matches.insert(std::pair<int, int>(maxLoc.x, maxLoc.y));


            for(int i =  0; i < associationMatrix.rows(); i++)
            {
                associationMatrix(i, maxLoc.y)    = distThreshold;
            }
            for(int j =  0; j < associationMatrix.cols(); j++)
            {
                associationMatrix(maxLoc.x, j)         = distThreshold;
            }

        }


    }

public:
    static bool insideFOVCam(double x, double y)
    {
        return (x > 0 && x < 120) && (y > -120 && y < 120);
    }
    inline static bool isFileExists (const std::string& name)
    {
        struct stat buffer;
        return (stat (name.c_str(), &buffer) == 0);
    }


    static void addPretrainedDetection3D(std::string stream, DataFrame* dataframe, string streamfile, PointCloudCalibration& pointCloudCalibration, MemoryManagement& memoryManagement)
    {
        ifstream file3D;
        std::string type    = parameters::Settings::dataset;
        std::string database = parameters::selectedDetector->getDataset();
        std::string basePath = Settings::basePath;

        if(parameters::Settings::dataset_name == "waymo")
            file3D.open(basePath + "/advanced_detection/waymo/"+ database + type + "/"+ streamfile+".txt");
        else
            file3D.open(basePath +"/advanced_detection/"+ database+ type + "/" + streamfile+".txt");


        string line3D;
        bool found = false;
        double counter = 0;

        std::vector<Object> objects;
        auto detectionDatabase = parameters::initializeDatabase(Settings::dataset_name);
        while(getline(file3D, line3D))
        {
            istringstream record(line3D);
            string data3D[detectionDatabase->totalFields[detection::SPACE]]; // kitti 18


            for(int i = 0; std::getline(record, data3D[i],' '); i++);

            if(data3D[detectionDatabase->frame[detection::FRAME]] == stream)
            {

                cv::Rect boundingBox;
                Object object;

                if(dynamic_cast<parameters::kitti*>(detectionDatabase.get())) {
                    object._2dBox = new Box2D();

                    object._2dBox->boundingBox(0) = stod(data3D[detectionDatabase->boundBoxCorner_x[detection::BBCorner_X]]); // 2
                    object._2dBox->boundingBox(1)= stod(data3D[detectionDatabase->boundBoxCorner_y[detection::BBCorner_Y]]); // 3
                    object._2dBox->boundingBox(2) = stod(data3D[detectionDatabase->boundBoxCornerWidth[detection::BBCornerWidth]]); // 4
                    object._2dBox->boundingBox(3) = stod(data3D[detectionDatabase->boundBoxCornerHeight[detection::BBCornerHeight]]); // 5



                    object._3dBoxLiDAR = get3DBoundingBox(
                        stod(data3D[detectionDatabase->height[detection::HEIGHT]]),
                        stod(data3D[detectionDatabase->width[detection::WIDTH]]),
                        stod(data3D[detectionDatabase->length[detection::LENGTH]]),
                        stod(data3D[detectionDatabase->boundBoxCenter_X[detection::BBCenter_X]]),
                        stod(data3D[detectionDatabase->boundBoxCenter_Y[detection::BBCenter_Y]]),
                        stod(data3D[detectionDatabase->boundBoxCenter_Z[detection::BBCenter_Z]]),
                        stod(data3D[detectionDatabase->rotation[detection::ROTATION]]),
                        pointCloudCalibration
                        );

                }
                else {

                    object._3dBoxLiDAR = get3DBoundingBox(
                        stod(data3D[detectionDatabase->height[detection::HEIGHT]]),
                        stod(data3D[detectionDatabase->width[detection::WIDTH]]),
                        stod(data3D[detectionDatabase->length[detection::LENGTH]]),
                        stod(data3D[detectionDatabase->boundBoxCenter_X[detection::BBCenter_X]]),
                        stod(data3D[detectionDatabase->boundBoxCenter_Y[detection::BBCenter_Y]]),
                        stod(data3D[detectionDatabase->boundBoxCenter_Z[detection::BBCenter_Z]]),
                        stod(data3D[detectionDatabase->rotation[detection::ROTATION]])
                        );
                }


                object._3dBoxLiDAR->found = true;
                object._3dBoxLiDAR->observationNumber += 1;

                object.rt = stof(data3D[detectionDatabase->rotation[detection::ROTATION]]); // 13
                object.confidenceScore = stof(data3D[detectionDatabase->confidence[detection::CONF]]); // 6
                if(object.confidenceScore  <parameters::selectedDetector->getConfThConfirmed()) continue; // lowest threshold
                bool pass = false;
                for( auto savedObject : memoryManagement.getSavedObjects())
                {
                    if(sqrt(pow(savedObject->_3dBoxLiDAR->stateEstimation(0) - object._3dBoxLiDAR->location(0), 2) + pow(savedObject->_3dBoxLiDAR->stateEstimation(1) - object._3dBoxLiDAR->location(1), 2)) < parameters::selectedDetector->getEclTh() && savedObject->isConfirmed)
                    {
                        pass = true;
                        break;
                    }
                }

                if(object.confidenceScore<selectedDetector->getConfTh()&& !pass ) continue;

                objects.push_back(object);
                found = true;
            }
            else
            {
                if(found)
                    break;
            }
        }

        file3D.close();

        for(int i = 0; i < objects.size(); i++)
        {

            auto object = new Object();
            object->_3dBoxLiDAR = objects.at(i)._3dBoxLiDAR;
            object->alpha = objects.at(i).alpha;
            object->confidenceScore = objects.at(i).confidenceScore;
            object->rt = objects.at(i).rt;
            object->_2dBox = objects.at(i)._2dBox;


            dataframe->objects.push_back(object);
        }
    }

    static Box3D *get3DBoundingBox(float h, float w, float l, float x, float y, float z, float r)  {
        Eigen::Vector4d dim;

        Eigen::Matrix3d R;
        R << std::cos(r), -std::sin(r), 0,
            std::sin(r),  std::cos(r), 0,
            0,                 0,                1;


        Eigen::Matrix<double, 8, 3> corners;
        corners <<  l / 2,  w / 2,  h / 2,
                   -l / 2,  w / 2,  h / 2,
                   -l / 2, -w / 2,  h / 2,
                    l / 2, -w / 2,  h / 2,
                    l / 2,  w / 2, -h / 2,
                   -l / 2,  w / 2, -h / 2,
                   -l / 2, -w / 2, -h / 2,
                    l / 2, -w / 2, -h / 2;

        Eigen::Vector3d location(x, y, z);

        for (int i = 0; i < 8; ++i) {
            corners.row(i) = R * corners.row(i).transpose() + location;
        }
        dim << l, w, h, r;

        return new Box3D(std::move(location),dim, corners);
    }

    static Box3D* get3DBoundingBox(float&& h, float&& w, float&& l, float&& x, float&& y, float&& z, float &&r, PointCloudCalibration& pointCloudCalibration)
    {

        Eigen::Vector4d dim;

        Eigen::Matrix3d R;
        Eigen::Matrix<double, 3, 8> corners;

        R << cos(r), 0.0, sin(r),
                0.0,    1.0,    0.0,
                -sin(r),    0.0,    cos(r);

        corners <<
                l / 2.0, l / 2.0, -l / 2.0, -l / 2.0, l / 2.0, l / 2.0, -l / 2.0, -l / 2.0,
                0.0, 0.0, 0.0, 0.0, -h, -h, -h, -h,
                w / 2.0, -w / 2.0, -w / 2.0, w / 2.0, w / 2.0, -w / 2.0, -w / 2.0, w / 2.0;

        corners = R * corners;

        Eigen::Vector3d location;

        location << x, y, z;

        pointCloudCalibration.cam2LiDAR(location);

        for(int i = 0; i < corners.cols(); i++)
        {
            corners(0, i) += x;
            corners(1, i) += y;
            corners(2, i) += z;

            Eigen::Vector3d point;
            point << corners(0, i),corners(1, i),corners(2, i);

            pointCloudCalibration.cam2LiDAR(point);

            corners(0, i) = point(0) - location(0);
            corners(1, i) = point(1) - location(1);
            corners(2, i) = point(2);

        }

        dim << l, w, h, r;

        return new Box3D(std::move(location), dim, corners.transpose());
    }


    static double distance(std::pair<int, int> point1,std::pair<int, int>  point2)
    {
        return sqrt(pow(point1.first - point2.first, 2) + pow(point1.second - point2.second, 2));
    }

    static bool boundingBoxAcceptable(std::pair<int, int> topLeft, std::pair<int, int> bottomLeft, std::pair<int, int> topRight)
    {
        // std::cout<<"distance: "<<distance(topLeft, topRight)<<std::endl;
        return distance(topLeft, bottomLeft) > 30 && distance(topLeft, topRight) > 30; //20
    }

    static bool boundingBoxAcceptable(std::pair<int, int> topLeft, std::pair<int, int> bottomRight)
    {
        std::pair<int, int> topRight(bottomRight.first, topLeft.second);
        std::pair<int, int> bottomLeft(topLeft.first, bottomRight.second);

        double diff_y = abs(topLeft.second - bottomLeft.second);
        double diff_x = abs(topLeft.first - topRight.first);

        return diff_x > 20 && diff_y > 20;
    }



    static bool checkPointInsideImage(std::pair<int, int> point)
    {
        int image_x = 1242;
        int image_y = 375;

        return (point.first > 0) && (point.first < image_x) && (  point.second > 0) && ( point.second < image_y);
    }

    static void pointModifier(std::pair<double, double>& point)
    {
        int image_x = 1242.;
        int image_y = 375.;

        point.first = (point.first < 0)?0: ((point.first > image_x)? image_x : point.first);
        point.second = ( point.second < 0)? 0 : (( point.second > image_y)? image_y :  point.second);
    }

    static bool minimumPixels(std::pair<int, int> point1, std::pair<int, int> point2)
    {
        int min =30;
        int numberOfPixels = sqrt(pow(point1.first - point2.first, 2) + pow(point1.second - point2.second, 2));
        return numberOfPixels >= min;
    }

    static void trajectoryValidationUpdate(Object& object)
    {
        if (object._3dBoxLiDAR->found) {
            object.validationScore += std::max(object.confidenceScore, 1.f) * exp(- object.intervalTime) -  object.intervalTime/std::max(object.confidenceScore, 1.f);

        }
    }

    static void ghostTracksElimination(std::vector<int>& confirmedObjects, string streamfile, string filename_j)
    {
        std::string path    =  parameters::Settings::basePath;

        std::fstream trackFile(path +"/tracking"+filename_j+"/"+streamfile.substr(0,streamfile.find('.'))+".txt");
        ofstream file;
        std::string dir_path = path + "/final" + filename_j+"/data";

        // Check if the directory exists, and create it if it doesn't
        if (!std::filesystem::exists(dir_path)) {
            if (!std::filesystem::create_directories(dir_path)) {
                std::cerr << "Failed to create directory: " << dir_path << std::endl;
            }
        }
        file.open(path + "/final"+filename_j+"/data/"+streamfile.substr(0,streamfile.find('.'))+".txt", std::ios_base::app);

        std::string data;
        std::string out = "";

        int iter =  (Settings::dataset_name == "kitti")? 16 : 8;
        while(trackFile >> data)
        {
            std::string line = "";

            line += data + " ";
            trackFile >> data; // id;
            int id = std::stoi(data);
            if(!isContained(confirmedObjects, id))
            {
                for(int i = 0 ; i < iter; i++)
                {
                    trackFile >> data; // id;
                }
                continue;
            }
            line += data + " ";
            for(int i = 0 ; i < iter; i++)
            {
                trackFile >> data; // id;
                line += data + " ";
            }


            line += "\n";
            out += line;

        }

        trackFile.close();

        if (!file.is_open())
        {
            std::cout<<"not"<<std::endl;
            return;
        }
        file << out;
        file.close();

    }

    static bool isContained(std::vector<int>& confirmedObjects, int i)
    {
        for(int value : confirmedObjects)
        {
            if(value == i)
                return true;
        }
        return false;
    }

    static void saveTrackingData(std::string stream, std::vector<Object*> objects, string streamfile, PointCloudCalibration& pointCloudCalibration, std::string filename_j)
    {
        ofstream file;

        std::string path    =  parameters::Settings::basePath;
        // Construct the full directory path
        std::string dir_path = path + "/tracking" + filename_j;

        // Check if the directory exists, and create it if it doesn't
        if (!std::filesystem::exists(dir_path)) {
            if (!std::filesystem::create_directories(dir_path)) {
                std::cerr << "Failed to create directory: " << dir_path << std::endl;
            }
        }

        file.open(path + "/tracking"+filename_j+"/"+streamfile.substr(0,streamfile.find('.'))+".txt", std::ios_base::app);
        if (!file.is_open())
        {
            std::cout<<"not"<<std::endl;
        }
        for(auto object: objects)
        {

            if (parameters::Settings::processDetectedObjectsOnly && !object->_3dBoxLiDAR->found) continue;

            if (Settings::dataset_name == "kitti") {

                Eigen::Vector3d locWorld, locLidar;

                locWorld << object->_3dBoxLiDAR->estimatedLocation(0), object->_3dBoxLiDAR->estimatedLocation(1), object->_3dBoxLiDAR->estimatedLocation(2);
                pointCloudCalibration.world2Lidar(locWorld, locLidar);
                if((!Helper::insideFOVCam(locLidar(0),locLidar(1)))) continue;
                std::pair<double, double> topLeft(object->_3dBoxLiDAR->boundingBox(0), object->_3dBoxLiDAR->boundingBox(1));
                std::pair<double, double> bottomRight(object->_3dBoxLiDAR->boundingBox(0) + object->_3dBoxLiDAR->boundingBox(2), object->_3dBoxLiDAR->boundingBox(1) + object->_3dBoxLiDAR->boundingBox(3));
                std::pair<double, double> topRight(object->_3dBoxLiDAR->boundingBox(0) + object->_3dBoxLiDAR->boundingBox(2), object->_3dBoxLiDAR->boundingBox(1));
                std::pair<double, double> bottomLeft(object->_3dBoxLiDAR->boundingBox(0), object->_3dBoxLiDAR->boundingBox(1) + object->_3dBoxLiDAR->boundingBox(3));

                if(!(Helper::checkPointInsideImage(topLeft) ||
                     Helper::checkPointInsideImage(bottomRight) ||
                     Helper::checkPointInsideImage(topRight) ||
                     Helper::checkPointInsideImage(bottomLeft)))continue;

                Helper::pointModifier(topLeft);
                Helper::pointModifier(bottomRight);

                pointCloudCalibration.LiDAR2cam(locLidar);

                    file << stream  <<" "<<object->trackId<<" Car "<<-1<<" "<<-1<<" "<<object->alpha <<" "<< topLeft.first <<" " << topLeft.second << " "<< bottomRight.first<<" " << bottomRight.second << " ";

                file <<object->_3dBoxLiDAR->estimatedLocation(0) <<" " <<object->_3dBoxLiDAR->estimatedLocation(1)<<" "
                << "-1" <<" " << locLidar(0)
                <<" " << locLidar(1)<<" " << locLidar(2)<<" "<<"-10 " <<object->confidenceScore<<"\n";

            }
            else {
                if(!object->_3dBoxLiDAR->found) continue;

                Eigen::Vector3d locWorld, locLidar;

                locWorld << object->_3dBoxLiDAR->estimatedLocation(0), object->_3dBoxLiDAR->estimatedLocation(1), object->_3dBoxLiDAR->estimatedLocation(2);
                pointCloudCalibration.world2Lidar(locWorld, locLidar);

                file    << stream << " " << object->trackId << " "<< locLidar(0)<< " " <<locLidar(1) << " "<< locLidar(2)<< " "
                        << object->_3dBoxLiDAR->dim(0) << " "<< object->_3dBoxLiDAR->dim(1) << " "
                        << object->_3dBoxLiDAR->dim(2)<< " " << object->_3dBoxLiDAR->dim(3) << " "<< object->confidenceScore << "\n";
            }
        }
        file.close();
    }


};

#endif