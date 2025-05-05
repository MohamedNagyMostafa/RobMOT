// parameters.h
#include "config.h"

 ///A. Motion Models ///
/*
 * Acceleration Motion Model
 */
float parameters::ConstantAccelerationModel::getPosVar() const  {return 2.;}
float parameters::ConstantAccelerationModel::getVelVar() const  {return 4.;}
float parameters::ConstantAccelerationModel::getAccVar() const  {return 8.;}

float parameters::ConstantAccelerationModel::getVelQ() const  {return 0.1;}
float parameters::ConstantAccelerationModel::getAccQ() const  {return 0.01;}

float parameters::ConstantAccelerationModel::getMemCov() const  {return 4.;}


///B. Detectors ///
const char* parameters::Casc::getDataset() const  { return "casce/"; }
float parameters::Casc::getEclTh() const  { return 3.0f; }
float parameters::Casc::getGhostRemTh() const  { return 25.0f; }
float parameters::Casc::getConfTh() const  { return 0.; }
float parameters::Casc::getConfThConfirmed() const  { return 0.; }
float parameters::Casc::getXVar() const  { return 0.034966f; }
float parameters::Casc::getYVar() const  { return 0.019720f; }

const char* parameters::CascWaymo::getDataset() const  { return "casce/"; }
float parameters::CascWaymo::getEclTh() const  { return 4.0f; }
float parameters::CascWaymo::getGhostRemTh() const  { return 8.0f; }
float parameters::CascWaymo::getConfTh() const  { return 0.75; }
float parameters::CascWaymo::getConfThConfirmed() const  { return 0.75; }
float parameters::CascWaymo::getXVar() const  { return 0.030696f; }
float parameters::CascWaymo::getYVar() const  { return 0.015416f; }

const char* parameters::Pointrcnn::getDataset() const  { return "pointrcnn/"; }
float parameters::Pointrcnn::getEclTh() const  { return 4.0f; }
float parameters::Pointrcnn::getGhostRemTh() const  { return 35.0f; }
float parameters::Pointrcnn::getConfTh() const  { return 0.f; }
float parameters::Pointrcnn::getConfThConfirmed() const  { return 0.0f; }
float parameters::Pointrcnn::getXVar() const  { return 0.030874f; }
float parameters::Pointrcnn::getYVar() const  { return 0.009379f; }

const char* parameters::Ctrl::getDataset() const  { return "ctrl/"; }
float parameters::Ctrl::getEclTh() const  { return 4.0f; }
float parameters::Ctrl::getGhostRemTh() const  { return 4.0f; }
float parameters::Ctrl::getConfTh() const  { return .6f; }
float parameters::Ctrl::getConfThConfirmed() const  { return 0.6f; }
float parameters::Ctrl::getXVar() const  { return 0.030696; }
float parameters::Ctrl::getYVar() const  { return 0.015416; }

const char* parameters::Pvcnn::getDataset() const  { return "pvcnn/"; }
float parameters::Pvcnn::getEclTh() const  { return 2.0f; }
float parameters::Pvcnn::getGhostRemTh() const  { return 20.0f; }
float parameters::Pvcnn::getConfTh() const  { return 0.5f; }
float parameters::Pvcnn::getConfThConfirmed() const  { return 0.5f; }
float parameters::Pvcnn::getXVar() const  { return 0.036383f; }
float parameters::Pvcnn::getYVar() const  { return 0.013067f; }

const char* parameters::Virconv::getDataset() const  { return "virconv/"; }
float parameters::Virconv::getEclTh() const  { return 4.0f; }
float parameters::Virconv::getGhostRemTh() const  { return 20.0f; }
float parameters::Virconv::getConfTh() const  { return 0.f; } // TODO:0
float parameters::Virconv::getConfThConfirmed() const  { return -1.0f; }
float parameters::Virconv::getXVar() const  { return 0.017221; }
float parameters::Virconv::getYVar() const  { return  0.005901; }

const char* parameters::SecondIou::getDataset() const  { return "second_iou/"; }
float parameters::SecondIou::getEclTh() const  { return 3.f; }
float parameters::SecondIou::getGhostRemTh() const  { return 10.0f; }
float parameters::SecondIou::getConfTh() const  { return -1.f; } // TODO: -1.
float parameters::SecondIou::getConfThConfirmed() const  { return -2.0f; }
float parameters::SecondIou::getXVar() const  { return 0.039156f; }
float parameters::SecondIou::getYVar() const  { return 0.014357f; }

parameters::kitti::kitti() {
    frame[FRAME] = 0;
    boundBoxCorner_x[BBCorner_X] = 5;
    boundBoxCorner_y[BBCorner_Y] = 6;
    boundBoxCornerWidth[BBCornerWidth] = 7;
    boundBoxCornerHeight[BBCornerHeight] = 8;
    height[HEIGHT] = 9;
    width[WIDTH] = 10;
    length[LENGTH] = 11;
    boundBoxCenter_X[BBCenter_X] = 12;
    boundBoxCenter_Y[BBCenter_Y] = 13;
    boundBoxCenter_Z[BBCenter_Z] = 14;
    rotation[ROTATION] = 15;
    confidence[CONF] = 16;

    totalFields[SPACE] = 18;
}

parameters::waymo::waymo(){
    frame[FRAME] = 0;
    boundBoxCenter_X[BBCenter_X] = 1;
    boundBoxCenter_Y[BBCenter_Y] = 2;
    boundBoxCenter_Z[BBCenter_Z] = 3;
    width[WIDTH] = 5;
    height[HEIGHT] = 6;
    length[LENGTH] = 4;
    rotation[ROTATION] = 7;
    confidence[CONF] = 8;

    totalFields[SPACE] = 9;
}


std::unique_ptr<parameters::detection> parameters::initializeDatabase(const std::string& database) {
    if (database == "waymo") {
        return std::make_unique<waymo>();
    }

    if (database == "kitti") {
        return std::make_unique<kitti>();
    }
    throw std::invalid_argument("Unknown dataset type");

}



