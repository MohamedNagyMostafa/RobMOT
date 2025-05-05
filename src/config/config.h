//
// Created by mohamed-nagy on 1/26/25.
//

#ifndef CONFIG_H
#define CONFIG_H
#pragma once
#include <map>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <utility>

#include "../datatype/Box3D.h"

namespace parameters
{
    struct Settings
    {
        constexpr static const char* dataset_name = "kitti";
        constexpr static const char* dataset = "training";
        constexpr static const bool  isTraining                 = true;
        constexpr static const bool  processDetectedObjectsOnly = true; // The output only for visible objects.
        constexpr static const bool  visualization              = false;
        constexpr static const bool selectedStreamsActivation   = false; // To run specific stream/s
        constexpr static const char* selectedStreams[1]   ={"0002"};
        //TODO: Include the fill project path. The path should end by 'RobMOT/src'
        constexpr static const char* basePath  = "/EXAMPLE/RobMOT/src";
        //TODO: Dataset directory
        constexpr static const char* basePathDataset = "TBA";
    };
    struct MotionModel
    {
        virtual ~MotionModel() = default;

        virtual float getPosVar() const = 0;
        virtual float getVelVar() const = 0;
        virtual float getAccVar() const = 0;
        virtual float getJerkVar() const = 0;

        virtual float getVelQ() const = 0;
        virtual float getAccQ() const = 0;
        virtual float getJerkQ() const = 0;

        virtual float getMemCov() const = 0;
    };


    class ConstantAccelerationModel  : public MotionModel
    {
    public:

        float getPosVar() const override;
        float getVelVar() const override;
        float getAccVar() const override;
        float getJerkVar() const override;

        float getVelQ() const override;
        float getAccQ() const override;
        float getJerkQ() const override;

        float getMemCov() const override;
    };

    struct Detector
    {
        virtual const char* getDataset() const = 0;
        virtual float getEclTh() const = 0;
        virtual float getGhostRemTh() const = 0;
        virtual float getConfTh() const = 0;
        virtual float getConfThConfirmed() const = 0;
        virtual float getXVar() const = 0;
        virtual float getYVar() const = 0;
    };

    class Casc : public Detector{
    public:
        const char* getDataset() const override;
        float getEclTh() const override;
        float getGhostRemTh() const override;
        float getConfTh() const override;
        float getConfThConfirmed() const override;
        float getXVar() const override;
        float getYVar() const override;
    };


    class CascWaymo : public Detector{
    public:
        const char* getDataset() const override;
        float getEclTh() const override;
        float getGhostRemTh() const override;
        float getConfTh() const override;
        float getConfThConfirmed() const override;
        float getXVar() const override;
        float getYVar() const override;
    };

    class Pointrcnn : public Detector {
    public:
        const char* getDataset() const override;
        float getEclTh() const override;
        float getGhostRemTh() const override;
        float getConfTh() const override;
        float getConfThConfirmed() const override;
        float getXVar() const override;
        float getYVar() const override;
    };


    class Ctrl : public Detector {
    public:
        const char* getDataset() const override;
        float getEclTh() const override;
        float getGhostRemTh() const override;
        float getConfTh() const override;
        float getConfThConfirmed() const override;
        float getXVar() const override;
        float getYVar() const override;
    };


    class Pvcnn : public Detector {
    public:
        const char* getDataset() const override;
        float getEclTh() const override;
        float getGhostRemTh() const override;
        float getConfTh() const override;
        float getConfThConfirmed() const override;
        float getXVar() const override;
        float getYVar() const override;
    };


    class Virconv : public Detector {
    public:
        const char* getDataset() const override ;
        float getEclTh() const override ;
        float getGhostRemTh() const override ;
        float getConfTh() const override;
        float getConfThConfirmed() const override;
        float getXVar() const override;
        float getYVar() const override;
    };

    class SecondIou : public Detector {
    public:
        const char* getDataset() const override ;
        float getEclTh() const override ;
        float getGhostRemTh() const override;
        float getConfTh() const override ;
        float getConfThConfirmed() const override;
        float getXVar() const override ;
        float getYVar() const override;
    };

    struct detection {
    public:
        virtual ~detection() = default;

        inline static std::string FRAME = "frame";
        inline static std::string BBCenter_X = "x";
        inline static std::string BBCenter_Y = "y";
        inline static std::string BBCenter_Z = "z";
        inline static std::string WIDTH = "width";
        inline static std::string HEIGHT = "height";
        inline static std::string LENGTH = "length";
        inline static std::string ROTATION = "rotation";
        inline static std::string CONF = "confidence";
        inline static std::string SPACE = "space";
        inline static std::string BBCorner_X = "corner x";
        inline static std::string BBCorner_Y = "corner y";
        inline static std::string BBCornerWidth = "corner width";
        inline static std::string BBCornerHeight = "corner height";

        std::map<std::string, int> frame;
        std::map<std::string, int> boundBoxCenter_X;
        std::map<std::string, int> boundBoxCenter_Y;
        std::map<std::string, int> boundBoxCenter_Z;
        std::map<std::string, int> width;
        std::map<std::string, int> height;
        std::map<std::string, int> length;
        std::map<std::string, int> rotation;
        std::map<std::string, int> confidence;
        std::map<std::string, int> totalFields;
        std::map<std::string, int> boundBoxCorner_x;
        std::map<std::string, int> boundBoxCorner_y;
        std::map<std::string, int> boundBoxCornerWidth;
        std::map<std::string, int> boundBoxCornerHeight;

    };

    struct waymo: detection {
        waymo();
    };

    struct kitti: detection {
        kitti();
    };

    // TODO: Define selectedDetector here
    inline std::unique_ptr<parameters::Detector> selectedDetector = std::make_unique<Casc>();
    inline std::unique_ptr<parameters::MotionModel> selectedMotionModel = std::make_unique<ConstantAccelerationModel>();


    std::unique_ptr<parameters::detection> initializeDatabase(const std::string& database);
}

#endif //CONFIG_H
