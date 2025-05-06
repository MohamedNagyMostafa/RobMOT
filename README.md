# RobMOT: Robust 3D Multi-Object Tracking Framework

[![arXiv](https://img.shields.io/badge/arXiv-2405.11536-b31b1b.svg)](https://arxiv.org/abs/2405.11536)
[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/robmot-robust-3d-multi-object-tracking-by/multiple-object-tracking-on-kitti-test-online)](https://paperswithcode.com/sota/multiple-object-tracking-on-kitti-test-online?p=robmot-robust-3d-multi-object-tracking-by)
[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/robmot-robust-3d-multi-object-tracking-by/3d-multi-object-tracking-on-waymo-open-1)](https://paperswithcode.com/sota/3d-multi-object-tracking-on-waymo-open-1?p=robmot-robust-3d-multi-object-tracking-by)


<img src="docs/teaser.gif" alt="Tracking Visualization" width="800"/>

Official implementation of "**RobMOT: Robust 3D Multi-Object Tracking by Observational Noise and State Estimation Drift Mitigation on LiDAR PointCloud**" (Online Method). 

## Key Features
- 🚀 **State Estimation Refinement**: Novel Kalman Filter adaptation reduces localization noise by 4.8% HOTA
- 👻 **Ghost Track Mitigation**: Introduces the first trajectory validity mechanism in object tracking, reduces false positives by 80%
- ⚡ **Real-Time Performance**: 3221 FPS on single CPU - 50x faster than SOTA methods
- 🔄 **Occlusion Handling**: Improved recovery after prolonged occlusions with uncertainty-aware tracking
- 📊 **Multi-Detector Support**

## Benchmarks
| Dataset       | MOTA ↑ | HOTA ↑ | IDSW ↓ |
|---------------|--------|--------|--------|
| KITTI Test    |91.02%  | 81.76% | 7      |
| KITTI Val     |86.31%  | 91.53% | 1      |
-------------------------------------------

| Waymo Test (Vehicle)| MOTA/L1 ↑ | MOTA/L2 ↑ |
|---------------|--------|--------|
| All    |77.72%|74.66%|
Range-[0.30)|92.17%|91.59%|
Range-[30, 50)|78.78%|75.99%|
Range-[50,+inf)|59.74%|55.14%|
-------------------------------------------


## Installation

### Requirements
- **OS**: Ubuntu 20.04/22.04 (64-bit)
- **Compiler**: GCC 13+ (C++17 required)
- **CMake**: 3.28+
- **Dependencies**:
  - OpenCV 4.1+
  - PCL 1.14+
  - Eigen3 3.4+

```bash
# Install dependencies
sudo apt-get install -y \
    gcc-13 g++-13 \
    libopencv-dev \
    libpcl-dev \
    libeigen3-dev

# Clone repository
git clone https://github.com/yourusername/RobMOT.git
cd RobMOT

```
## Data Preparation
Organize the dataset as: (**Note the code does not include visualization, you may exclude directories with "# No need with no visualization")
```
datasets/
├── kitti/           
    ├── Camera # No need with no visualization
        ├── training 
            ├── 0000
            ...
        ├── testing
            ├── 0000
            ...
    ├── LiDAR
        ├── training
            ├── calib
                ├── 0000.txt
                ...
            ├── pose
                ├── 0000.txt
                ...
            ├── velodyne # No need with no visualization
                ├── 0000
                ...
        ├── testing
            ├── calib
                ├── 0000.txt
                ...
            ├── pose
                ├── 0000.txt
                ... 
            ├── velodyne # No need with no visualization
                ├── 0000
                ..

├── waymo/
    ├── Camera # No need with no visualization
        ├── training
            ├── 17065833287841703_2980_000_3000_000
            ...
        ├── testing
            ├── 2601205676330128831_4880_000_4900_000.parquet
            ...
    ├── LiDAR
        ├── training
            ├── pose
                ├── 17065833287841703_2980_000_3000_000.txt
                ...
            ├── velodyne # No need with no visualization
                ├── 17065833287841703_2980_000_3000_000
                ...
        ├── testing
            ├── pose
                ├── 17065833287841703_2980_000_3000_000.txt
                ...
            ├── velodyne # No need with no visualization
                ├── 17065833287841703_2980_000_3000_000
                ...
```

## Configuration
All directories should be set up correctly to avoid issues when running the software. **Please note all detector data are in folder `advanced_detection`**.

**`config/config.cpp`**
```cpp
        //TODO: Include the fill project path. The path should end by 'RobMOT/src'
        constexpr static const char* basePath  = "/EXAMPLE/RobMOT/src"; # The directory where you placed the repository, update "/EXAMPLE" only.
```
**`main.cpp`**
```cpp
            const string BASE_DIR               = "EXAMPLE/datasets/"; // TODO: Update to the dataset directory on your machine, update "/EXAMPLE" only.
```

### Configuration Options
**`config/config.cpp`**
```cpp
        # select dataset -> values: kitti or waymo
        constexpr static const char* dataset_name = "kitti";
        # select dataset type -> values: training or testing
        constexpr static const char* dataset = "training";
        # true/false based on the dataset type.   
        constexpr static const bool  isTraining                 = true;
        # To not include state estimation in the results when objects are occluded. (Note: some datasets do not have ground truth for occluded objects)
        constexpr static const bool  processDetectedObjectsOnly = true;
        # To run a specific stream. Add a single or multiple stream names to the list, and put true for the `selectedStreamsActivation`
        constexpr static const bool selectedStreamsActivation   = false; // To run specific stream/s
        constexpr static const char* selectedStreams[1]   ={"0002"};

....
        // Define selectedDetector here
        # Change the detector name, Options:
        # for kitti (Virconv, Casc, Pointrcnn, Pvcnn, SecondIou),
        # for waymo (CascWaymo, Ctrl)
        inline std::unique_ptr<parameters::Detector> selectedDetector = std::make_unique<Virconv>();
```
## Citation
```
@article{nagy2024robmot,
  title={RobMOT: Robust 3D Multi-Object Tracking by Observational Noise and State Estimation Drift Mitigation on LiDAR PointCloud},
  author={Nagy, Mohamed and Werghi, Naoufel and Hassan, Bilal and Dias, Jorge and Khonji, Majid},
  journal={arXiv preprint arXiv:2405.11536},
  year={2024}
}
```
