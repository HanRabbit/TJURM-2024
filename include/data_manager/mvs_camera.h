#ifndef TJURM_2024_MVS_CAMERA_H_
#define TJURM_2024_MVS_CAMERA_H_

#include <openrm.h>

namespace rm {

bool getMVSCameraNum(int& device_num);

bool setMVSArgs(
    Camera* camera,
    double exposure,
    double gain,
    double fps);

bool openMVS(
    Camera* camera,
    int device_num = 1,
    float* yaw = nullptr,
    float* pitch = nullptr,
    float* roll = nullptr,
    bool flip = false,
    double exposure = 2000.0,
    double gain = 15.0,
    double fps = 200.0);

bool closeMVS(Camera* camera);

}

#endif
