#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <MvCameraControl.h>

#include "data_manager/mvs_camera.h"

namespace {

struct MVSContext {
    void* handle = nullptr;
    std::thread grab_thread;
    std::atomic<bool> running{false};
    float* yaw = nullptr;
    float* pitch = nullptr;
    float* roll = nullptr;
    bool flip = false;
    uint32_t payload_size = 0;
    std::vector<uint8_t> bgr_buffer;
};

std::mutex g_mvs_mutex;
std::unordered_map<rm::Camera*, std::shared_ptr<MVSContext>> g_mvs_contexts;
std::atomic<int> g_mvs_ref_count{0};
std::atomic<bool> g_mvs_initialized{false};

bool ensure_mvs_initialized() {
    bool expected = false;
    if (g_mvs_initialized.compare_exchange_strong(expected, true)) {
        int ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            g_mvs_initialized.store(false);
            rm::message("MVS SDK initialize failed", rm::MSG_ERROR);
            return false;
        }
    }
    return true;
}

void finalize_mvs_if_needed() {
    if (g_mvs_ref_count.load() == 0 && g_mvs_initialized.load()) {
        MV_CC_Finalize();
        g_mvs_initialized.store(false);
    }
}

void grab_loop(const std::shared_ptr<MVSContext>& ctx, rm::Camera* camera) {
    int fail_count = 0;
    TimePoint last_log_time = getTime();
    while (ctx->running.load()) {
        MV_FRAME_OUT frame_out;
        std::memset(&frame_out, 0, sizeof(frame_out));

        int ret = MV_CC_GetImageBuffer(ctx->handle, &frame_out, 1000);
        if (ret != MV_OK) {
            fail_count++;
            if (getDoubleOfS(last_log_time, getTime()) > 2.0) {
                rm::message("MVS get image buffer failed: " + std::to_string(ret), rm::MSG_WARNING);
                last_log_time = getTime();
            }
            continue;
        }
        fail_count = 0;

        const int width = static_cast<int>(frame_out.stFrameInfo.nWidth);
        const int height = static_cast<int>(frame_out.stFrameInfo.nHeight);
        const size_t bgr_bytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;

        cv::Mat image(height, width, CV_8UC3);

        if (frame_out.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
            std::memcpy(image.data, frame_out.pBufAddr, bgr_bytes);
        } else {
            if (ctx->bgr_buffer.size() < bgr_bytes) {
                ctx->bgr_buffer.resize(bgr_bytes);
            }
            MV_CC_PIXEL_CONVERT_PARAM_EX cvt_param;
            std::memset(&cvt_param, 0, sizeof(cvt_param));
            cvt_param.nWidth = frame_out.stFrameInfo.nWidth;
            cvt_param.nHeight = frame_out.stFrameInfo.nHeight;
            cvt_param.enSrcPixelType = frame_out.stFrameInfo.enPixelType;
            cvt_param.pSrcData = frame_out.pBufAddr;
            cvt_param.nSrcDataLen = frame_out.stFrameInfo.nFrameLen;
            cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            cvt_param.pDstBuffer = ctx->bgr_buffer.data();
            cvt_param.nDstBufferSize = static_cast<unsigned int>(ctx->bgr_buffer.size());

            ret = MV_CC_ConvertPixelTypeEx(ctx->handle, &cvt_param);
            if (ret != MV_OK || cvt_param.nDstLen < bgr_bytes) {
                MV_CC_FreeImageBuffer(ctx->handle, &frame_out);
                continue;
            }

            std::memcpy(image.data, ctx->bgr_buffer.data(), bgr_bytes);
        }

        if (ctx->flip) {
            cv::flip(image, image, -1);
        }

        auto frame = std::make_shared<rm::Frame>();
        frame->image = std::make_shared<cv::Mat>(image);
        frame->time_point = getTime();
        frame->camera_id = camera->camera_id;
        frame->width = width;
        frame->height = height;
        frame->yaw = ctx->yaw ? *ctx->yaw : 0.0f;
        frame->pitch = ctx->pitch ? *ctx->pitch : 0.0f;
        frame->roll = ctx->roll ? *ctx->roll : 0.0f;

        if (camera->buffer != nullptr) {
            camera->buffer->push(frame);
        }

        MV_CC_FreeImageBuffer(ctx->handle, &frame_out);
    }
}

bool set_camera_double(void* handle, const char* key, double value) {
    int ret = MV_CC_SetFloatValue(handle, key, static_cast<float>(value));
    return ret == MV_OK;
}

bool set_camera_double_any(void* handle, const std::vector<const char*>& keys, double value) {
    for (const char* key : keys) {
        if (set_camera_double(handle, key, value)) {
            return true;
        }
    }
    return false;
}

bool set_camera_enum(void* handle, const char* key, unsigned int value) {
    int ret = MV_CC_SetEnumValue(handle, key, value);
    return ret == MV_OK;
}

bool set_camera_bool(void* handle, const char* key, bool value) {
    int ret = MV_CC_SetBoolValue(handle, key, value ? 1 : 0);
    return ret == MV_OK;
}

bool get_camera_int(void* handle, const char* key, unsigned int& value) {
    MVCC_INTVALUE st_value;
    std::memset(&st_value, 0, sizeof(st_value));
    int ret = MV_CC_GetIntValue(handle, key, &st_value);
    if (ret != MV_OK) {
        return false;
    }
    value = st_value.nCurValue;
    return true;
}

}

namespace rm {

bool getMVSCameraNum(int& device_num) {
    device_num = 0;
    if (!ensure_mvs_initialized()) {
        return false;
    }

    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));

    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        message("MVS enum devices failed", MSG_ERROR);
        return false;
    }

    device_num = static_cast<int>(device_list.nDeviceNum);
    return true;
}

bool setMVSArgs(Camera* camera, double exposure, double gain, double fps) {
    if (camera == nullptr) {
        return false;
    }

    std::shared_ptr<MVSContext> ctx;
    {
        std::lock_guard<std::mutex> lock(g_mvs_mutex);
        auto it = g_mvs_contexts.find(camera);
        if (it == g_mvs_contexts.end()) {
            return false;
        }
        ctx = it->second;
    }

    bool exp_auto_ok = set_camera_enum(ctx->handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    bool gain_auto_ok = set_camera_enum(ctx->handle, "GainAuto", MV_GAIN_MODE_OFF);

    bool exp_set_ok = set_camera_double_any(
        ctx->handle,
        {"ExposureTime", "ExposureTimeAbs"},
        exposure);
    bool gain_set_ok = set_camera_double_any(
        ctx->handle,
        {"Gain", "GainRaw", "GainAbs"},
        gain);

    bool fps_ok = true;
    fps_ok &= set_camera_bool(ctx->handle, "AcquisitionFrameRateEnable", true);
    fps_ok &= set_camera_double(ctx->handle, "AcquisitionFrameRate", fps);

    if (!exp_auto_ok) {
        message("MVS set ExposureAuto failed (ignored)", MSG_WARNING);
    }
    if (!gain_auto_ok) {
        message("MVS set GainAuto failed (ignored)", MSG_WARNING);
    }
    if (!exp_set_ok) {
        message("MVS set exposure failed (ignored)", MSG_WARNING);
    }
    if (!gain_set_ok) {
        message("MVS set gain failed (ignored)", MSG_WARNING);
    }

    if (!fps_ok) {
        message("MVS set frame rate failed (ignored)", MSG_WARNING);
    }

    return true;
}

bool openMVS(
    Camera* camera,
    int device_num,
    float* yaw,
    float* pitch,
    float* roll,
    bool flip,
    double exposure,
    double gain,
    double fps) {

    if (camera == nullptr) {
        return false;
    }

    if (!ensure_mvs_initialized()) {
        return false;
    }

    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        message("MVS enum devices failed", MSG_ERROR);
        return false;
    }

    if (device_num < 1 || static_cast<unsigned int>(device_num) > device_list.nDeviceNum) {
        message("MVS invalid device index", MSG_ERROR);
        return false;
    }

    void* handle = nullptr;
    ret = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[device_num - 1]);
    if (ret != MV_OK) {
        message("MVS create handle failed", MSG_ERROR);
        return false;
    }

    ret = MV_CC_OpenDevice(handle);
    if (ret != MV_OK) {
        message("MVS open device failed", MSG_ERROR);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    set_camera_enum(handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    set_camera_enum(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (!set_camera_enum(handle, "PixelFormat", PixelType_Gvsp_BGR8_Packed)) {
        message("MVS set PixelFormat failed (ignored)", MSG_WARNING);
    }

    bool exp_auto_ok = set_camera_enum(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    bool gain_auto_ok = set_camera_enum(handle, "GainAuto", MV_GAIN_MODE_OFF);

    bool exp_set_ok = set_camera_double_any(
        handle,
        {"ExposureTime", "ExposureTimeAbs"},
        exposure);
    bool gain_set_ok = set_camera_double_any(
        handle,
        {"Gain", "GainRaw", "GainAbs"},
        gain);

    bool fps_ok = true;
    fps_ok &= set_camera_bool(handle, "AcquisitionFrameRateEnable", true);
    fps_ok &= set_camera_double(handle, "AcquisitionFrameRate", fps);

    if (!exp_auto_ok) {
        message("MVS set ExposureAuto failed (ignored)", MSG_WARNING);
    }
    if (!gain_auto_ok) {
        message("MVS set GainAuto failed (ignored)", MSG_WARNING);
    }
    if (!exp_set_ok) {
        message("MVS set exposure failed (ignored)", MSG_WARNING);
    }
    if (!gain_set_ok) {
        message("MVS set gain failed (ignored)", MSG_WARNING);
    }

    if (!fps_ok) {
        message("MVS set frame rate failed (ignored)", MSG_WARNING);
    }

    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int payload_size = 0;
    if (!get_camera_int(handle, "Width", width) || !get_camera_int(handle, "Height", height)) {
        message("MVS get image size failed", MSG_ERROR);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }
    get_camera_int(handle, "PayloadSize", payload_size);

    if (camera->buffer == nullptr) {
        camera->buffer = new SwapBuffer<Frame>();
    }
    camera->camera_id = device_num;
    camera->width = static_cast<int>(width);
    camera->height = static_cast<int>(height);
    camera->flip = flip;

    ret = MV_CC_StartGrabbing(handle);
    if (ret != MV_OK) {
        message("MVS start grabbing failed", MSG_ERROR);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return false;
    }

    auto ctx = std::make_shared<MVSContext>();
    ctx->handle = handle;
    ctx->running.store(true);
    ctx->yaw = yaw;
    ctx->pitch = pitch;
    ctx->roll = roll;
    ctx->flip = flip;
    ctx->payload_size = payload_size;

    {
        std::lock_guard<std::mutex> lock(g_mvs_mutex);
        g_mvs_contexts[camera] = ctx;
        g_mvs_ref_count.fetch_add(1);
    }

    ctx->grab_thread = std::thread(grab_loop, ctx, camera);

    return true;
}

bool closeMVS(Camera* camera) {
    if (camera == nullptr) {
        return false;
    }

    std::shared_ptr<MVSContext> ctx;
    {
        std::lock_guard<std::mutex> lock(g_mvs_mutex);
        auto it = g_mvs_contexts.find(camera);
        if (it == g_mvs_contexts.end()) {
            return false;
        }
        ctx = it->second;
        g_mvs_contexts.erase(it);
    }

    if (ctx->handle != nullptr) {
        ctx->running.store(false);
        MV_CC_StopGrabbing(ctx->handle);
        if (ctx->grab_thread.joinable()) {
            ctx->grab_thread.join();
        }
        MV_CC_CloseDevice(ctx->handle);
        MV_CC_DestroyHandle(ctx->handle);
        ctx->handle = nullptr;
    }

    g_mvs_ref_count.fetch_sub(1);
    finalize_mvs_if_needed();

    return true;
}

}
