//
// Created by tgu on 2026/5/21.
//

#include "hikrobot.hpp"

#include <cstdlib>
#include <libusb-1.0/libusb.h>

#include "tools/logger.hpp"
#include "tools/time.hpp"
#include "tools/tomlpp.hpp"

namespace io {
    Hikrobot::Hikrobot(std::string_view cfg_file_path)
        : cfg_file_path_(cfg_file_path) {
        auto config = toml::parse_file(cfg_file_path);

        device_id_ = config["camera"]["camera_id"].value_or("");

        exposure_us_ = config["camera"]["exposure_us"].value_or(4000.0f);

        gain_ = config["camera"]["gain"].value_or(5.0f);

        n_ret_ = MV_CC_Initialize();

        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "SDK init failed, n_ret: {:#x}", n_ret_);
        }

        LOG_DEBUG(MODULE, "HIKSDK Version: {:#x}", MV_CC_GetSDKVersion());
    }

    Hikrobot::~Hikrobot() {
        close();

        MV_CC_Finalize();
    }

    void Hikrobot::close() {
        running_ = false;

        if (camera_handle_) {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);

            camera_handle_ = nullptr;
        }

        if (pDataForRGB_) {
            free(pDataForRGB_);
            pDataForRGB_ = nullptr;
        }
    }

    bool Hikrobot::init() {
        close();

        // 检查libusb
        if (libusb_init(nullptr)) {
            LOG_ERROR(MODULE, "Unable to init libusb!");
            return false;
        }

        memset(&device_list_, 0, sizeof(device_list_));

        // 枚举设备
        n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);

        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_EnumDevices fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        if (device_list_.nDeviceNum == 0) {
            LOG_WARN(MODULE, "No camera found");
            return false;
        }

        LOG_DEBUG(MODULE, "Found camera count = {}", device_list_.nDeviceNum);

        int id = -1;

        for (uint32_t i = 0; i < device_list_.nDeviceNum; ++i) {
            auto *info = device_list_.pDeviceInfo[i];

            if (info->nTLayerType != MV_USB_DEVICE) {
                continue;
            }

            std::string current_id = reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);

            if (current_id == device_id_) {
                id = static_cast<int>(i);
                break;
            }
        }

        if (id < 0) {
            LOG_ERROR(MODULE, "Camera not found, target sn={}", device_id_);
            return false;
        }

        LOG_INFO(MODULE, "Use camera: {}", device_id_);

        // 创建句柄
        n_ret_ = MV_CC_CreateHandle(&camera_handle_, device_list_.pDeviceInfo[id]);
        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_CreateHandle fail, n_ret: {:#x}", n_ret_);
            close();
            return false;
        }

        // 打开设备
        n_ret_ = MV_CC_OpenDevice(camera_handle_);
        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_OpenDevice fail, n_ret: {:#x}", n_ret_);
            close();
            return false;
        }

        // USB传输大小
        n_ret_ = MV_USB_SetTransferSize(camera_handle_, 0x2000000);
        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_USB_SetTransferSize fail, n_ret: {:#x}", n_ret_);
            close();
            return false;
        }

        // 设置触发模式为off
        n_ret_ = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
        if(n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_SetTriggerMode fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        // 设置采集模式为连续采集
        n_ret_ = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2);

        if(n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_SetAcquisitionMode fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        // 设置为8bit位深
        n_ret_ = MV_CC_SetEnumValue(camera_handle_, "ADCBitDepth", 0);
        if(n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_SetADCBitDepth fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        // 设置像素格式BayerRG8
        n_ret_ = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
        if(n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_SetPixelFormat fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        // 白平衡
        MV_CC_SetEnumValue(camera_handle_, "BalanceWhiteAuto", 1);

        // 自动曝光关闭
        MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);

        // 曝光时间
        n_ret_ = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_us_);
        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_SetExposureTime fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        // 增益
        n_ret_ = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_SetGain fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        // 设置插值算法类型
        MV_CC_SetBayerCvtQuality(camera_handle_, 1);

        // 获取宽高
        MVCC_INTVALUE width{};
        MVCC_INTVALUE height{};

        MV_CC_GetIntValue(camera_handle_, "Width", &width);

        MV_CC_GetIntValue(camera_handle_, "Height", &height);

        width_ = width.nCurValue;
        height_ = height.nCurValue;

        LOG_DEBUG(MODULE, "Resolution: {}x{}", width_, height_);

        // RGB Buffer
        size_t rgb_size = width_ * height_ * 4 + 2048;

        pDataForRGB_ = static_cast<unsigned char *>(malloc(rgb_size));

        if (!pDataForRGB_) {
            LOG_ERROR(MODULE, "malloc RGB buffer failed");
            return false;
        }

        LOG_DEBUG(MODULE, "RGB buffer size: {}", rgb_size);

        return true;
    }

    bool Hikrobot::start() {
        if (!camera_handle_) {
            return false;
        }

        n_ret_ = MV_CC_StartGrabbing(camera_handle_);

        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_StartGrabbing fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        running_ = true;

        LOG_INFO(MODULE, "Camera streaming started");

        return true;
    }

    bool Hikrobot::grab(cv::Mat &image, uint64_t &timestamp_ns) {
        if (!camera_handle_) {
            return false;
        }

        n_ret_ = MV_CC_GetImageBuffer(camera_handle_, &frameOut_, 1000);

        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_GetImageBuffer fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        pstCvtParam_ = {0};

        pstCvtParam_.nWidth = frameOut_.stFrameInfo.nWidth;

        pstCvtParam_.nHeight = frameOut_.stFrameInfo.nHeight;

        pstCvtParam_.pSrcData = frameOut_.pBufAddr;

        pstCvtParam_.nSrcDataLen = frameOut_.stFrameInfo.nFrameLen;

        pstCvtParam_.enSrcPixelType = frameOut_.stFrameInfo.enPixelType;

        pstCvtParam_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

        pstCvtParam_.pDstBuffer = pDataForRGB_;

        pstCvtParam_.nDstBufferSize = width_ * height_ * 4 + 2048;

        n_ret_ = MV_CC_ConvertPixelTypeEx(camera_handle_, &pstCvtParam_);

        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_ConvertPixelTypeEx fail, n_ret: {:#x}", n_ret_);

            MV_CC_FreeImageBuffer(camera_handle_, &frameOut_);

            return false;
        }

        cv::Mat tmp(frameOut_.stFrameInfo.nHeight, frameOut_.stFrameInfo.nWidth, CV_8UC3, pDataForRGB_);

        tmp.copyTo(image);

        timestamp_ns = tools::steady_time_ns();

        n_ret_ = MV_CC_FreeImageBuffer(camera_handle_, &frameOut_);

        if (n_ret_ != MV_OK) {
            LOG_ERROR(MODULE, "MV_CC_FreeImageBuffer fail, n_ret: {:#x}", n_ret_);
            return false;
        }

        return true;
    }

    bool Hikrobot::is_running() const {
        return running_;
    }
} // namespace io
