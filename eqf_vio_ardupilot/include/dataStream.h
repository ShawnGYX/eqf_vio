#pragma once

#include "eqf_vio/CSVReader.h"
#include "eqf_vio/IMUVelocity.h"
#include "eqf_vio/VIOFilter.h"
#include "eqf_vio/VIOFilterSettings.h"
#include "eqf_vio/VisionMeasurement.h"


#include "GIFT/PointFeatureTracker.h"
#include "GIFT/Visualisation.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include "yaml-cpp/yaml.h"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <exception>

#include "all/all.h"
#include "GCS_MAVLink/GCS.h"

#include "AP_Math/AP_Math.h"
#include "AP_Common/Location.h"





#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif



class dataStream{
    public:
    dataStream();
    ~dataStream();
    void startThreads();
    void stopThreads();
    void recv_thread();
    // void send_thread();
    void cam_thread();

    GIFT::PointFeatureTracker featureTracker;
    VIOFilter filter;

    VIOState callbackImage(const cv::Mat image);
    void callbackImu(const mavlink_message_t &IMUmsg);


    private:

    std::thread recv_th;
    std::thread cam_th;
    // std::thread send_th;
    bool stop_recv = false;
    bool stop_cam = false;
    // bool stop_send = false;
    // todo: update these params
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);

    

    bool get_free_msg_buf_index(uint8_t &index);

    uint64_t last_observation_usec;
    uint64_t time_offset_us;

    struct
    {
        uint64_t time_send_us;
        mavlink_message_t obs_msg;
    } msg_buf[3];
    

    enum class TypeMask: uint8_t {
        VISION_POSITION_ESTIMATE   = (1 << 0),
        VISION_SPEED_ESTIMATE      = (1 << 1),
        VISION_POSITION_DELTA      = (1 << 2)
    };

    bool should_send(TypeMask type_mask) const;
    void update_vp_estimate(const VIOState estimatedState);
    // void update_vp_estimate(const Location &loc,
    //                         const Vector3f &position,
    //                         const Vector3f &velocity,
    //                         const Quaternion &attitude);

    void maybe_send_heartbeat();
    uint32_t last_heartbeat_ms;

    Quaternion _attitude_prev;

    Vector3f _position_prev;

};