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

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

using namespace cv;
using namespace std;


class dataStream{
    public:


    private:

    // todo: update these params
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);

    Mat record_cam(bool indoor_lighting);

    bool get_free_msg_buf_index(uint8_t &index);

    struct
    {
        uint64_t time_send_us;
        mavlink_message_t obs_msg;
    } msg_buf[3];
    


    void maybe_send_heartbeat();
    uint32_t last_heatbeat_ms;

};