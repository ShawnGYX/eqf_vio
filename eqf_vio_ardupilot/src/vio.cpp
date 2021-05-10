#include "vio.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

struct CallbackStruct
{
    GIFT::PointFeatureTracker featureTracker;
    VIOFilter filter;

    void callbackImage(const Mat image);
    void callbackImu(const IMUVelocity imuVel);

};

void CallbackStruct::callbackImu(const IMUVelocity imuVel)
{

}

void CallbackStruct::callbackImage(const Mat image)
{

}

// Provide a heartbeat signal every so often
void dataStream::maybe_send_heartbeat()
{
    
    // Get a timestamp
    const uint32_t now = chrono::steady_clock::now().time_since_epoch().count();

    if (now - last_heatbeat_ms < 100)
    {
        return;
    }

    last_heatbeat_ms = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack( system_id,
                                component_id,
                                &msg,
                                MAV_TYPE_GCS,
                                MAV_AUTOPILOT_INVALID,
                                0,
                                0,
                                0);
}

bool dataStream::get_free_msg_buf_index(uint8_t &index)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(msg_buf); i++)
    {
        if (msg_buf[i].time_send_us == 0)
        {
            index = i;
            return true;
        }
    }
    return false;
}

bool dataStream::should_send(TypeMask type_mask) const 
{
    return 1;
}

Mat dataStream::record_cam(bool indoor_lighting)
{
    // Initialize image capture module
    cv::VideoCapture *cap;
    cap = new cv::VideoCapture(0);
    cap->set(CAP_PROP_AUTO_EXPOSURE, 0.25);

    // Adjust exposure
    float exposure;
    Mat frame;
    if (indoor_lighting)
    {
        exposure = 0.5;
    }
    else
    {
        exposure = 0.001;
    }
    float gain = 1e-4;
    for(;;)
    {
        cap->read(frame);
        if (frame.empty())
        {
            cerr << "Blank frame captured!\n";
            break;
        }

        // Set camera exposure
        cap->set(CAP_PROP_EXPOSURE, exposure);

        float img_mean = cv::mean(frame);
        if (img_mean > 128-32 && img_mean < 128+32)
        {
            continue;
        }
        exposure += gain * (128 - img_mean) * exposure;
        if (exposure > 0.7)
        {
            exposure = 0.7;
        }
        else if (exposure <=0.0)
        {
            exposure = 1e-6;
        }
    }
    
}


void dataStream::update_vp_estimate(const Location &loc,
                                    const Vector3f &position,
                                    const Vector3f &velocity,
                                    const Quaternion &attitude)
{
    const uint32_t now_us = chrono::steady_clock::now().time_since_epoch().count();

    // Calculate a random time offset to the time sent in the message
    if (time_offset_us == 0)
    {
        time_offset_us = (unsigned(random()) % 7000) * 1000000ULL;
        printf("time_off_us %llu\n", (long long unsigned)time_offset_us);
    }

    // send all messages in the buffer
    bool waiting_to_send = false;
    for (uint8_t i=0; i<ARRAY_SIZE(msg_buf); i++)
    {
        if ((msg_buf[i].time_send_us > 0) && (now_us >= msg_buf[i].time_send_us))
        {
            uint8_t buf[300];
            uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg_buf[i].obs_msg);
            msg_buf[i].time_send_us = 0;

        }
        waiting_to_send = msg_buf[i].time_send_us != 0;

    }
    if (waiting_to_send) {
        return;
    }

    if (now_us - last_observation_usec < 20000)
    {
        return;
    }

    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);


    // load estimates from the filter
    Vector3f pos_corrected;




    uint32_t delay_ms = 25 + unsigned(random()) % 100;
    uint64_t time_send_us = now_us + delay_ms * 1000UL;
    // send message
    uint8_t msg_buf_index;
    if (should_send(TypeMask::VISION_POSITION_ESTIMATE) && get_free_msg_buf_index(msg_buf_index))
    {
        mavlink_msg_vision_position_estimate_pack_chan(
            system_id,
            component_id,
            mavlink_ch,
            &msg_buf[msg_buf_index].obs_msg,
            now_us + time_offset_us,
            pos_corrected.x,
            pos_corrected.y,
            pos_corrected.z,
            roll,
            pitch,
            yaw,
            NULL, 0);
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }



    if (should_send(TypeMask::VISION_SPEED_ESTIMATE) && get_free_msg_buf_index(msg_buf_index))
    {
        mavlink_msg_vision_speed_estimate_pack_chan(
            system_id,
            component_id,
            mavlink_ch,
            &msg_buf[msg_buf_index].obs_msg,
            now_us + time_offset_us,
            vel_corrected.x,
            vel_corrected.y,
            vel_corrected.z,
            NULL, 0
        );
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }

    uint64_t time_delta = now_us - last_observation_usec;

    Quaternion attitude_curr;

    attitude_curr.from_euler(roll, pitch, yaw);

    attitude_curr.invert();

    Quaternion attitude_curr_prev = attitude_curr * _attitude_prev.inverse();

    float angle_data[3] = {
        attitude_curr_prev.get_euler_roll(),
        attitude_curr_prev.get_euler_pitch(),
        attitude_curr_prev.get_euler_yaw()};
    
    Matrix3f body_ned_m;
    attitude_curr.rotation_matrix(body_ned_m);

    

}


int main(int argc, char* argv[])
{
    
    // Read argument
    if (argc != 1)
    {
        std::cout<< "No configuration file was provided.\n";
        std::cout<< "Usage: EQVIO_config_file."<<std::endl;
        return 1;
    }
    else if (argc > 1)
    {
        std::cout<< "Too many files were provided.\n";
        std::cout<< "Usage: EQVIO_config_file."<<std::endl;
        return 1;
    }
    std::string EQVIO_config_fname(argv[1]);

    // Load EQVIO configurations
    if (!std::ifstream(EQVIO_config_fname).good())
    {
        std::stringstream ess;
        ess << "Couldn't open the configuration file: "<< EQVIO_config_fname;
        throw std::runtime_error(ess.str());
    }
    const YAML::Node eqf_vioConfig = YAML::LoadFile(EQVIO_config_fname);

    // Initialize the feature tracker and the filter
    const std::string camera_intrinsics_fname = eqf_vioConfig["GIFT"]["intrinsicsFile"].as<std::string>();
    if (!std::ifstream(camera_intrinsics_fname).good())
    {
        std::stringstream ess;
        ess << "Couldn't open the GIFT camera intrinsics file: "<< camera_intrinsics_fname;
        throw std::runtime_error(ess.str());
    }
    GIFT::PinholeCamera camera = GIFT::PinholeCamera(cv::String(camera_intrinsics_fname));
    GIFT::PointFeatureTracker featureTracker = GIFT::PointFeatureTracker(camera);
    safeConfig(eqf_vioConfig["GIFT"]["maxFeatures"], featureTracker.maxFeatures);
    safeConfig(eqf_vioConfig["GIFT"]["featureDist"], featureTracker.featureDist);
    safeConfig(eqf_vioConfig["GIFT"]["minHarrisQuality"], featureTracker.minHarrisQuality);
    safeConfig(eqf_vioConfig["GIFT"]["featureSearchThreshold"], featureTracker.featureSearchThreshold);
    safeConfig(eqf_vioConfig["GIFT"]["maxError"], featureTracker.maxError);
    safeConfig(eqf_vioConfig["GIFT"]["winSize"], featureTracker.winSize);
    safeConfig(eqf_vioConfig["GIFT"]["maxLevel"], featureTracker.maxLevel);
    

    // Mavlink related



    // Filter processing
    Mat image = dataStream::record_cam(true);
    
    
    







}