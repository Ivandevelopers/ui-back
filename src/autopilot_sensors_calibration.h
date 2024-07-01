#pragma once
#include <ardupilotmega/ardupilotmega.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


// using namespace mavsdk;

enum class AccelCalibrationPosition : uint8_t {
    LEVEL = 1,
    LEFT,
    RIGHT,
    NOSEDOWN,
    NOSEUP,
    BACK
};

struct RawImu {
    int xacc = 0;
    int yacc = 0;
    int zacc = 0;
};

void send_calibration_command(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough);

void updateImuData(std::array<RawImu, 6>& imuData, AccelCalibrationPosition pos, int xacc, int yacc, int zacc);

void send_accelcal_command(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough);

void handle_mavlink_message(const mavlink_message_t& message);

void request_sensor_offsets(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough);

void handle_sensor_offsets(const mavlink_message_t& message);