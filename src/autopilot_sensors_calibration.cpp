#include <iostream>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include "autopilot_sensors_calibration.h"


// using namespace mavsdk;

void send_calibration_command(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough) {
    mavsdk::MavlinkPassthrough::CommandLong command;

    command.target_sysid = mavlink_passthrough->get_target_sysid();
    command.target_compid = mavlink_passthrough->get_target_compid();
    command.command = MAV_CMD_PREFLIGHT_CALIBRATION;
    command.param1 = 0.0;  // Gyro calibration
    command.param2 = 0.0;  // Magnetometer calibration
    command.param3 = 0.0;  // Ground pressure calibration
    command.param4 = 0.0;  // Radio calibration
    command.param5 = 1.0;  // Accelerometer calibration
    command.param6 = 0.0;  // Vehicle position
    command.param7 = 0.0;  // Empty

    mavsdk::MavlinkPassthrough::Result result = mavlink_passthrough->send_command_long(command);

    if(result == mavsdk::MavlinkPassthrough::Result::Success) {
        std::cout << "Accel cal success" << std::endl;
    } else {
        std::cout << "Accel failed " << result << std::endl;
    }   
}

void updateImuData(std::array<RawImu, 6>& imuData, AccelCalibrationPosition pos, int xacc, int yacc, int zacc) {
    int index = static_cast<int>(pos) - 1;  // Приведення enum до індексу масиву
    imuData[index].xacc = xacc;
    imuData[index].yacc = yacc;
    imuData[index].zacc = zacc;
}

void send_accelcal_command(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough) {
    mavsdk::MavlinkPassthrough::CommandLong command;

    command.target_sysid = mavlink_passthrough->get_target_sysid();
    command.target_compid = mavlink_passthrough->get_target_compid();
    command.command = MAV_CMD_ACCELCAL_VEHICLE_POS;
    command.param1 = static_cast<float>(ACCELCAL_VEHICLE_POS_RIGHT);  // Gyro calibration
    command.param2 = 0.0;  // Magnetometer calibration
    command.param3 = 0.0;  // Ground pressure calibration
    command.param4 = 0.0;  // Radio calibration
    command.param5 = 0.0;  // Accelerometer calibration
    command.param6 = 0.0;  // Vehicle position
    command.param7 = 0.0;  // Empty

    std::cout << "ACCELCAL" << std::endl;

    mavlink_passthrough->send_command_long(command);
}

void handle_mavlink_message(const mavlink_message_t& message) {
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t commandLong;
        mavlink_msg_command_long_decode(&message, &commandLong);

        std::cout << "AAA" << std::endl;

        std::cout << commandLong.command << std::endl;
        std::cout << commandLong.confirmation << std::endl;
        std::cout << commandLong.param1 << std::endl;


        if (commandLong.command == MAV_CMD_ACCELCAL_VEHICLE_POS) {
            switch (static_cast<int>(commandLong.param1)) {
                case ACCELCAL_VEHICLE_POS_LEVEL:
                    std::cout << "Received ACCELCAL_VEHICLE_POS_LEVEL" << std::endl;
                    // Дії для калібрування в рівній позиції
                    break;
                case ACCELCAL_VEHICLE_POS_LEFT:
                    std::cout << "Received ACCELCAL_VEHICLE_POS_LEFT" << std::endl;
                    // Дії для калібрування на лівій стороні
                    break;
                case ACCELCAL_VEHICLE_POS_RIGHT:
                    std::cout << "Received ACCELCAL_VEHICLE_POS_RIGHT" << std::endl;
                    // Дії для калібрування на правій стороні
                    break;
                case ACCELCAL_VEHICLE_POS_NOSEDOWN:
                    std::cout << "Received ACCELCAL_VEHICLE_POS_NOSEDOWN" << std::endl;
                    // Дії для калібрування в позиції носом вниз
                    break;
                case ACCELCAL_VEHICLE_POS_NOSEUP:
                    std::cout << "Received ACCELCAL_VEHICLE_POS_NOSEUP" << std::endl;
                    // Дії для калібрування в позиції носом вгору
                    break;
                case ACCELCAL_VEHICLE_POS_BACK:
                    std::cout << "Received ACCELCAL_VEHICLE_POS_BACK" << std::endl;
                    // Дії для калібрування на задній стороні
                    break;
                default:
                    std::cerr << "Unknown position: " << commandLong.param1 << std::endl;
                    break;
            }
        }

        if (commandLong.command == MAV_CMD_PREFLIGHT_CALIBRATION) {
            std::cout << "mavlink cmd preflight cal" << std::endl;
            }
        }
    }

void request_sensor_offsets(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough) {
    mavsdk::MavlinkPassthrough::CommandLong command;

    command.target_sysid = mavlink_passthrough->get_target_sysid();
    command.target_compid = mavlink_passthrough->get_target_compid();
    command.command = MAV_CMD_REQUEST_MESSAGE;
    command.param1 = 150.0;  // SENSOR_OFFSETS
    command.param2 = 0.0;
    command.param3 = 0.0;
    command.param4 = 0.0;
    command.param5 = 0.0;
    command.param6 = 0.0;
    command.param7 = 0.0;

    std::cout << "Requesting sensor offsets..." << std::endl;
    mavlink_passthrough->send_command_long(command);
}

void handle_sensor_offsets(const mavlink_message_t& message) {
    mavlink_sensor_offsets_t sensor_offsets;
    mavlink_msg_sensor_offsets_decode(&message, &sensor_offsets);

    std::cout << "SENSOR_OFFSETS received:" << std::endl;
    std::cout << "  mag_ofs_x: " << sensor_offsets.mag_ofs_x << std::endl;
    std::cout << "  mag_ofs_y: " << sensor_offsets.mag_ofs_y << std::endl;
    std::cout << "  mag_ofs_z: " << sensor_offsets.mag_ofs_z << std::endl;
    std::cout << "  mag_declination: " << sensor_offsets.mag_declination << std::endl;
    std::cout << "  raw_press: " << sensor_offsets.raw_press << std::endl;
    std::cout << "  raw_temp: " << sensor_offsets.raw_temp << std::endl;
    std::cout << "  gyro_cal_x: " << sensor_offsets.gyro_cal_x << std::endl;
    std::cout << "  gyro_cal_y: " << sensor_offsets.gyro_cal_y << std::endl;
    std::cout << "  gyro_cal_z: " << sensor_offsets.gyro_cal_z << std::endl;
    std::cout << "  accel_cal_x: " << sensor_offsets.accel_cal_x << std::endl;
    std::cout << "  accel_cal_y: " << sensor_offsets.accel_cal_y << std::endl;
    std::cout << "  accel_cal_z: " << sensor_offsets.accel_cal_z << std::endl;
}