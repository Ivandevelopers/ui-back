#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <mavsdk/plugins/mission/mission.h>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <mavsdk/mavsdk.h>

#include <arpa/inet.h>
#include <fstream>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "autopilot_mission.h"
#include "autopilot_params.h"

using namespace mavsdk;

// #define DEBUG

#define PORT 10000

#define TELEMETRY_PORT "udp://:14550" // 14552

#define PACKET_SIZE 93

#define PACKET_SIZE_RX 12

// dev
#define DIR_NAME "/data/"
#define MISSION_WP_FILENAME "mission.plan"
#include <pwd.h>
std::string destDirPath;
std::string getDestDirPath()
{
  if (!destDirPath.empty())
  {
    return destDirPath;
  }

  const char *homeEnv = getenv("HOME");
  if (homeEnv != nullptr)
  {
    destDirPath = homeEnv;
  }
  else
  {
    destDirPath = getpwuid(getuid())->pw_dir;
  }
  destDirPath += DIR_NAME;

  return destDirPath;
}

int flag_read_waypoint = 0;
int flag_write_waypoint = 0;
int flag_read_param = 0;
int flag_write_param = 0;

uint32_t result = 0;

uint8_t crc_buf[] = {0};

char header[] = {0};

#define GPS_MESAGE 0x01
double gps_lon_val;
double gps_lat_val;
float gps_alt_val;

float gps_hdop_val;
float gps_vdop_val;
int gps_num_satellites_val;

unsigned char gps_lon_byte[sizeof(gps_lon_val)];
unsigned char gps_lat_byte[sizeof(gps_lat_val)];
unsigned char gps_alt_byte[sizeof(gps_alt_val)];

unsigned char gps_hdop_byte[sizeof(gps_hdop_val)];
unsigned char gps_vdop_byte[sizeof(gps_vdop_val)];
unsigned char gps_num_satellites_byte[sizeof(gps_num_satellites_val)];

#define VELOCITY_MESAGE 0x02
float velocity_north_direction;
float velocity_east_direction;
float velocity_down_direction;

unsigned char velocity_north_byte[sizeof(velocity_north_direction)];
unsigned char velocity_east_byte[sizeof(velocity_east_direction)];
unsigned char velocity_down_byte[sizeof(velocity_down_direction)];

#define COMPASS_MESAGE 0x03
int compass_azimuth_val;

unsigned char compass_azimuth_byte[sizeof(compass_azimuth_val)];

#define IMU_MESAGE 0x04
float imu_yaw_val;
float imu_pitch_val;
float imu_rool_val;

unsigned char imu_yaw_byte[sizeof(imu_yaw_val)];
unsigned char imu_pitch_byte[sizeof(imu_pitch_val)];
unsigned char imu_rool_byte[sizeof(imu_rool_val)];

#define BATTERY_MESAGE 0x05
float batt_voltage;
float batt_current;
float batt_charge;
float batt_remaining;

unsigned char batt_voltage_byte[sizeof(batt_voltage)];
unsigned char batt_current_byte[sizeof(batt_current)];
unsigned char batt_charge_byte[sizeof(batt_charge)];
unsigned char batt_remaining_byte[sizeof(batt_remaining)];

// connection status
int connection_status_val = 0;

unsigned char connection_status_byte[sizeof(connection_status_val)];

// flight mode
int flight_mode_val = 0;

unsigned char flight_mode_byte[sizeof(flight_mode_val)];

// ready to fly
int ready_to_fly_val = 0;

unsigned char ready_to_fly_byte[sizeof(ready_to_fly_val)];

struct Packet
{
  unsigned char data[PACKET_SIZE];
  uint16_t crc16;
};

Packet packet;

struct PacketRX
{
  unsigned char data[PACKET_SIZE_RX];
  uint16_t crc16;
};

PacketRX packetRX;

const uint16_t crc16_tab[256] = {
    0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0xe70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x2b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0xed1, 0x1ef0};

/***********************************************************
CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
***********************************************************/
uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init)
{
  uint16_t crc, oldcrc16;
  uint8_t temp;
  crc = crc_init;
  while (len-- != 0)
  {
    temp = (crc >> 8) & 0xff;
    oldcrc16 = crc16_tab[*ptr ^ temp];
    crc = (crc << 8) ^ oldcrc16;
    ptr++;
  }
  // crc=~crc; //crc invert
  return (crc);
}

uint8_t crc_check_16bites(uint8_t *pbuf, uint32_t len, uint32_t *p_result)
{
  uint16_t crc_result = 0;
  crc_result = CRC16_cal(pbuf, len, 0);
  *p_result = crc_result;
  return 2;
}

void mavlink_message_callback(const mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_VFR_HUD:
  {
    mavlink_vfr_hud_t vfr_hud;
    mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
    // std::cout << "Azimuth: " << vfr_hud.heading << std::endl;
    compass_azimuth_val = vfr_hud.heading;
    break;
  }
  }

  // std::cout << "74 compass" << std::endl;
}

int main(int argc, char **argv)
{
  int cliSockDes, readStatus;
  struct sockaddr_in serAddr;
  socklen_t serAddrLen;
  char msg[] = "Hello!!!\n";
  char buff[1024] = {0};

  // create a socket
  if ((cliSockDes = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket creation error...\n");
    exit(-1);
  }

  // server socket address
  serAddr.sin_family = AF_INET;
  serAddr.sin_port = htons(PORT);

  serAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

  Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};

  // auto connection_result = mavsdk.add_any_connection("udp://:14550");
  auto connection_result = mavsdk.add_any_connection(TELEMETRY_PORT);

  if (connection_result == mavsdk::ConnectionResult::Success)
  {
    std::cout << "Connected!" << std::endl;
  }

  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();
  Mavsdk::NewSystemHandle handle =
      mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]()
                                     {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
          std::cout << "Discovered Autopilot from Client." << std::endl;
          connection_status_val = 1;

          mavsdk.unsubscribe_on_new_system(handle);
          prom.set_value(system);
        } else {
          std::cout << "No MAVSDK found." << std::endl;
        } });

  if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout)
  {
    std::cout << "No autopilot found, exiting." << std::endl;
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(10));

  auto system = fut.get();

  auto param = std::make_shared<Param>(system);
  auto telemetry = std::make_shared<Telemetry>(system);
  auto mission_raw = std::make_shared<MissionRaw>(system);

  // MavlinkPassthrough mavlink_passthrough(system);

  auto mavlink_passthrough = std::make_shared<MavlinkPassthrough>(system);

  mavlink_passthrough->subscribe_message(74, mavlink_message_callback);

  std::cout << "Subscribe message: " << std::endl;

  // altitude & gps (long, lat)
  telemetry->subscribe_position([](Telemetry::Position position)
                                {
    // std::cout << "Altitude & GPS (long, lat)" << std::endl;
    // std::cout << "Altitude: " << position.absolute_altitude_m
    //           << " Latitude: " << position.latitude_deg
    //           << " Longitude: " << position.longitude_deg << std::endl;

    gps_alt_val = position.absolute_altitude_m;
    gps_lon_val = position.longitude_deg;
    gps_lat_val = position.latitude_deg; });

  // gps hdop/vdop
  telemetry->subscribe_raw_gps([](Telemetry::RawGps raw_gps)
                               {
                                //  std::cout << "Gps hdop: " << raw_gps.hdop
                                //            << "  gps vdop: " << raw_gps.vdop << std::endl;

                                 gps_hdop_val = raw_gps.hdop;
                                 gps_vdop_val = raw_gps.vdop; });

  // gps number of satellites:
  telemetry->subscribe_gps_info([](Telemetry::GpsInfo gps_info)
                                {
    // std::cout << "Gps number of satellites: " << gps_info.num_satellites << std::endl;

    gps_num_satellites_val = gps_info.num_satellites; });

  // flight mode
  telemetry->subscribe_flight_mode([](Telemetry::FlightMode flight_mode)
                                   {
                                     // std::cout << "Flight mode: " << flight_mode << std::endl;

                                     switch (flight_mode)
                                     {
                                     case Telemetry::FlightMode::Acro:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Altctl:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::FollowMe:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Hold:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Land:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Manual:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Mission:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Offboard:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Posctl:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Rattitude:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Ready:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::ReturnToLaunch:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Stabilized:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Takeoff:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;

                                     case Telemetry::FlightMode::Unknown:
                                       flight_mode_val = static_cast<int>(flight_mode);

                                       //  std::cout << static_cast<int>(flight_mode) << std::endl;
                                       break;
                                     }

                                     // std::cout << "Flight mode int: " << flight_mode_val << std::endl;
                                   });

  // ready to fly

  bool result_health_all_ok = telemetry->health_all_ok();

  if (result_health_all_ok == 1)
  {
    ready_to_fly_val = 1;
  }

  // velocity
  telemetry->subscribe_velocity_ned([](Telemetry::VelocityNed vel_ned)
                                    {
    // std::cout << "Velocity" << std::endl;
    // std::cout << "Velocity along north direction in metres per second: "
    //           << vel_ned.north_m_s << " "
    //           << "velocity along east direction in metres per second: "
    //           << vel_ned.east_m_s << " "
    //           << "velocity along down direction in metres per second: "
    //           << vel_ned.down_m_s << std::endl;

    velocity_north_direction = vel_ned.north_m_s;
    velocity_east_direction = vel_ned.east_m_s;
    velocity_down_direction = vel_ned.down_m_s; });

  // roll, pitch & yaw
  telemetry->subscribe_attitude_angular_velocity_body(
      [](Telemetry::AngularVelocityBody angular_velocity_body)
      {
        // std::cout << "Roll, pitch, yaw" << std::endl;
        // std::cout << "Roll angular velocity: "
        //           << angular_velocity_body.roll_rad_s << std::endl
        //           << "Pitch angular velocity: "
        //           << angular_velocity_body.pitch_rad_s << std::endl
        //           << "Yaw angular velocity: " << angular_velocity_body.yaw_rad_s
        //           << '\n';

        imu_rool_val = angular_velocity_body.roll_rad_s;
        imu_pitch_val = angular_velocity_body.pitch_rad_s;
        imu_yaw_val = angular_velocity_body.yaw_rad_s;
      });

  // battery
  telemetry->subscribe_battery([](Telemetry::Battery battery)
                               {
    // std::cout << "Battery" << std::endl;
    // std::cout << "Voltage: " << battery.voltage_v << std::endl
    //           << "Battery current: " << battery.current_battery_a << std::endl
    //           << "Consumed charge: " << battery.capacity_consumed_ah
    //           << std::endl
    //           << "Estimated battery remaining: " << battery.remaining_percent
    //           << '\n';

    batt_voltage = battery.voltage_v;
    batt_current = battery.current_battery_a;
    batt_charge = battery.capacity_consumed_ah;
    batt_remaining = battery.remaining_percent; });

  while (1)
  {

    // Convert float to byte

    // #define GPS_MESAGE 0x01
    memcpy(gps_lon_byte, &gps_lon_val, sizeof(double));
    memcpy(gps_lat_byte, &gps_lat_val, sizeof(double));
    memcpy(gps_alt_byte, &gps_alt_val, sizeof(float));

    // #define VELOCITY_MESAGE 0x02
    memcpy(velocity_north_byte, &velocity_north_direction, sizeof(float));
    memcpy(velocity_east_byte, &velocity_east_direction, sizeof(float));
    memcpy(velocity_down_byte, &velocity_down_direction, sizeof(float));

    // #define COMPASS_MESAGE 0x03
    //  int compass_azimuth_val = 128;

    // #define IMU_MESAGE 0x04
    memcpy(imu_yaw_byte, &imu_yaw_val, sizeof(float));
    memcpy(imu_pitch_byte, &imu_pitch_val, sizeof(float));
    memcpy(imu_rool_byte, &imu_rool_val, sizeof(float));

    // #define BATTERY_MESAGE 0x05
    memcpy(batt_voltage_byte, &batt_voltage, sizeof(float));
    memcpy(batt_current_byte, &batt_current, sizeof(float));
    memcpy(batt_charge_byte, &batt_charge, sizeof(float));
    memcpy(batt_remaining_byte, &batt_remaining, sizeof(float));

    memcpy(connection_status_byte, &connection_status_val, sizeof(int));

    memcpy(flight_mode_byte, &flight_mode_val, sizeof(int));

    memcpy(ready_to_fly_byte, &ready_to_fly_val, sizeof(int));

    memcpy(compass_azimuth_byte, &compass_azimuth_val, sizeof(int));

    // Set to buf
    packet.data[0] = 0x55; // STX  starting mark Low byte in the front
    packet.data[1] = 0x66; // STX  starting mark Low byte in the front
    //////////////////////////////////////////////////////////////////////////
    packet.data[2] = gps_lon_byte[0];
    packet.data[3] = gps_lon_byte[1];
    packet.data[4] = gps_lon_byte[2];
    packet.data[5] = gps_lon_byte[3];
    packet.data[6] = gps_lon_byte[4];
    packet.data[7] = gps_lon_byte[5];
    packet.data[8] = gps_lon_byte[6];
    packet.data[9] = gps_lon_byte[7];

    packet.data[10] = gps_lat_byte[0];
    packet.data[11] = gps_lat_byte[1];
    packet.data[12] = gps_lat_byte[2];
    packet.data[13] = gps_lat_byte[3];
    packet.data[14] = gps_lat_byte[4];
    packet.data[15] = gps_lat_byte[5];
    packet.data[16] = gps_lat_byte[6];
    packet.data[17] = gps_lat_byte[7];

    packet.data[18] = gps_alt_byte[0];
    packet.data[19] = gps_alt_byte[1];
    packet.data[20] = gps_alt_byte[2];
    packet.data[21] = gps_alt_byte[3];

    packet.data[22] = gps_hdop_byte[0];
    packet.data[23] = gps_hdop_byte[1];
    packet.data[24] = gps_hdop_byte[2];
    packet.data[25] = gps_hdop_byte[3];

    packet.data[26] = gps_vdop_byte[0];
    packet.data[27] = gps_vdop_byte[1];
    packet.data[28] = gps_vdop_byte[2];
    packet.data[29] = gps_vdop_byte[3];

    packet.data[30] = gps_num_satellites_byte[0];
    packet.data[31] = gps_num_satellites_byte[1];
    packet.data[32] = gps_num_satellites_byte[2];
    packet.data[33] = gps_num_satellites_byte[3];
    //////////////////////////////////////////////////////////////////////////
    packet.data[34] = velocity_north_byte[0];
    packet.data[35] = velocity_north_byte[1];
    packet.data[36] = velocity_north_byte[2];
    packet.data[37] = velocity_north_byte[3];

    packet.data[38] = velocity_east_byte[0];
    packet.data[39] = velocity_east_byte[1];
    packet.data[40] = velocity_east_byte[2];
    packet.data[41] = velocity_east_byte[3];

    packet.data[42] = velocity_down_byte[0];
    packet.data[43] = velocity_down_byte[1];
    packet.data[44] = velocity_down_byte[2];
    packet.data[45] = velocity_down_byte[3];
    //////////////////////////////////////////////////////////////////////////
    packet.data[46] = imu_yaw_byte[0];
    packet.data[47] = imu_yaw_byte[1];
    packet.data[48] = imu_yaw_byte[2];
    packet.data[49] = imu_yaw_byte[3];

    packet.data[50] = imu_pitch_byte[0];
    packet.data[51] = imu_pitch_byte[1];
    packet.data[52] = imu_pitch_byte[2];
    packet.data[53] = imu_pitch_byte[3];

    packet.data[54] = imu_rool_byte[0];
    packet.data[55] = imu_rool_byte[1];
    packet.data[56] = imu_rool_byte[2];
    packet.data[57] = imu_rool_byte[3];
    //////////////////////////////////////////////////////////////////////////
    packet.data[58] = batt_voltage_byte[0];
    packet.data[59] = batt_voltage_byte[1];
    packet.data[60] = batt_voltage_byte[2];
    packet.data[61] = batt_voltage_byte[3];

    packet.data[62] = batt_current_byte[0];
    packet.data[63] = batt_current_byte[1];
    packet.data[64] = batt_current_byte[2];
    packet.data[65] = batt_current_byte[3];

    packet.data[66] = batt_charge_byte[0];
    packet.data[67] = batt_charge_byte[1];
    packet.data[68] = batt_charge_byte[2];
    packet.data[69] = batt_charge_byte[3];

    packet.data[70] = batt_remaining_byte[0];
    packet.data[71] = batt_remaining_byte[1];
    packet.data[72] = batt_remaining_byte[2];
    packet.data[73] = batt_remaining_byte[3];

    packet.data[74] = connection_status_byte[0];
    packet.data[75] = connection_status_byte[1];
    packet.data[76] = connection_status_byte[2];
    packet.data[77] = connection_status_byte[3];

    packet.data[78] = flight_mode_byte[0];
    packet.data[79] = flight_mode_byte[1];
    packet.data[80] = flight_mode_byte[2];
    packet.data[81] = flight_mode_byte[3];

    packet.data[82] = ready_to_fly_byte[0];
    packet.data[83] = ready_to_fly_byte[1];
    packet.data[84] = ready_to_fly_byte[2];
    packet.data[85] = ready_to_fly_byte[3];

    packet.data[86] = compass_azimuth_byte[0];
    packet.data[87] = compass_azimuth_byte[1];
    packet.data[88] = compass_azimuth_byte[2];
    packet.data[89] = compass_azimuth_byte[3];

    //////////////////////////////////////////////////////////////////////////
    packet.data[90] = 0x03;
    packet.data[91] = 0x07;

    // Calc to CRC16
    packet.crc16 = CRC16_cal(packet.data, PACKET_SIZE - 2, *crc16_tab);

    // Set CRC to buf
    packet.data[PACKET_SIZE - 2] = (char)(packet.crc16 & 0xFF);
    packet.data[PACKET_SIZE - 1] = (char)((packet.crc16 >> 8) & 0xFF);

    switch (flag_read_waypoint)
    {
    case 1:
    {
      // todo research::should we use async?
      auto waypoints = mission_raw->download_mission();
      std::cout << "Downloading mission..." << std::endl;

      auto result_download = waypoints.first;

      if (result_download != MissionRaw::Result::Success)
      {
        std::cout << "Mission does not download."
                  << " " << result_download << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::seconds(10));

#if defined(DEBUG)
      for (auto wp : waypoints.second)
      {
        std::cout << wp.seq << " " << wp.x << " " << wp.y << std::endl;
      }
#endif

      readWaypoints(waypoints.second, getDestDirPath() + MISSION_WP_FILENAME);

      break;
    }
    default:
      break;
    }

    switch (flag_write_waypoint)
    {
    case 1:
    {
      std::cout << "Writing waypoints..." << std::endl;
      auto waypoints = writeWaypoints(getDestDirPath() + MISSION_WP_FILENAME);

      std::cout << sizeof(waypoints);
      std::this_thread::sleep_for(std::chrono::seconds(10));

      auto result_upload = mission_raw->upload_mission(waypoints);

      if (result_upload != MissionRaw::Result::Success)
      {
        std::cout << "Mission does not upload."
                  << " " << result_upload << std::endl;
      }
      else
      {
        std::cout << "Mission uploaded." << std::endl;
      }

      break;
    }
    default:
      break;
    }

    switch (flag_read_param)
    {
    case 1:
    {
      std::cout << "Reading parameters..." << std::endl;
      auto all_parameters = param->get_all_params();
      readParams(getDestDirPath() + "file.parm", all_parameters);
      break;
    }
    default:
      break;
    }

    switch (flag_write_param)
    {
    case 1:
    {
      auto parameters = writeParams(getDestDirPath() + "mav.parm");
      std::cout << "Parameters succesfully written to the file!" << std::endl;

      for (auto param_int : parameters.int_params)
      {
        param->set_param_int(param_int.name, param_int.value);
      }
      for (auto param_float : parameters.float_params)
      {
        param->set_param_float(param_float.name, param_float.value);
      }
    }
    default:
      break;
    }

    // Send
    if (sendto(cliSockDes, packet.data, sizeof(packet.data), 0,
               (struct sockaddr *)&serAddr, sizeof(serAddr)) < 0)
    {
      perror("sending error...\n");
      // close(cliSockDes);
      // exit(-1);
    }
    // else
    // {
    //   std::cout << "Sending data..." << std::endl;
    // }

    // Read
    serAddrLen = sizeof(serAddr);
    readStatus = recvfrom(cliSockDes, packetRX.data, sizeof(packetRX.data), 0,
                          (struct sockaddr *)&serAddr, &serAddrLen);

    if (readStatus < 0)
    {
      perror("reading error...\n");
      // close(cliSockDes);
      // exit(-1);
    }

    header[0] = packetRX.data[0]; // STX  starting mark Low byte in the front
    header[1] = packetRX.data[1]; // STX  starting mark Low byte in the front

    packetRX.data[2];
    packetRX.data[3];

    packetRX.data[4];
    packetRX.data[5];

    packetRX.data[6];
    packetRX.data[7];

    packetRX.data[8];
    packetRX.data[9];

    // Set CRC to buf
    crc_buf[0] = packet.data[10];
    crc_buf[1] = packet.data[11];

    uint8_t status = crc_check_16bites(crc_buf, sizeof(crc_buf), &result);

    if (status == 2)
    {

#if defined(DEBUG)
      std::cout << "CRC OK...  " << result;
#endif
    }
    else
    {
      perror("CRC error...\n");
    }

#if defined(DEBUG)

    std::cout << "   Read_waypoint " << flag_read_waypoint;
    std::cout << "   Write_waypoint " << flag_write_waypoint;
    std::cout << "   Read_param " << flag_read_param;
    std::cout << "   Write_param " << flag_write_param << std::endl;

#endif

    if (packetRX.data[6] == 0x67)
    {
      flag_read_waypoint = 1;
    }
    else
    {
      flag_read_waypoint = 0;
    }

    if (packetRX.data[7] == 0x57)
    {
      flag_write_waypoint = 1;
    }
    else
    {
      flag_write_waypoint = 0;
    }

    if (packetRX.data[8] == 0x47)
    {
      flag_read_param = 1;
    }
    else
    {
      flag_read_param = 0;
    }

    if (packetRX.data[9] == 0x37)
    {
      flag_write_param = 1;
    }
    else
    {
      flag_write_param = 0;
    }
  }

  close(cliSockDes);

  return 0;
}