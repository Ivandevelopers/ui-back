#include "autopilot_mission.h"
#include "autopilot_params.h"

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
#include <mavsdk/plugins/action/action.h>

#include "timer.cpp"

// dev
#include <iomanip>

using namespace mavsdk;

// #define DEBUG

#define PORT 10000

#define CONNECTION_PORT "udp://:14552" // 14552

#define PACKET_SIZE 152

#define PACKET_SIZE_RX 60

#define WRITE_PARAM_INTERVAL_MS 50
#define WRITE_PARAM_NUM_RETRIES 3

// dev
#define DIR_NAME "/data/"
#define MISSION_WP_FILENAME "mission.plan"
#define PARAM_FILENAME "autopilot.param"
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

/************************************** WAYPOINTS FLAGS */
int flag_read_waypoint = 0;
int flag_write_waypoint = 0;

std::atomic<bool> flag_read_waypoint_ready = false;
std::atomic<bool> flag_write_waypoint_ready = false;

unsigned char flag_read_waypoint_ready_byte[sizeof(flag_read_waypoint_ready)];
unsigned char flag_write_waypoint_ready_byte[sizeof(flag_write_waypoint_ready)];

/************************************** PARAMS FLAGS */
int flag_read_param = 0;
int flag_write_param = 0;

bool flag_read_param_ready = false;
bool flag_write_param_ready = 0;

unsigned char flag_read_param_ready_byte[sizeof(flag_read_param_ready)];
unsigned char flag_write_param_ready_byte[sizeof(flag_write_param_ready)];

/******************************** WRITE ONE PARAM */
bool flag_write_one_param = false;

char param_name[16] = {0};
float param_value = 0.0;
unsigned char param_value_byte[sizeof(param_name)];
// TX
bool flag_write_one_param_ready = false;

u_int16_t param_failed_count = 0;
char param_failed_name[16] = {0};

unsigned char param_failed_count_byte[sizeof(param_failed_count)];
unsigned char param_failed_name_byte[sizeof(param_failed_name)];

uint32_t result = 0;

uint8_t crc_buf[] = {0};

char header[] = {0};

#define GPS_MESSAGE 0x01
double gps_lon_val;
double gps_lat_val;
float gps_alt_amsl_val;
float gps_alt_rel_val;

float gps_hdop_val;
float gps_vdop_val;
int gps_num_satellites_val;

unsigned char gps_lon_byte[sizeof(gps_lon_val)];
unsigned char gps_lat_byte[sizeof(gps_lat_val)];
unsigned char gps_alt_amsl_byte[sizeof(gps_alt_amsl_val)];
unsigned char gps_alt_rel_byte[sizeof(gps_alt_rel_val)];

unsigned char gps_hdop_byte[sizeof(gps_hdop_val)];
unsigned char gps_vdop_byte[sizeof(gps_vdop_val)];
unsigned char gps_num_satellites_byte[sizeof(gps_num_satellites_val)];

#define VELOCITY_MESSAGE 0x02
float velocity_north_direction;
float velocity_east_direction;
float velocity_down_direction;

unsigned char velocity_north_byte[sizeof(velocity_north_direction)];
unsigned char velocity_east_byte[sizeof(velocity_east_direction)];
unsigned char velocity_down_byte[sizeof(velocity_down_direction)];

#define IMU_MESSAGE 0x03
float imu_yaw_val;
float imu_pitch_val;
float imu_roll_val;

unsigned char imu_yaw_byte[sizeof(imu_yaw_val)];
unsigned char imu_pitch_byte[sizeof(imu_pitch_val)];
unsigned char imu_roll_byte[sizeof(imu_roll_val)];

#define BATTERY_MESSAGE 0x04
float batt_voltage;
float batt_current;
float batt_charge;
float batt_remaining;

unsigned char batt_voltage_byte[sizeof(batt_voltage)];
unsigned char batt_current_byte[sizeof(batt_current)];
unsigned char batt_charge_byte[sizeof(batt_charge)];
unsigned char batt_remaining_byte[sizeof(batt_remaining)];

#define CONNECTION_STATUS 0x05
int connection_status_val = 0;

unsigned char connection_status_byte[sizeof(connection_status_val)];

#define FLIGHT_MODE 0x06
int flight_mode_val;

unsigned char flight_mode_byte[sizeof(flight_mode_val)];

#define READY_TO_FLY 0x07
bool ready_to_fly_val;

unsigned char ready_to_fly_byte[sizeof(ready_to_fly_val)];

#define COMPASS_MESSAGE 0x08
int16_t compass_azimuth_val;

unsigned char compass_azimuth_byte[sizeof(compass_azimuth_val)];

#define RADIO_STATUS 0x09
uint8_t radio_status_rssi_val;
uint8_t radio_status_remrssi_val;
uint8_t radio_status_txbuf_val;
uint8_t radio_status_noise_val;
uint8_t radio_status_remnoise_val;
uint16_t radio_status_rxerrors_val;
uint16_t radio_status_fixed_val;

unsigned char radio_status_rssi_byte[sizeof(radio_status_rssi_val)];
unsigned char radio_status_remrssi_byte[sizeof(radio_status_remrssi_val)];
unsigned char radio_status_txbuf_byte[sizeof(radio_status_txbuf_val)];
unsigned char radio_status_noise_byte[sizeof(radio_status_noise_val)];
unsigned char radio_status_remnoise_byte[sizeof(radio_status_remnoise_val)];
unsigned char radio_status_rxerrors_byte[sizeof(radio_status_rxerrors_val)];
unsigned char radio_status_fixed_byte[sizeof(radio_status_fixed_val)];

/******************************* ARMED STATUS *********************************************************/
bool armed_status_val = 0;
unsigned char armed_status_byte[sizeof(armed_status_val)];

/******************************* HOME POSITION *********************************************************/
#define HOME_POSITION 0x09
double home_latitude_val;
double home_longitude_val;
float home_altitude_val;

unsigned char home_latitude_byte[sizeof(home_latitude_val)];
unsigned char home_longitude_byte[sizeof(home_longitude_val)];
unsigned char home_altitude_byte[sizeof(home_altitude_val)];


int flag_read_home_position = 0;
int flag_write_home_position = 0;

int flag_read_home_position_ready = 0;

unsigned char flag_read_home_position_ready_byte[sizeof(flag_read_home_position_ready)];

/******************************* HOME POSITION TO WRITE *********************************************************/
double home_latitude_val_rcv;
double home_longitude_val_rcv;
float home_altitude_val_rcv;

unsigned char home_latitude_byte_rcv[sizeof(home_latitude_val_rcv)];
unsigned char home_longitude_byte_rcv[sizeof(home_longitude_val_rcv)];
unsigned char home_altitude_byte_rcv[sizeof(home_altitude_val_rcv)];

// RX flag
int flag_write_new_home_position = 0;
// TX flag
int flag_write_home_position_ready = 0;

unsigned char flag_write_new_home_position_byte[sizeof(flag_write_new_home_position)];
unsigned char flag_write_home_position_ready_byte[sizeof(flag_write_home_position_ready)];

/******************************** ACTION TYPES */
uint8_t flag_action_type = 0;
#define TAKE_OFF 0x01;

unsigned char flag_action_type_byte[sizeof(flag_action_type)];
/******************************** TAKE OFF ALTITUDA */
uint16_t altitude_takeoff_val = 0;
unsigned char altitude_takeoff_byte[sizeof(altitude_takeoff_val)];

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


struct timeval tv;

static void mavlink_message_callback(const mavlink_message_t &msg);

static bool writeHomePosition(MavlinkPassthrough &mavlink_passthrough);

static void do_takeoff(Action &action, int altitute);

// reading params
bool all_params_read = false;
bool in_reading_state = false;
// write params
bool all_params_sent = false;
bool all_params_write_result_ready = false;
bool responding_param_failed = false;


std::vector<AutopilotParam> params_list;
std::vector<AutopilotParam> write_params_list_result;
std::vector<AutopilotParam> write_params_fails_list;
std::vector<int> write_params_missing_index_list;

std::vector<int> read_param_missing_index_list;


int current_index_to_write = 0;
int write_param_retries = 0;

static bool request_param_by_index(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, int index);

static MavlinkPassthrough::MessageHandle subscribe_param_list(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, ThreadTimer &timer);

static bool write_single_param(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, MavlinkPassthrough::MessageHandle &write_single_param_handle, char param_name[16], float param_value);

static void write_param_list(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, ThreadTimer &timer);

static void subscribe_write_param_list_result(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, 
                                      MavlinkPassthrough::MessageHandle &write_param_list_handle,
                                      ThreadTimer &_params_write_result_timer
                                      );

static void write_missing_params(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, ThreadTimer &_params_write_timer);

static void get_params_failed(std::vector<AutopilotParam> &params_list, 
                      std::vector<AutopilotParam> &_write_params_list_result, 
                      std::vector<AutopilotParam> &write_params_fails_list,
                      std::vector<int> &_write_params_missing_index_list
                      );

static void merge_failed_params(std::vector<AutopilotParam> &params_list,
                        std::vector<AutopilotParam> &write_params_fails_list,
                        std::vector<int> &_write_params_missing_index_list
                        );

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

  tv.tv_sec = 3;
  tv.tv_usec = 0;
  setsockopt(cliSockDes, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));

  Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};

  auto connection_result = mavsdk.add_any_connection(CONNECTION_PORT);
  if (connection_result == mavsdk::ConnectionResult::Success) {
    std::cout << "Connected!" << std::endl;
  }

  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();
  Mavsdk::NewSystemHandle handle =
      mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().back();

        std::cout << "........DEBUG SYSTEM_MSG.....NEW SYSTEM........" << std::endl;
        connection_status_val = 1;
        if (system->has_autopilot()) {
          std::cout << "........AUTOPILOT ADDED........" << std::endl;

          mavsdk.unsubscribe_on_new_system(handle);
          prom.set_value(system);
        } else {
          std::cout << "No MAVSDK found." << std::endl;
        }
      });

  fut.wait();

  auto system = fut.get();

  system->subscribe_is_connected([](bool is_connected) {
      connection_status_val = is_connected;
  });

  auto param = std::make_shared<Param>(system);
  auto telemetry = std::make_shared<Telemetry>(system);
  auto mission_raw = std::make_shared<MissionRaw>(system);
  auto mavlink_passthrough = std::make_shared<MavlinkPassthrough>(system);
  //
  auto action = std::make_shared<Action>(system);
  

  // heading
  mavlink_passthrough->subscribe_message(MAVLINK_MSG_ID_VFR_HUD, mavlink_message_callback);

  // rssi
  mavlink_passthrough->subscribe_message(MAVLINK_MSG_ID_RADIO_STATUS, mavlink_message_callback);

  // roll, pitch & yaw
  mavlink_passthrough->subscribe_message(MAVLINK_MSG_ID_ATTITUDE, mavlink_message_callback);

  telemetry->subscribe_rc_status([](Telemetry::RcStatus rc_status)
    {
      radio_status_rssi_val = rc_status.signal_strength_percent;
    });

  telemetry->subscribe_armed([](bool armed) {
    // if (DEBUG) {
    //   std::cout << "Armed: " << armed << std::endl; 
    // }
    armed_status_val = armed;
  });
  //---------------------------------


  telemetry->subscribe_health([](Telemetry::Health health) {
#if defined(DEBUG)
    std::cout << "________ is_accelerometer_calibration_ok: " << health.is_accelerometer_calibration_ok <<
                 "\n_______ is_armable: " << health.is_armable <<
                 "\n_______ is_global_position_ok: " << health.is_global_position_ok <<
                  "\n_______ is_gyrometer_calibration_ok: " << health.is_gyrometer_calibration_ok <<
                  "\n_______ is_home_position_ok: " << health.is_home_position_ok <<
                  "\n_______ is_local_position_ok: " << health.is_local_position_ok <<
                  "\n_______ is_magnetometer_calibration_ok: " << health.is_magnetometer_calibration_ok << "\n" << std::endl;
#endif
    ready_to_fly_val = health.is_armable;
  });

  // altitude & gps (long, lat)
  telemetry->subscribe_position([](Telemetry::Position position)
                                {
#if defined(DEBUG)
                                  std::cout << "Altitude & GPS (long, lat)" << std::endl;
                                  std::cout << "Altitude AMSL (above mean sea level) in metres: " << position.absolute_altitude_m
                                            << " Altitude relative to takeoff altitude in metres: " << position.relative_altitude_m
                                            << " Latitude: " << position.latitude_deg
                                            << " Longitude: " << position.longitude_deg << std::endl;
                                  std::cout << " alt asl" << gps_alt_amsl_val << " alt rel" << gps_alt_rel_val << std::endl;
#endif

                                  gps_alt_amsl_val = position.absolute_altitude_m;
                                  gps_alt_rel_val = position.relative_altitude_m;
                                  gps_lon_val = position.longitude_deg;
                                  gps_lat_val = position.latitude_deg; });

  // gps hdop/vdop
  telemetry->subscribe_raw_gps([](Telemetry::RawGps raw_gps)
                               {
#if defined(DEBUG)
                                 std::cout << "Gps hdop: " << raw_gps.hdop
                                           << "  gps vdop: " << raw_gps.vdop << std::endl;
#endif

                                 gps_hdop_val = raw_gps.hdop;
                                 gps_vdop_val = raw_gps.vdop; });

  // gps number of satellites:
  telemetry->subscribe_gps_info([](Telemetry::GpsInfo gps_info)
                                {
#if defined(DEBUG)
    std::cout << "Gps number of satellites: " << gps_info.num_satellites << std::endl;
#endif

    gps_num_satellites_val = gps_info.num_satellites; });

  // home position
  Telemetry::HomeHandle home_handle = telemetry->subscribe_home([](Telemetry::Position home_position)
                              {
#if defined(DEBUG)
        std::cout << "NEW HOME POS" << std::endl;
        std::cout << "Home lat:" << home_position.latitude_deg << std::endl;
        std::cout << "Home long: " << home_position.longitude_deg << std::endl;
        std::cout << "Home alt: " << home_position.absolute_altitude_m << std::endl;
#endif
        home_latitude_val = home_position.latitude_deg;
        home_longitude_val = home_position.longitude_deg;
        home_altitude_val = home_position.absolute_altitude_m;

        // flag_read_home_position_ready.store(true, std::memory_order_release);
        flag_read_home_position_ready = 1;
    });

  // flight mode
  telemetry->subscribe_flight_mode([](Telemetry::FlightMode flight_mode)
                                   {
#if defined(DEBUG)
                                     std::cout << "Current flight mode: " << flight_mode << std::endl;
#endif

                                     switch (flight_mode)
                                     {
                                     case Telemetry::FlightMode::Acro:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Altctl:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::FollowMe:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Hold:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Land:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Manual:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Mission:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Offboard:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Posctl:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Rattitude:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Ready:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::ReturnToLaunch:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Stabilized:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Takeoff:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;

                                     case Telemetry::FlightMode::Unknown:
                                       flight_mode_val = static_cast<int>(flight_mode);
                                       break;
                                     } });

  // velocity
  telemetry->subscribe_velocity_ned([](Telemetry::VelocityNed vel_ned)
                                    {

#if defined(DEBUG)
    
    std::cout << "Velocity" << std::endl;
    std::cout << "Velocity along north direction in metres per second: "
              << vel_ned.north_m_s << " "
              << "velocity along east direction in metres per second: "
              << vel_ned.east_m_s << " "
              << "velocity along down direction in metres per second: "
              << vel_ned.down_m_s << std::endl;

#endif

    velocity_north_direction = vel_ned.north_m_s;
    velocity_east_direction = vel_ned.east_m_s;
    velocity_down_direction = vel_ned.down_m_s; });

  // battery
  telemetry->subscribe_battery([](Telemetry::Battery battery)
                               {
#if defined(DEBUG)
    std::cout << "Battery" << std::endl;
    std::cout << "Voltage: " << battery.voltage_v << std::endl
              << "Battery current: " << battery.current_battery_a << std::endl
              << "Consumed charge: " << battery.capacity_consumed_ah
              << std::endl
              << "Estimated battery remaining: " << battery.remaining_percent
              << '\n';
#endif

    batt_voltage = battery.voltage_v;
    batt_current = battery.current_battery_a;
    batt_charge = battery.capacity_consumed_ah;
    batt_remaining = battery.remaining_percent; });   

  MavlinkPassthrough::MessageHandle read_param_list_handle;
  MavlinkPassthrough::MessageHandle write_single_param_handle;
  MavlinkPassthrough::MessageHandle write_param_list_handle;

  ThreadTimer params_read_timer;
  ThreadTimer params_write_timer;
  ThreadTimer params_write_result_timer;

  while (1)
  {

    switch (flag_read_waypoint) {
      case 1: {
        mission_raw->download_mission_async([&](MissionRaw::Result result, std::vector<MissionRaw::MissionItem> items)
                                            {
          if (result == MissionRaw::Result::Success) {
              std::cout << "Mission download successful: " << std::endl;
              readWaypointsFromControllerToFile(items, getDestDirPath() + MISSION_WP_FILENAME);

              flag_read_waypoint_ready.store(true, std::memory_order_release);
          } else {
            std::cout << "Error reading waypoints" << std::endl;
          } });
      }
    }

    switch (flag_write_waypoint) {
      case 1: {
        std::cout << "Writing waypoints..." << std::endl;

        auto destDir = getDestDirPath() + MISSION_WP_FILENAME;
        auto waypoints = writeWaypointsFromFileToController(destDir);

        mission_raw->upload_mission_async(waypoints, [&](MissionRaw::Result result)
                                          {

            if (result != MissionRaw::Result::Success) {
                std::cout << "Mission upload failed. Error code: " << result << std::endl;
            } else {
                std::cout << "Mission uploaded successfully." << std::endl;
                
                flag_write_waypoint_ready.store(true, std::memory_order_release);
            } });

        break;
      }
    }

    switch (flag_read_param) {
      case 1: {
        in_reading_state = true;
        all_params_read = false;
        //
        if (params_list.size() > 0) {
          params_list.clear();
        }
        mavlink_passthrough->unsubscribe_message(MAVLINK_MSG_ID_PARAM_VALUE, read_param_list_handle);

        read_param_list_handle = subscribe_param_list(mavlink_passthrough, params_read_timer);

        MavlinkPassthrough::Result params_request_result = mavlink_passthrough->queue_message(
            [&](MavlinkAddress mavlink_address, uint8_t channel) {

            mavlink_message_t message;
            mavlink_msg_param_request_list_pack(
                mavlink_passthrough->get_our_sysid(),
                mavlink_passthrough->get_our_compid(),
                &message,
                mavlink_passthrough->get_target_sysid(),
                mavlink_passthrough->get_target_compid()
              );
            return message; 
          });

        if (params_request_result == MavlinkPassthrough::Result::Success) {
          flag_read_param = 0;
        }
        break;
      }
      default:
        break;
    }

    switch (flag_write_param)
    {
    case 1:
    {
      std::cout << "...DEBUG FLAG...Writing params..." << std::endl;

      mavlink_passthrough->unsubscribe_message(MAVLINK_MSG_ID_PARAM_VALUE, write_param_list_handle);

      all_params_sent = false;  
      all_params_write_result_ready = false;    

      current_index_to_write = 0;

      params_list.clear();
      write_params_list_result.clear();

      write_param_retries = WRITE_PARAM_NUM_RETRIES;

      subscribe_write_param_list_result(mavlink_passthrough, write_param_list_handle, params_write_result_timer);

      write_param_list(mavlink_passthrough, params_write_timer);
    }
    default:
      break;
    }

    if (all_params_sent && all_params_write_result_ready) {
      std::cout << "......DEBUG...ALL PARAM SENT AND RECEIEVED" << std::endl;

      get_params_failed(params_list, write_params_list_result, write_params_fails_list, write_params_missing_index_list);

      all_params_sent = false;
      all_params_write_result_ready = false;

      if (write_params_missing_index_list.size() > 0 && write_param_retries > 0) {
        subscribe_write_param_list_result(mavlink_passthrough, write_param_list_handle, params_write_result_timer);
        write_missing_params(mavlink_passthrough, params_write_timer);
        
        write_param_retries--;
      } else {

        if (write_params_missing_index_list.size() > 0) {
          merge_failed_params(params_list, write_params_fails_list, write_params_missing_index_list);
        }

        write_params_list_result.clear();
        params_list.clear();
        if (write_params_fails_list.size() > 0) {
          responding_param_failed = true;

          param_failed_count = write_params_fails_list.size();
        }

        flag_write_param_ready = true;
      }

    }

    if (!responding_param_failed) {
      memset(param_failed_count_byte, 0, sizeof(param_failed_count));
      memset(param_failed_name, 0, sizeof(param_failed_name));
    }

    if (responding_param_failed && write_params_fails_list.size() > 0) {
      std::cout << "......DEBUG...PROCCESS FAILES" << std::endl;

      memcpy(param_failed_count_byte, &param_failed_count, sizeof(param_failed_count));

      AutopilotParam _param = write_params_fails_list.back();
      memcpy(param_failed_name_byte, _param.get_name().c_str(), _param.get_name().length());
      if (_param.get_name().length() < 16) {
        param_failed_name_byte[_param.get_name().length()] = '\0';
      }

      write_params_fails_list.pop_back();

      if (write_params_fails_list.size() == 0) {
        responding_param_failed = false;
      }
    }


    // Convert float to byte

    // #define GPS_MESSAGE 0x01
    memcpy(gps_lon_byte, &gps_lon_val, sizeof(double));
    memcpy(gps_lat_byte, &gps_lat_val, sizeof(double));
    memcpy(gps_alt_amsl_byte, &gps_alt_amsl_val, sizeof(float));
    memcpy(gps_alt_rel_byte, &gps_alt_rel_val, sizeof(float));

    memcpy(gps_hdop_byte, &gps_hdop_val, sizeof(float));
    memcpy(gps_vdop_byte, &gps_vdop_val, sizeof(float));
    memcpy(gps_num_satellites_byte, &gps_num_satellites_val, sizeof(int));

    // #define VELOCITY_MESSAGE 0x02
    memcpy(velocity_north_byte, &velocity_north_direction, sizeof(float));
    memcpy(velocity_east_byte, &velocity_east_direction, sizeof(float));
    memcpy(velocity_down_byte, &velocity_down_direction, sizeof(float));

    // #define IMU_MESAGE 0x03
    memcpy(imu_yaw_byte, &imu_yaw_val, sizeof(float));
    memcpy(imu_pitch_byte, &imu_pitch_val, sizeof(float));
    memcpy(imu_roll_byte, &imu_roll_val, sizeof(float));

    // #define BATTERY_MESSAGE 0x04
    memcpy(batt_voltage_byte, &batt_voltage, sizeof(float));
    memcpy(batt_current_byte, &batt_current, sizeof(float));
    memcpy(batt_charge_byte, &batt_charge, sizeof(float));
    memcpy(batt_remaining_byte, &batt_remaining, sizeof(float));

    // #define CONNECTION_STATUS 0x05
    memcpy(connection_status_byte, &connection_status_val, sizeof(int));

    // #define FLIGHT_MODE 0x06
    memcpy(flight_mode_byte, &flight_mode_val, sizeof(int));

    // #define READY_TO_FLY 0x07
    memcpy(ready_to_fly_byte, &ready_to_fly_val, sizeof(bool));

    // #define COMPASS_MESSAGE 0x08
    memcpy(compass_azimuth_byte, &compass_azimuth_val, sizeof(int16_t));

    // #define RADIO_STATUS 0x09
    memcpy(radio_status_rssi_byte, &radio_status_rssi_val, sizeof(uint8_t));
    memcpy(radio_status_remrssi_byte, &radio_status_remrssi_val, sizeof(uint8_t));
    memcpy(radio_status_txbuf_byte, &radio_status_txbuf_val, sizeof(uint8_t));
    memcpy(radio_status_noise_byte, &radio_status_noise_val, sizeof(uint8_t));
    memcpy(radio_status_remnoise_byte, &radio_status_noise_val, sizeof(uint8_t));
    memcpy(radio_status_rxerrors_byte, &radio_status_rxerrors_val, sizeof(uint16_t));
    memcpy(radio_status_fixed_byte, &radio_status_fixed_val, sizeof(uint16_t));

    bool _flag_read_waypoint_ready = flag_read_waypoint_ready.load(std::memory_order_acquire);
    memcpy(flag_read_waypoint_ready_byte, &_flag_read_waypoint_ready, sizeof(bool));

    bool _flag_write_waypoint_ready = flag_write_waypoint_ready.load(std::memory_order_acquire);
    memcpy(flag_write_waypoint_ready_byte, &_flag_write_waypoint_ready, sizeof(bool));

    bool _flag_read_param_ready = flag_read_param_ready;
    memcpy(flag_read_param_ready_byte, &_flag_read_param_ready, sizeof(bool));

    bool _flag_write_param_ready = flag_write_param_ready;
    memcpy(flag_write_param_ready_byte, &_flag_write_param_ready, sizeof(bool));

    memcpy(home_latitude_byte, &home_latitude_val, sizeof(double));
    memcpy(home_longitude_byte, &home_longitude_val, sizeof(double));
    memcpy(home_altitude_byte, &home_altitude_val, sizeof(float));

    // home position read result
    memcpy(flag_read_home_position_ready_byte, &flag_read_home_position_ready, sizeof(int));
    // home position write result
    memcpy(flag_write_home_position_ready_byte, &flag_write_home_position_ready, sizeof(int));

    /**************************************** ARMED STATUS */
    memcpy(armed_status_byte, &armed_status_val, sizeof(bool));

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

    packet.data[18] = gps_alt_amsl_byte[0];
    packet.data[19] = gps_alt_amsl_byte[1];
    packet.data[20] = gps_alt_amsl_byte[2];
    packet.data[21] = gps_alt_amsl_byte[3];

    packet.data[22] = gps_alt_rel_byte[0];
    packet.data[23] = gps_alt_rel_byte[1];
    packet.data[24] = gps_alt_rel_byte[2];
    packet.data[25] = gps_alt_rel_byte[3];

    packet.data[26] = gps_hdop_byte[0];
    packet.data[27] = gps_hdop_byte[1];
    packet.data[28] = gps_hdop_byte[2];
    packet.data[29] = gps_hdop_byte[3];

    packet.data[30] = gps_vdop_byte[0];
    packet.data[31] = gps_vdop_byte[1];
    packet.data[32] = gps_vdop_byte[2];
    packet.data[33] = gps_vdop_byte[3];

    packet.data[34] = gps_num_satellites_byte[0];
    packet.data[35] = gps_num_satellites_byte[1];
    packet.data[36] = gps_num_satellites_byte[2];
    packet.data[37] = gps_num_satellites_byte[3];
    //////////////////////////////////////////////////////////////////////////
    packet.data[38] = velocity_north_byte[0];
    packet.data[39] = velocity_north_byte[1];
    packet.data[40] = velocity_north_byte[2];
    packet.data[41] = velocity_north_byte[3];

    packet.data[42] = velocity_east_byte[0];
    packet.data[43] = velocity_east_byte[1];
    packet.data[44] = velocity_east_byte[2];
    packet.data[45] = velocity_east_byte[3];

    packet.data[46] = velocity_down_byte[0];
    packet.data[47] = velocity_down_byte[1];
    packet.data[48] = velocity_down_byte[2];
    packet.data[49] = velocity_down_byte[3];
    //////////////////////////////////////////////////////////////////////////
    packet.data[50] = imu_yaw_byte[0];
    packet.data[51] = imu_yaw_byte[1];
    packet.data[52] = imu_yaw_byte[2];
    packet.data[53] = imu_yaw_byte[3];

    packet.data[54] = imu_pitch_byte[0];
    packet.data[55] = imu_pitch_byte[1];
    packet.data[56] = imu_pitch_byte[2];
    packet.data[57] = imu_pitch_byte[3];

    packet.data[58] = imu_roll_byte[0];
    packet.data[59] = imu_roll_byte[1];
    packet.data[60] = imu_roll_byte[2];
    packet.data[61] = imu_roll_byte[3];
    
    /************************ BATTERY STATUS */
    packet.data[62] = batt_voltage_byte[0];
    packet.data[63] = batt_voltage_byte[1];
    packet.data[64] = batt_voltage_byte[2];
    packet.data[65] = batt_voltage_byte[3];

    packet.data[66] = batt_current_byte[0];
    packet.data[67] = batt_current_byte[1];
    packet.data[68] = batt_current_byte[2];
    packet.data[69] = batt_current_byte[3];

    packet.data[70] = batt_charge_byte[0];
    packet.data[71] = batt_charge_byte[1];
    packet.data[72] = batt_charge_byte[2];
    packet.data[73] = batt_charge_byte[3];

    packet.data[74] = batt_remaining_byte[0];
    packet.data[75] = batt_remaining_byte[1];
    packet.data[76] = batt_remaining_byte[2];
    packet.data[77] = batt_remaining_byte[3];

    /************************** CONNECTION STATUS */
    packet.data[78] = connection_status_byte[0];
    packet.data[79] = connection_status_byte[1];
    packet.data[80] = connection_status_byte[2];
    packet.data[81] = connection_status_byte[3];

    /************************** FLIGHT MODE */
    packet.data[82] = flight_mode_byte[0];
    packet.data[83] = flight_mode_byte[1];
    packet.data[84] = flight_mode_byte[2];
    packet.data[85] = flight_mode_byte[3];

    /************************** READY TO FLY STATUS */
    packet.data[86] = ready_to_fly_byte[0];

    /*************************** AZIMUTH */
    packet.data[87] = compass_azimuth_byte[0];
    packet.data[88] = compass_azimuth_byte[1];

    /*************************** RADIO STATUS */
    packet.data[89] = radio_status_rssi_byte[0];
    packet.data[90] = radio_status_remrssi_byte[0];
    packet.data[91] = radio_status_txbuf_byte[0];
    packet.data[92] = radio_status_noise_byte[0];

    packet.data[93] = radio_status_remnoise_byte[0];

    packet.data[94] = radio_status_rxerrors_byte[0];
    packet.data[95] = radio_status_rxerrors_byte[1];

    packet.data[96] = radio_status_fixed_byte[0];
    packet.data[97] = radio_status_fixed_byte[1];

    /*************************** WAYPOINT */
    packet.data[98] = flag_read_waypoint_ready_byte[0];
    packet.data[99] = flag_write_waypoint_ready_byte[0];

    /*************************** Params */
    packet.data[100] = flag_read_param_ready_byte[0];
    packet.data[101] = flag_write_param_ready_byte[0];

    /*************************** HOME POSITION */
    packet.data[102] = home_latitude_byte[0];
    packet.data[103] = home_latitude_byte[1];
    packet.data[104] = home_latitude_byte[2];
    packet.data[105] = home_latitude_byte[3];
    packet.data[106] = home_latitude_byte[4];
    packet.data[107] = home_latitude_byte[5];
    packet.data[108] = home_latitude_byte[6];
    packet.data[109] = home_latitude_byte[7];

    packet.data[110] = home_longitude_byte[0];
    packet.data[111] = home_longitude_byte[1];
    packet.data[112] = home_longitude_byte[2];
    packet.data[113] = home_longitude_byte[3];
    packet.data[114] = home_longitude_byte[4];
    packet.data[115] = home_longitude_byte[5];
    packet.data[116] = home_longitude_byte[6];
    packet.data[117] = home_longitude_byte[7];

    packet.data[118] = home_altitude_byte[0];
    packet.data[119] = home_altitude_byte[1];
    packet.data[120] = home_altitude_byte[2];
    packet.data[121] = home_altitude_byte[3];

    /****************************** READ HOME POSITION - RESULT */
    packet.data[122] = flag_read_home_position_ready_byte[0];
    packet.data[123] = flag_read_home_position_ready_byte[1];
    packet.data[124] = flag_read_home_position_ready_byte[2];
    packet.data[125] = flag_read_home_position_ready_byte[3];

    /****************************** WRITE HOME POSITION - RESULT */
    packet.data[126] = flag_write_home_position_ready_byte[0];
    packet.data[127] = flag_write_home_position_ready_byte[1];
    packet.data[128] = flag_write_home_position_ready_byte[2];
    packet.data[129] = flag_write_home_position_ready_byte[3];

    /****************************** ARMED STATUS */
    packet.data[130] = armed_status_byte[0];
    /****************************** WRITE ONE PARAM */
    bool _flag_write_one_param = flag_write_one_param;
    packet.data[131] = _flag_write_one_param ? 0x01 : 0x00;

    packet.data[132] = param_failed_count_byte[0];
    packet.data[133] = param_failed_count_byte[1];

    packet.data[134] = param_failed_name_byte[0];
    packet.data[135] = param_failed_name_byte[1];
    packet.data[136] = param_failed_name_byte[2];
    packet.data[137] = param_failed_name_byte[3];
    packet.data[138] = param_failed_name_byte[4];
    packet.data[139] = param_failed_name_byte[5];
    packet.data[140] = param_failed_name_byte[6];
    packet.data[141] = param_failed_name_byte[7];
    packet.data[142] = param_failed_name_byte[8];
    packet.data[143] = param_failed_name_byte[9];
    packet.data[144] = param_failed_name_byte[10];
    packet.data[145] = param_failed_name_byte[11];
    packet.data[146] = param_failed_name_byte[12];
    packet.data[147] = param_failed_name_byte[13];
    packet.data[148] = param_failed_name_byte[14];
    packet.data[149] = param_failed_name_byte[15];

    //////////////////////////////////////////////////////////////////////////

    // Calc to CRC16
    packet.crc16 = CRC16_cal(packet.data, PACKET_SIZE - 2, *crc16_tab);

    // Set CRC to buf
    packet.data[PACKET_SIZE - 2] = (char)(packet.crc16 & 0xFF);
    packet.data[PACKET_SIZE - 1] = (char)((packet.crc16 >> 8) & 0xFF); 

    // Send
    if (sendto(cliSockDes, packet.data, sizeof(packet.data), 0,
               (struct sockaddr *)&serAddr, sizeof(serAddr)) < 0)
    {
      perror("sending error...\n");
      // close(cliSockDes);
      // exit(-1);
    } else {
      if (flag_write_home_position_ready) {
        // reset flag
        flag_write_home_position_ready = 0;
      }

      if (_flag_read_waypoint_ready) {
        flag_read_waypoint_ready.store(false, std::memory_order_release);
      }

      if (_flag_write_waypoint_ready) {
        flag_write_waypoint_ready.store(false, std::memory_order_release);
      }

      if (_flag_read_param_ready) {
        flag_read_param_ready = false;
        // remove params handler
        mavlink_passthrough->unsubscribe_message(MAVLINK_MSG_ID_PARAM_VALUE, read_param_list_handle);
      }

      if (_flag_write_one_param) {
        flag_write_one_param = 0;
      }
    
      if (_flag_write_param_ready) {
        flag_write_param_ready = false;
      }
    }

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
    
    /******************************* RECEIVED HOME POSITION TO WRITE */
    flag_write_new_home_position_byte[0] = packetRX.data[10];
    flag_write_new_home_position_byte[1] = packetRX.data[11];
    flag_write_new_home_position_byte[2] = packetRX.data[12];
    flag_write_new_home_position_byte[3] = packetRX.data[13];

    home_latitude_byte_rcv[0] = packetRX.data[14];
    home_latitude_byte_rcv[1] = packetRX.data[15];
    home_latitude_byte_rcv[2] = packetRX.data[16];
    home_latitude_byte_rcv[3] = packetRX.data[17];
    home_latitude_byte_rcv[4] = packetRX.data[18];
    home_latitude_byte_rcv[5] = packetRX.data[19];
    home_latitude_byte_rcv[6] = packetRX.data[20];
    home_latitude_byte_rcv[7] = packetRX.data[21];

    home_longitude_byte_rcv[0] = packetRX.data[22];
    home_longitude_byte_rcv[1] = packetRX.data[23];
    home_longitude_byte_rcv[2] = packetRX.data[24];
    home_longitude_byte_rcv[3] = packetRX.data[25];
    home_longitude_byte_rcv[4] = packetRX.data[26];
    home_longitude_byte_rcv[5] = packetRX.data[27];
    home_longitude_byte_rcv[6] = packetRX.data[28];
    home_longitude_byte_rcv[7] = packetRX.data[29];

    home_altitude_byte_rcv[0] = packetRX.data[30];
    home_altitude_byte_rcv[1] = packetRX.data[31];
    home_altitude_byte_rcv[2] = packetRX.data[32];
    home_altitude_byte_rcv[3] = packetRX.data[33];

    /********************************* TAKE OFF */
    flag_action_type_byte[0] = packetRX.data[34];

    altitude_takeoff_byte[0] = packetRX.data[35];
    altitude_takeoff_byte[1] = packetRX.data[36];

    /********************************* WRITE ONE PARAM */
    flag_write_one_param = packetRX.data[37];

    param_name[0] = packetRX.data[38];
    param_name[1] = packetRX.data[39];
    param_name[2] = packetRX.data[40];
    param_name[3] = packetRX.data[41];
    param_name[4] = packetRX.data[42];
    param_name[5] = packetRX.data[43];
    param_name[6] = packetRX.data[44];
    param_name[7] = packetRX.data[45];
    param_name[8] = packetRX.data[46];
    param_name[9] = packetRX.data[47];
    param_name[10] = packetRX.data[48];
    param_name[11] = packetRX.data[49];
    param_name[12] = packetRX.data[50];
    param_name[13] = packetRX.data[51];
    param_name[14] = packetRX.data[52];
    param_name[15] = packetRX.data[53];

    param_value_byte[0] = packetRX.data[54];
    param_value_byte[1] = packetRX.data[55];
    param_value_byte[2] = packetRX.data[56];
    param_value_byte[3] = packetRX.data[57];

    // Set CRC to buf
    crc_buf[0] = packet.data[PACKET_SIZE_RX - 2];
    crc_buf[1] = packet.data[PACKET_SIZE_RX - 1];

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

    std::cout << " Home lat " << home_latitude_val_rcv << std::endl;
    std::cout << " Home long " << home_longitude_val_rcv << std::endl;
    std::cout << " Home alt " << home_altitude_val_rcv << std::endl;

#endif

    /*********************************************** HOME POSITION TO WRITE */
    memcpy(&flag_write_new_home_position, flag_write_new_home_position_byte, sizeof(int));  // test

    memcpy(&home_latitude_val_rcv, home_latitude_byte_rcv, sizeof(double));
    memcpy(&home_longitude_val_rcv, home_longitude_byte_rcv, sizeof(double));
    memcpy(&home_altitude_val_rcv, home_altitude_byte_rcv, sizeof(float));

    /*********************************************** TAKE OFF */
    memcpy(&flag_action_type, flag_action_type_byte, sizeof(int));

    // todo add condition
    memcpy(&altitude_takeoff_val, altitude_takeoff_byte, sizeof(float));

    /*********************************************** WRITE ONE PARAM */
    memcpy(&param_value, param_value_byte, sizeof(float));

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
      std::cout << "......DEBUG:: READ PARAM FLAG::" << (int)packetRX.data[9] << std::endl;
      flag_read_param = 1;
    }
    else
    {
      flag_read_param = 0;
    }

    if (packetRX.data[9] == 0x37)
    {
      std::cout << "......DEBUG:: WRITE PARAM FLAG::" << (int)packetRX.data[9] << std::endl;
      flag_write_param = 1;
    }
    else
    {
      flag_write_param = 0;
    }

    /*********************************** WRITE HOME POSITION */
    if (flag_write_new_home_position)
    {
      std::cout << "__________________Write home position::" << flag_write_new_home_position << std::endl;
      if (writeHomePosition(*mavlink_passthrough)) {
        flag_write_new_home_position = 0;
        flag_write_home_position_ready = 1;
      }
    }

    /*********************************** TAKE OFF */
    if (flag_action_type != 0)
    {
      std::cout << "__________________ACTION::" << (int)flag_action_type << std::endl;
      switch (flag_action_type)
      {
        case 1:
        {
          std::cout << "__________________Take off::" << (int)flag_action_type << std::endl;
          do_takeoff(*action, 100.0);
          break;
        }

        default: {
          std::cout << "__________________ACTION(def)::" << (int)flag_action_type << std::endl;
          break;
        }
      }

    }
  
    /*********************************** WRITE ONE PARAM */
    if (flag_write_one_param) {
      std::cout << "........ DEBUG WRITE ONE PARAM::" << (int)flag_write_one_param 
      << " --NAME::" << param_name << " --VALUE::" << param_value
      << std::endl;

      write_single_param(mavlink_passthrough, write_single_param_handle, param_name, param_value);
    }
  }

  close(cliSockDes);

  return 0;
};


bool writeHomePosition(mavsdk::MavlinkPassthrough &mavlink_passthrough) {
  
  mavsdk::MavlinkPassthrough::CommandLong set_pos_cmd = {
        mavlink_passthrough.get_target_sysid(),
        mavlink_passthrough.get_target_compid(),
        MAV_CMD_DO_SET_HOME,
        MAV_FRAME_GLOBAL,
        0,
        0,
        0,
        home_latitude_val_rcv,
        home_longitude_val_rcv,
        home_altitude_val_rcv
    };

    MavlinkPassthrough::Result mavlink_result = mavlink_passthrough.send_command_long(set_pos_cmd);

    if (mavlink_result == MavlinkPassthrough::Result::Success)
    {
        std::cout << "Write home position command successfully send" << std::endl;
    }
    else
    {
        std::cout << "Write home position command don`t send " << mavlink_result << std::endl;
    }

    return mavlink_result == MavlinkPassthrough::Result::Success;
}

//  todo add return result
void do_takeoff(Action &action, int altitute) {
  // std::cout << "Arming...\n";
  // const Action::Result arm_result = action.arm();

  // if (arm_result != Action::Result::Success) {
  //     std::cerr << "Arming failed: " << arm_result << '\n';
  //     return;
  // } else {
  //     std::cout << "Arming success" << std::endl;
  // }

    mavsdk::Action::Result result_takeoff_alt = action.set_takeoff_altitude(100.0);

  if (result_takeoff_alt == mavsdk::Action::Result::Success) {
    std::cout << "Takeoff altitude::Success" << std::endl;
  } else {
    std::cout << "Takeoff altitude command don`t send " << result_takeoff_alt << std::endl;
  }

  // Take off
  std::cout << "Taking off...\n";
  const Action::Result takeoff_result = action.takeoff();
  if (takeoff_result != Action::Result::Success) {
      std::cerr << "Takeoff failed: " << takeoff_result << '\n';
  } else {
      std::cout << "Takeoff success" << std::endl;
  }
};

MavlinkPassthrough::MessageHandle subscribe_param_list(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, ThreadTimer &params_read_timer) {
  MavlinkPassthrough::MessageHandle param_handle = mavlink_passthrough->subscribe_message(
    MAVLINK_MSG_ID_PARAM_VALUE,
    [&mavlink_passthrough, &param_handle, &params_read_timer](const mavlink_message_t &msg) {
      mavlink_param_value_t param;
      mavlink_msg_param_value_decode(&msg, &param);


      // initialize params list, MOVE FROM HERE
      if (params_list.size() == 0) {
        params_list.resize(param.param_count);
      }


      if ((int)param.param_index < (int)param.param_count) {
        params_list[(int)param.param_index] = AutopilotParam(param.param_id, param.param_value, param.param_type , param.param_index);
        
      } else {
        std::cout << "**NEW MESSAGE** INDEX BIGGER THEN COUNT::" << (int)param.param_index << " --NAME::" << param.param_id <<  std::endl;
      }

      // remove received param from missing list
      // if (read_param_missing_index_list.size() > 0 && (int)param.param_index < (int)param.param_count) {
      //   auto it = std::find(read_param_missing_index_list.begin(), read_param_missing_index_list.end(), (int)param.param_index);
      //   if (it != read_param_missing_index_list.end()) {
      //     read_param_missing_index_list.erase(it);
      //   }
      // }

      int param_timeout_ms = in_reading_state ? 1500 : 700;

      if (in_reading_state) {
        params_read_timer.setTimeout(
          [&]() {
            // todo use flag ready result
            if (read_param_missing_index_list.size() == 0) {
              for (int i = 0; i < params_list.size(); i++) {
                if (params_list[i].get_index() < 0) {
                  read_param_missing_index_list.push_back(i);
                }
              }
            }

            in_reading_state = false;

            if (read_param_missing_index_list.size() > 0) {
              request_param_by_index(mavlink_passthrough, read_param_missing_index_list.back());
            } else {
              if (saveParamsToFile(getDestDirPath() + PARAM_FILENAME, params_list)) {
                flag_read_param_ready = true;
              }
            }

            std::cout << ">>>>>>>>>>>>>TIMEOUT-RECEIVING PARAMS::SIZE MISSING:: " << read_param_missing_index_list.size() << std::endl;
          },
          param_timeout_ms
        );

      } else {
        params_read_timer.setReties(5);
        params_read_timer.clear();
        params_read_timer.setInterval(
          [&]() {
            if (read_param_missing_index_list.size() == 0) {
              for (int i = 0; i < params_list.size(); i++) {
                if (params_list[i].get_index() < 0) {
                  read_param_missing_index_list.push_back(i);
                }
              }
            }

            if (read_param_missing_index_list.size() > 0) {
              request_param_by_index(mavlink_passthrough, read_param_missing_index_list.back());
              read_param_missing_index_list.pop_back();
              std::cout << ">>>>>>>>>>>>>TIME-INTERVAL-RECEIVING PARAMS::SIZE MISSING:: " << read_param_missing_index_list.size() << std::endl;
            } else {
              if (saveParamsToFile(getDestDirPath() + PARAM_FILENAME, params_list)) {
                std::cout << "....... DEBUG SET FLAG READY"  << std::endl;
                flag_read_param_ready = true;
                all_params_read = true;

                params_read_timer.clear();
              }
            }
          },
          param_timeout_ms
        );
      }
    });

    return param_handle;
}

bool request_param_by_index(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, int index) {

  MavlinkPassthrough::Result params_request_result = mavlink_passthrough->queue_message(
    [&](MavlinkAddress mavlink_address, uint8_t channel) {

    mavlink_message_t message;
    char param_name[16] = {0};

    mavlink_msg_param_request_read_pack(
        mavlink_passthrough->get_our_sysid(),
        mavlink_passthrough->get_our_compid(),
        &message,
        mavlink_passthrough->get_target_sysid(),
        mavlink_passthrough->get_target_compid(),
        param_name,
        (int16_t)index
      );

    return message; 
  });

  return params_request_result == MavlinkPassthrough::Result::Success;
}


void subscribe_write_param_list_result(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, 
                                      MavlinkPassthrough::MessageHandle &write_param_list_handle,
                                      ThreadTimer &_params_write_result_timer
                                      ) {

  write_param_list_handle = mavlink_passthrough->subscribe_message(
    MAVLINK_MSG_ID_PARAM_VALUE,
    [&mavlink_passthrough, &write_param_list_handle, &_params_write_result_timer](const mavlink_message_t &msg) {
      mavlink_param_value_t param;
      mavlink_msg_param_value_decode(&msg, &param);

      // todo STAT_RUNTIME can be block in global interceptor
      if (compareParamName("STAT_RUNTIME", param.param_id)) {
        std::cout << ".......GET::STAT_RUNTIME -- RETURN " << std::endl;
        std::cout << ".......DEBUG::ALL SEND?:: " << all_params_sent 
        << " -- WRITE LIST RESULT:: " << write_params_list_result.size()
        << std::endl;
        // return;
      } else {
        write_params_list_result.push_back(AutopilotParam(param.param_id, param.param_value, param.param_type, param.param_index));
      }


      // TODO:: refactor (remove condition & test)
      // if (all_params_sent && (write_params_list_result.size() >= params_list.size())) {
      if (all_params_sent) {

        std::cout << ".......DEBUG::MEAT CONDITION::RECEIVING WRITE PARAMS" << std::endl;

        _params_write_result_timer.setTimeout(
          [&]() {
            all_params_write_result_ready = true;
            mavlink_passthrough->unsubscribe_message(MAVLINK_MSG_ID_PARAM_VALUE, write_param_list_handle);
            std::cout << ".......DEBUG::UNSUBSCRIBE WRITE PARAM LIST RESULT::" << std::endl;
            
          }, 5000
        );
      }
    });
}

void write_missing_params(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, ThreadTimer &_params_write_timer) {

  _params_write_timer.setReties(write_params_missing_index_list.size());
  _params_write_timer.setInterval(
    [&mavlink_passthrough]() {

      MavlinkPassthrough::Result set_param = mavlink_passthrough->queue_message(
            [&mavlink_passthrough](MavlinkAddress mavlink_address, uint8_t channel) {
            
            auto &param = params_list.at(write_params_missing_index_list.back());

            mavlink_message_t message;
            mavlink_msg_param_set_pack(
                mavlink_passthrough->get_our_sysid(),
                mavlink_passthrough->get_our_compid(),
                &message,
                mavlink_passthrough->get_target_sysid(), 
                mavlink_passthrough->get_target_compid(),
                param.get_name().c_str(),
                param.get_value(),
                param.get_type()
            );
            return message; 
        });

        if (set_param != MavlinkPassthrough::Result::Success) {
            std::cerr << "xxxxxxxxxxxxxxx FAILED SEND PARAM" << set_param << std::endl;
        } else {
          write_params_missing_index_list.pop_back();
        }

        if (write_params_missing_index_list.size() == 0) {
          all_params_sent = true;
          std::cout << "+++++++++++++++++++++++WRITE ALL PARAM DONE::" << std::endl;
        }

    }, WRITE_PARAM_INTERVAL_MS
  );
}


void write_param_list(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, ThreadTimer &_params_write_timer) {
  loadParamFromFile(getDestDirPath() + "write_all.param", params_list);

  if (params_list.size() == 0) {
    // todo
    std::cout << "____________FAIL WRITE PARAM LIST________PARAM LIST EMPTY::" << std::endl;
    return;
  }
  std::cout << "...DEBUG::START UPDATE PARAMS" << std::endl;

  _params_write_timer.setReties(params_list.size());
  _params_write_timer.setInterval(
    [&mavlink_passthrough]() {

      MavlinkPassthrough::Result set_param = mavlink_passthrough->queue_message(
            [&mavlink_passthrough](MavlinkAddress mavlink_address, uint8_t channel) {
            
            auto &param = params_list.at(current_index_to_write);

            mavlink_message_t message;
            mavlink_msg_param_set_pack(
                mavlink_passthrough->get_our_sysid(),
                mavlink_passthrough->get_our_compid(),
                &message,
                mavlink_passthrough->get_target_sysid(), 
                mavlink_passthrough->get_target_compid(),
                param.get_name().c_str(),
                param.get_value(),
                param.get_type()
            );
            return message; 
        });

        if (set_param != MavlinkPassthrough::Result::Success) {
            std::cerr << "xxxxxxxxxxxxxxx FAILED SEND PARAM " << set_param << std::endl;
        } else {
          current_index_to_write++;
        }

        if (current_index_to_write >= params_list.size()) {
          all_params_sent = true;
          std::cout << "++++++++++++++++++++++++++++++++++++WRITE ALL PARAM DONE::" << std::endl;
        }

    }, WRITE_PARAM_INTERVAL_MS
  );
}

bool write_single_param(std::shared_ptr<mavsdk::MavlinkPassthrough> &mavlink_passthrough, MavlinkPassthrough::MessageHandle &write_single_param_handle, char param_name[16], float param_value) {
  loadParamFromFile(getDestDirPath() + PARAM_FILENAME, params_list);

  uint8_t param_type = 0;
  for (int i = 0; i < params_list.size(); i++) {
    if (params_list[i].get_name() == param_name) {
      param_type = params_list[i].get_type();
      break;
    }
  }

  write_single_param_handle = mavlink_passthrough->subscribe_message(
    MAVLINK_MSG_ID_PARAM_VALUE,
    [&mavlink_passthrough, &write_single_param_handle, param_name, param_value](const mavlink_message_t &msg) {
      mavlink_param_value_t param;
      mavlink_msg_param_value_decode(&msg, &param);

      if (strcmp(param.param_id, param_name) == 0) {
        mavlink_passthrough->unsubscribe_message(MAVLINK_MSG_ID_PARAM_VALUE, write_single_param_handle);
        std::cout << "____________________PARAM UNSIBSCRIBE:ONE_P: " << std::endl;

        if (param.param_value == param_value) {
          std::cout << "____________________PARAM UPDATE SUCCESS:ONE_P:" << param.param_id << "--VALUE::" << param.param_value << std::endl;
        } else {
          std::cout << "____________________PARAM UPDATE FAIL:ONE_P:" << param.param_id << " --VALUE::" << param.param_value << " --SHOULD BE::" << param_value << std::endl;
        }
      }

    });

  MavlinkPassthrough::Result set_param = mavlink_passthrough->queue_message(
        [param_name, param_value, param_type, &mavlink_passthrough](MavlinkAddress mavlink_address, uint8_t channel) {
        
        mavlink_message_t message;
        mavlink_msg_param_set_pack(
            mavlink_passthrough->get_our_sysid(),
            mavlink_passthrough->get_our_compid(),
            &message,
            mavlink_passthrough->get_target_sysid(), 
            mavlink_passthrough->get_target_compid(),
            param_name,
            param_value,
            param_type
        );
        return message; 
    });

    if (set_param != MavlinkPassthrough::Result::Success) {
        std::cerr << "xxxxxxxxxxxxxxx FAILED SEND PARAM " << set_param << std::endl;
    } else {
        std::cout << "____________________Send param new value: " << set_param << std::endl;
    }

    return set_param == MavlinkPassthrough::Result::Success;
}

void mavlink_message_callback(const mavlink_message_t &msg)
{
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t vfr_hud;
      mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);

#if defined(DEBUG)
    std::cout << "Heading: " << vfr_hud.heading << std::endl;
#endif

      compass_azimuth_val = vfr_hud.heading;
      break;
    }

    case MAVLINK_MSG_ID_RADIO_STATUS: {
      mavlink_radio_status_t radio_status;
      mavlink_msg_radio_status_decode(&msg, &radio_status);

  #if defined(DEBUG)
      std::cout << "RSSI: " << static_cast<unsigned int>(radio_status.rssi) << std::endl;
      std::cout << "Remote RSSI: " << radio_status.remrssi << std::endl;
      std::cout << "TX buffer: " << static_cast<unsigned int>(radio_status.txbuf) << std::endl;
      std::cout << "Noise: " << static_cast<unsigned int>(radio_status.noise) << std::endl;
      std::cout << "Remote noise: " << static_cast<unsigned int>(radio_status.remnoise) << std::endl;
      std::cout << "RX errors: " << static_cast<unsigned int>(radio_status.rxerrors) << std::endl;
      std::cout << "Fixed: " << static_cast<unsigned int>(radio_status.fixed) << std::endl;
  #endif

      // radio_status_rssi_val = radio_status.rssi;
      radio_status_remrssi_val = radio_status.remrssi;
      radio_status_txbuf_val = radio_status.txbuf;
      radio_status_noise_val = radio_status.noise;
      radio_status_remnoise_val = radio_status.remnoise;
      radio_status_rxerrors_val = radio_status.rxerrors;
      radio_status_fixed_val = radio_status.fixed;
      break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t attitude;
      mavlink_msg_attitude_decode(&msg, &attitude);

#if defined(DEBUG)
    std::cout << "Roll attitude: " << attitude.roll << std::endl;
    std::cout << "Pitch attitude: " << attitude.pitch << std::endl;
    std::cout << "Yaw attitude: " << attitude.yaw << std::endl;
#endif

      imu_roll_val = attitude.roll;
      imu_pitch_val = attitude.pitch;
      imu_yaw_val = attitude.yaw;
    }

  }
}

void get_params_failed(std::vector<AutopilotParam> &params_list, 
                      std::vector<AutopilotParam> &_write_params_list_result, 
                      std::vector<AutopilotParam> &write_params_fails_list,
                      std::vector<int> &_write_params_missing_index_list
                      ) {
  std::cout << "..........DEBUG::GET FAILED PARAMS::" << std::endl;

  removeFirstOccurrenceOfDuplicates(_write_params_list_result);

  // todo remove param from write_params_fails_list that in _write_params_list_result
  if (write_params_fails_list.size() > 0) {
    for (int i = 0; i < _write_params_list_result.size(); i++) {
      
      for (int j = 0; j < write_params_fails_list.size(); j++) {
        if (_write_params_list_result[i].get_name() == write_params_fails_list[j].get_name()) {
          write_params_fails_list.erase(write_params_fails_list.begin() + j);
          break;
        }
      }
    }
  }

  for (int i = 0; i < params_list.size(); i++) {

    bool is_param_found = false;
    for (int j = 0; j < _write_params_list_result.size(); j++) {
      if (params_list[i].get_name() == _write_params_list_result[j].get_name()) {
        if (params_list[i].get_value() != _write_params_list_result[j].get_value()) {
          std::cout << ".........PARAM UPDATE FAIL:: " 
          << params_list[i].get_name() 
          << std::setprecision(15)
          << " --VALUE::" << _write_params_list_result[j].get_value() 
          << " --SHOULD BE::" << params_list[i].get_value() << std::endl;

          if (write_param_retries > 0 && write_param_retries < 2) {
            _write_params_missing_index_list.push_back(i);
          }
          write_params_fails_list.push_back(_write_params_list_result[j]);

        }
        is_param_found = true;
        break;
      }
    }
    if (!is_param_found) {
      _write_params_missing_index_list.push_back(i);
    }
  }

  removeFirstOccurrenceOfDuplicates(write_params_fails_list);

  std::cout << "..........DEBUG::PARAMS GET -> FAILES COUNT:: " << write_params_fails_list.size() 
  << "......::MISSING COUNT:: " << _write_params_missing_index_list.size() << std::endl;
}

void merge_failed_params(std::vector<AutopilotParam> &params_list,
                        std::vector<AutopilotParam> &write_params_fails_list,
                        std::vector<int> &_write_params_missing_index_list
                        ) {
  std::cout << "..........DEBUG::MERGING FAILED PARAMS" << std::endl;

  for (int i = 0; i < _write_params_missing_index_list.size(); i++) {
    write_params_fails_list.push_back(params_list[_write_params_missing_index_list[i]]);
  }
}