
#include "autopilot_mission.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <regex>
#include <sstream>
#include <thread>

#include <filesystem>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#define DEBUG

using namespace mavsdk;

void readWaypoints(
    const std::vector<MissionRaw::MissionItem> &mission_raw_items,
    const std::string &filename) {

  std::filesystem::path file_path(filename);

  if (file_path.extension() != ".plan") {
    std::cerr << "Mission file must have .plan extension." << std::endl;
    return;
  }

  std::ofstream file(file_path);

  if (!file.is_open()) {
    std::cerr << "Cannot open file!" << std::endl;
    return;
  }

  file << "QGC\tWPL\t110\n";

  for (const auto &waypoint : mission_raw_items) {
    file << waypoint.seq << '\t' << waypoint.current << '\t' << waypoint.frame
         << '\t' << waypoint.command << '\t' << waypoint.param1 << '\t'
         << waypoint.param2 << '\t' << waypoint.param3 << '\t'
         << waypoint.param4 << '\t' << waypoint.x / 1e7 << '\t'
         << waypoint.y / 1e7 << '\t' << waypoint.z << '\t'
         << waypoint.autocontinue << '\t' << waypoint.mission_type << '\n';

#if defined(DEBUG)
    std::cout << waypoint.seq << " " << waypoint.current << " "
              << waypoint.frame << " " << waypoint.command << " "
              << waypoint.param1 << " " << waypoint.param2 << " "
              << waypoint.param3 << " " << waypoint.param4 << " " << waypoint.x
              << " " << waypoint.y << " " << waypoint.z << " "
              << waypoint.autocontinue << '\n';
#endif
  }

  file.close();

  std::cout << "Waypoints written to mission.plan" << std::endl;
}

std::vector<MissionRaw::MissionItem>
writeWaypoints(const std::string &filename) {

  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Unable to open file." << std::endl;
    return std::vector<MissionRaw::MissionItem>();
  }

  std::string firstLine;
  std::getline(file, firstLine);

  // TO DO: Change for more efficient approach

  std::vector<MissionRaw::MissionItem> waypoints;
  std::string line;

  while (std::getline(file, line)) {

    std::stringstream ss(line);

    std::string seq;
    std::getline(ss, seq, '\t');

    std::cout << "Seq: " << seq << std::endl;

    std::string current;
    std::getline(ss, current, '\t');

    std::cout << "Current: " << current << std::endl;

    std::string coord_frame;
    std::getline(ss, coord_frame, '\t');

    std::cout << "Coord frame: " << coord_frame << std::endl;

    std::string command;
    std::getline(ss, command, '\t');

    std::cout << "Command: " << command << std::endl;

    std::string param1;
    std::getline(ss, param1, '\t');

    std::cout << "Param 1: " << param1 << std::endl;

    std::string param2;
    std::getline(ss, param2, '\t');

    std::cout << "Param 2: " << param2 << std::endl;

    std::string param3;
    std::getline(ss, param3, '\t');

    std::cout << "Param 3: " << param3 << std::endl;

    std::string param4;
    std::getline(ss, param4, '\t');

    std::cout << "Param 4: " << param4 << std::endl;

    std::string x;
    std::getline(ss, x, '\t');

    std::cout << "X: " << x << std::endl;

    std::string y;
    std::getline(ss, y, '\t');

    std::cout << "Y: " << y << std::endl;

    std::string z;
    std::getline(ss, z, '\t');

    std::cout << "Z: " << z << std::endl;

    std::string autocontinue;
    std::getline(ss, autocontinue, '\t');

    std::cout << "Autocontinue: " << autocontinue << std::endl;

    MissionRaw::MissionItem wp;
    wp.seq = std::stoi(seq);
    wp.current = std::stoi(current);
    wp.frame = std::stoi(coord_frame);
    wp.command = std::stoi(command);
    wp.param1 = std::stod(param1);
    wp.param2 = std::stod(param2);
    wp.param3 = std::stod(param3);
    wp.param4 = std::stod(param4);
    wp.x = std::stof(x) * 1e7;
    wp.y = std::stof(y) * 1e7;
    wp.z = std::stof(z);
    wp.autocontinue = std::stoi(autocontinue);

    waypoints.push_back(wp);
  }

  file.close();

  return waypoints;
}