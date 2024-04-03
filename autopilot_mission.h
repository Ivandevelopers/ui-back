#pragma once

#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <string>

using namespace mavsdk;

// Reads waypoints from controller to file .plan
void readWaypoints(
    const std::vector<MissionRaw::MissionItem> &mission_raw_items,
    const std::string &filename);

// Reads waypoints from .plan file
std::vector<MissionRaw::MissionItem>
writeWaypoints(const std::string &filename);