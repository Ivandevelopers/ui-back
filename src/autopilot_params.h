#pragma once

#include <mavsdk/plugins/param/param.h>
#include <string>

// Reads waypoints from controller to file .parm
void readParams(const std::string &filename,
                const mavsdk::Param::AllParams &all_params);

// Reads waypoints from .parm file
mavsdk::Param::AllParams writeParams(const std::string &filename);