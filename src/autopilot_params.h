#pragma once

#ifndef AUTOPILOT_PARAMS_H
#define AUTOPILOT_PARAMS_H

#include <mavsdk/plugins/param/param.h>
#include <string>
#include <iostream>
#include <cstring>  


static bool compareParamName(const std::string& stdStr, const char charStr[16]) {
    size_t charStrLen = 16;
    for (size_t i = 0; i < 16; ++i) {
        if (charStr[i] == '\0') {
            charStrLen = i;
            break;
        }
    }
    
    if (stdStr.size() != charStrLen) {
        return false;
    }
    
    for (size_t i = 0; i < charStrLen; ++i) {
        if (stdStr[i] != charStr[i]) {
            return false;
        }
    }
    
    return true;
}

// todo - refactor
struct AutopilotParam {

    void set_value(float new_value) { value = new_value; }

    std::string get_name() const { return name; }
    float get_value() const { return value; }
    uint8_t get_type() const { return type; }
    int get_index() const { return index; }

    AutopilotParam(const char *name = "", float value = 0.0, uint8_t type = 0, int index = -1) 
        : value(value), type(type), index(index) {
            
            if (name[15] != 0) {
                this->name = std::string(name, 16);
            } else {
                this->name = std::string(name);
            }
    }

private:
    std::string name;
    float value;
    uint8_t type;
    int index;
}; 

void removeFirstOccurrenceOfDuplicates(std::vector<AutopilotParam>& param_list);

bool saveParamsToFile(const std::string &filename, std::vector<AutopilotParam> &params_list);

bool loadParamFromFile(const std::string &filename, std::vector<AutopilotParam> &params_list);

// Reads waypoints from .parm file
mavsdk::Param::AllParams writeParamsFromFileToController(const std::string &filename);

#endif // AUTOPILOT_PARAMS_H