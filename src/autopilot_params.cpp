
#include "autopilot_params.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mavsdk/plugins/param/param.h>
#include <regex>
#include <string>
#include <unordered_set>

#define DEBUG

bool saveParamsToFile(const std::string &filename, std::vector<AutopilotParam> &params_list) {
  std::ofstream outputFile(filename);

  if (!outputFile.is_open()) {
    std::cerr << "Unable to open the file for writing: " << filename << std::endl;
    return 0;
  }

  for (auto &param : params_list) {


    outputFile << param.get_name() << ",";

    if (param.get_type() == 9 || param.get_type() == 0) {
      outputFile << std::fixed << std::setprecision(7);
    } else if (param.get_type() == 10) {
      outputFile << std::fixed << std::setprecision(15);
    } else {
      outputFile << std::fixed << std::setprecision(0);
    }
    
    outputFile << param.get_value() << std::endl;
  }

  outputFile.close();

  std::cout << "..............Data successfully written to the file " << filename << '\n';

  return 1;
}

bool loadParamFromFile(const std::string &filename, std::vector<AutopilotParam> &params_list) {
  std::ifstream inputFile(filename);

  if (!inputFile.is_open())
  {
    std::cerr << "Unable to open the file " << filename << " for reading."
              << std::endl;
    return false;
  } 

  std::string line;

  bool is_empty = params_list.empty();
  int count = 0;

  // TODO: refactor logic
  while (std::getline(inputFile, line))
  {
    std::istringstream line_stream(line);

    std::string paramName;
    std::string paramValueStr;

    if (std::getline(line_stream, paramName, ',') && std::getline(line_stream, paramValueStr)) {
      try {
        float paramValue = std::stof(paramValueStr);
        if (!is_empty) {
          for (auto &param : params_list) {
            if (param.get_name() == paramName) {
              param.set_value(paramValue);
              break;
            }
          }
        } else {
          params_list.push_back(AutopilotParam(paramName.c_str(), paramValue, 0, count++));
        }
      } catch (const std::invalid_argument& e) {
        std::cerr << e.what() << '\n';
        // todo return false??
      }
      
    } else {
      std::cerr << "Error reading line: " << line << std::endl;
      return false;
    }
  }
  return true;
}

void removeFirstOccurrenceOfDuplicates(std::vector<AutopilotParam>& param_list) {
    std::unordered_map<std::string, int> last_occurrence;

    for (int i = 0; i < param_list.size(); ++i) {
        last_occurrence[param_list[i].get_name()] = i;
    }

    std::unordered_set<int> indexes_to_remove;

    for (int i = 0; i < param_list.size(); ++i) {
        if (last_occurrence[param_list[i].get_name()] != i) {
            indexes_to_remove.insert(i);
        }
    }

    for (int i = param_list.size() - 1; i >= 0; --i) {
        if (indexes_to_remove.find(i) != indexes_to_remove.end()) {
            param_list.erase(param_list.begin() + i);
        }
    }
}

mavsdk::Param::AllParams writeParamsFromFileToController(const std::string &filename)
{
  mavsdk::Param::AllParams all_params;

  std::ifstream inputFile(filename);

  if (!inputFile.is_open())
  {
    std::cerr << "Unable to open the file " << filename << " for reading."
              << std::endl;
    return all_params;
  }

  std::string line;
  while (std::getline(inputFile, line))
  {
    std::istringstream iss(line);

    std::string paramName;
    double paramValue;

    if (iss >> paramName >> paramValue)
    {
      if (std::floor(paramValue) == paramValue)
      {
        all_params.int_params.push_back(
            {paramName, static_cast<int32_t>(paramValue)});
        std::cout << paramName << "," << static_cast<int32_t>(paramValue)
                  << std::endl;
      }
      else
      {
        all_params.float_params.push_back(
            {paramName, static_cast<float>(paramValue)});
        std::cout << paramName << "," << paramValue << std::endl;
      }
    }
    else
    {
      std::cerr << "Error reading line: " << line << std::endl;
    }
  }

  inputFile.close();
  std::cout << "Successfully read params from file" << std::endl;
  return all_params;
}