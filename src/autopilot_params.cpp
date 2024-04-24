
#include "autopilot_params.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mavsdk/plugins/param/param.h>
#include <regex>
#include <string>

#define DEBUG

void readParams(const std::string &filename,
                const mavsdk::Param::AllParams &all_params)
{
  std::ofstream outputFile(filename);

  if (!outputFile.is_open())
  {
    std::cerr << "Unable to open the file for writing: " << filename << std::endl;
    return;
  }

  for (const auto &float_param : all_params.float_params)
  {
    outputFile << float_param.name << " " << float_param.value << std::endl;

#if defined(DEBUG)
    std::cout << float_param.name << " " << float_param.value << std::endl;
#endif
  }

  for (const auto &int_param : all_params.int_params)
  {
    outputFile << int_param.name << " " << int_param.value << std::endl;

#if defined(DEBUG)
    std::cout << int_param.name << " " << int_param.value << std::endl;
#endif
  }

  outputFile.close();
  std::cout << "Data successfully written to the file " << filename << '\n';
}

mavsdk::Param::AllParams writeParams(const std::string &filename)
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
        std::cout << paramName << " " << static_cast<int32_t>(paramValue)
                  << std::endl;
      }
      else
      {
        all_params.float_params.push_back(
            {paramName, static_cast<float>(paramValue)});
        std::cout << paramName << " " << paramValue << std::endl;
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