
#include "autopilot_params.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mavsdk/plugins/param/param.h>
#include <regex>
#include <string>

#define DEBUG

class ParamBase
{
public:
  virtual ~ParamBase() {}
  virtual std::string getName() const = 0;
  virtual float getValue() const = 0;
};

class FloatParam : public ParamBase
{
public:
  FloatParam(const mavsdk::Param::FloatParam &param) : name(param.name), value(param.value) {}

  std::string getName() const override { return name; }
  float getValue() const override { return value; }

  // Add other members, if needed
private:
  std::string name;
  float value;
};

// Define IntParam class implementing ParamBase
class IntParam : public ParamBase
{
public:
  IntParam(const mavsdk::Param::IntParam &param) : name(param.name), value(param.value) {}

  std::string getName() const override { return name; }
  float getValue() const override { return value; }

  // Add other members, if needed
private:
  std::string name;
  int value;
};

bool compareParams(const std::unique_ptr<ParamBase> &param1, const std::unique_ptr<ParamBase> &param2)
{
  return param1->getName() < param2->getName();
}

bool readParamsFromControllerToFile(const std::string &filename,
                                    const mavsdk::Param::AllParams &all_params)
{
  std::ofstream outputFile(filename);

  if (!outputFile.is_open())
  {
    std::cerr << "Unable to open the file for writing: " << filename << std::endl;
    return 0;
  }

  // Combine float_params and int_params into a single vector of ParamBase pointers
  std::vector<std::unique_ptr<ParamBase>> all_sorted_params;
  for (const mavsdk::Param::FloatParam &float_param : all_params.float_params)
  {
    all_sorted_params.push_back(std::make_unique<FloatParam>(float_param));
  }
  for (const mavsdk::Param::IntParam &int_param : all_params.int_params)
  {
    all_sorted_params.push_back(std::make_unique<IntParam>(int_param));
  }

  // Sort the combined vector based on parameter names
  std::sort(all_sorted_params.begin(), all_sorted_params.end(), compareParams);

  // Write the sorted parameters to the file
  for (const auto &param : all_sorted_params)
  {
    outputFile << param->getName() << "," << param->getValue() << std::endl;
#if defined(DEBUG)
    std::cout << param->getName() << " " << param->getValue() << std::endl;
#endif
  }

  outputFile.close();
  std::cout << "Data successfully written to the file " << filename << '\n';

  return 1;
}

mavsdk::Param::AllParams writeParamsToControllerFromFile(const std::string &filename)
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