# Flight Control Backend using MAVSDK

This repository contains the backend code for a flight control system developed using MAVSDK C++ API. 
MAVSDK is a set of libraries in various programming languages for communicating with MAVLink systems like drones, ground stations, and companion computers. 

## Requirements

Install MAVSDK:

https://mavsdk.mavlink.io/main/en/cpp/guide/build.html

Install mavproxy:

https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html

Start mavproxy:

mavproxy.py --out 127.0.0.1:14550

## Build the project

Generate build files using CMake:

```
cmake -Bbuild -H.
```

Build the project:

```
cmake --build build -j8
```

Run the project

```
./build/autopilot_control
```

> [!NOTE]
> For more information about MAVSDK and its usage, refer to the [official documentation](https://mavsdk.mavlink.io/main/en/cpp/guide/).



