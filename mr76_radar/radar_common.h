#ifndef RADAR_COMMON_H_
#define RADAR_COMMON_H_

#include <iostream>
#include <vector>
#include <cmath>

const int kCanDataSize = 8; // can data is 8 bytes

// target properties
struct TargetData {
    int16_t id;           // object id， 目标 ID
    float dist_long;      // longitudinal distance，目标纵向距离，单位 m
    float dist_lat;       // latitudinal distance， 目标横向距离，单位 m
    float vel_rel_long;   // relative velocity in longitudinal direction， 目标纵向速度，单位 m/s
    float vel_rel_lat;    // relative velocity in latitudinal direction，  目标横向速度，单位 m/s
    int16_t dynamic_prop; // dynamic properties， 目标动态属性
    float rcs; // radar cross section， 目标雷达散射截面，单位 dBsm
};

// mr76 radar properties
struct RadarData {
    int16_t radar_state;
    int32_t total_targets;
    std::vector<TargetData> targets;
};

#endif // RADAR_COMMON_H_