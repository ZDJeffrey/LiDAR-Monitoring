#ifndef SOCKET_TYPE_H
#define SOCKET_TYPE_H

#include <cctype>

#define POINT_NUM 2500
#define POINT_SEGMENT 100

struct Point
{
    float x;
    float y;
};

struct PointCloudData
{
    unsigned int len;
    Point points[POINT_NUM];
};

struct UpdateDataType
{
    bool static_update;
    bool safe_update;
    bool warning_update;
};

enum ControlMsg
{
    SENDCLOUD,
    INITIALIZATION,
    START,
    STOP,
    SETCONFIG,
    GETCONFIG,
    SAVECONFIG,
    DEFAULTCONFIG
};

struct ConfigMsg
{
    // 雷达参数
    size_t mode_id; // 雷达模式

    // 点云处理参数
    size_t proc_freq; // 处理频率
    double max_dist; // 最大距离
    double dist_threshold; // 距离精度
    size_t num_threshold; // 报警阈值

    // 报警器参数
    size_t alarm_volume;
};

#endif