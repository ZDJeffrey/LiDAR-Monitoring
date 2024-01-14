#ifndef SCAN_H
#define SCAN_H

#include "sl_lidar.h"
#include <cstdio>
#include <cmath>
#include <vector>
#include "config.h"
using namespace sl;
using namespace std;

typedef sl_lidar_response_measurement_node_hq_t Node_t;

// 计算弧度
inline double getAngleRad(const Node_t &node)
{
    return (180.0 - node.angle_z_q14 * 90.f / (1 << 14)) * M_PI / 180.0;
}

inline double getDist(const Node_t &node)
{
    return node.dist_mm_q2 / 1000.f / (1 << 2);
}

inline size_t getQuality(const Node_t &node)
{
    return node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
}

class Scan
{

public:
    Scan(const Config &config);
    ~Scan();
    bool createDriver();
    void deleteDriver();
    void start();
    bool scan(vector<Node_t> &nodes, size_t num);
    void stop();
    void updateConfig(const Config &config);

    size_t mode_id_;       // 模式ID
private:
    ILidarDriver *driver_; // 雷达驱动
    sl_result result_;     // 雷达相关操作结果（正确性检测）
    Node_t *buffer;        // 缓冲区
    size_t capacity_;      // 缓冲区容量
    string port_name_;     // 端口名
    size_t baud_rate_;     // 波特率
};

#endif
