#ifndef MONITOR_H
#define MONITOR_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "scan.h"
#include "config.h"
#include "analysis.h"
#include "socket_type.h"
#include "alarm.h"
#include "rate.h"
#include "visualization.h"
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

class Visualization; // 声明可视化模块，两个类互相包含，需要声明

class Monitor
{
public:
    Monitor(Config &config);
    ~Monitor();
    void start();
    void stop();
    void initialize();
    void updatePointcloud(UpdateDataType &type, PointCloudPtr &static_cloud, PointCloudPtr &safe_cloud, PointCloudPtr &warning_cloud);
    void setConfig(ConfigMsg &config_msg) { config_->setConfig(config_msg); updateConfig();}
    void saveConfig(ConfigMsg &config_msg) { config_->saveConfig(config_msg); updateConfig();}
    void getConfig(ConfigMsg &config_msg) { config_->getConfig(config_msg); }
    void defaultConfig(ConfigMsg &config_msg) { config_->defaultConfig(config_msg); updateConfig();}
    void updateConfig();

private:
    void scan();
    bool getPointcloud();
    void processPointcloud();

private:
    // 状态
    bool is_initialized_;
    bool is_scanning_;
    bool is_new_scan_;

    // 变量
    vector<vector<Node_t>> scan_data_;
    vector<Node_t> nodes_;
    mutex flag_mtx_, data_mtx_;
    thread set_thread_, scan_thread_;

    // 参数
    bool be_visualized_;
    bool verbose_;
    const size_t initialize_scan_num_, scan_num_;
    size_t proc_freq_;

    // 模块
    Config *config_;
    Scan *scan_;                   // 激光雷达扫描模块
    Analysis *analysis_;           // 数据分析模块
    Alarm *alarm_;                 // 报警模块
    Visualization *visualization_; // 可视化模块
};

#endif