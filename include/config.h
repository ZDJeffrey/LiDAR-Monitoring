#ifndef CONFIG_H
#define CONFIG_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <fstream>
#include "socket_type.h"

using namespace std;

class Config
{
public:
    Config(const string &file,const string& default_file);

    void loadYAML();

    void setConfig(ConfigMsg& cfg_msg);
    void getConfig(ConfigMsg& cfg_msg);
    void saveConfig(ConfigMsg &config_msg);
    void defaultConfig(ConfigMsg &config_msg);

    // yaml文件
    YAML::Node node;
    // 保存文件路径
    string path,default_path;

    // 雷达参数
    string lidar_port_name;
    size_t lidar_baud_rate;
    size_t initialize_scan_num, scan_num;
    size_t buffer_capacity;
    size_t mode_id;
    // 点云处理参数
    size_t proc_freq;
    double max_dist;
    double dist_threshold;
    size_t num_threshold;
    double dynamic_radius, static_radius;
    size_t dynamic_min_neighbor, static_min_neighbor;
    double cluster_tolerance;
    size_t min_cluster_size, max_cluster_size;
    // 报警器参数
    string alarm_port_name;
    size_t alarm_baud_rate;
    size_t alarm_volume;
    // 可视化参数
    bool be_visualized;
    size_t port;
    // 其他参数
    bool verbose;
};

#endif