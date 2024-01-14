#include "config.h"

Config::Config(const string &file, const string &default_file)
{
    path = file, default_path = default_file;
    node = YAML::LoadFile(path);
    loadYAML();
}

void Config::loadYAML()
{
    // 读取雷达参数
    lidar_port_name = node["LiDAR"]["port_name"].as<string>();
    lidar_baud_rate = node["LiDAR"]["baud_rate"].as<size_t>();
    initialize_scan_num = node["LiDAR"]["initialize_scan_num"].as<size_t>();
    scan_num = node["LiDAR"]["scan_num"].as<size_t>();
    buffer_capacity = node["LiDAR"]["buffer_capacity"].as<size_t>();
    mode_id = node["LiDAR"]["mode_id"].as<size_t>();
    // 读取点云处理参数
    proc_freq = node["Analysis"]["proc_freq"].as<size_t>();
    max_dist = node["Analysis"]["max_dist"].as<double>();
    dist_threshold = node["Analysis"]["dist_threshold"].as<double>();
    num_threshold = node["Analysis"]["num_threshold"].as<size_t>();
    static_radius = node["Analysis"]["static_radius"].as<double>();
    static_min_neighbor = node["Analysis"]["static_min_neighbor"].as<size_t>();
    dynamic_radius = node["Analysis"]["dynamic_radius"].as<double>();
    dynamic_min_neighbor = node["Analysis"]["dynamic_min_neighbor"].as<size_t>();
    cluster_tolerance = node["Analysis"]["cluster_tolerance"].as<double>();
    min_cluster_size = node["Analysis"]["min_cluster_size"].as<size_t>();
    max_cluster_size = node["Analysis"]["max_cluster_size"].as<size_t>();
    // 报警器参数
    alarm_port_name = node["Alarm"]["port_name"].as<string>();
    alarm_baud_rate = node["Alarm"]["baud_rate"].as<size_t>();
    alarm_volume = node["Alarm"]["volume"].as<size_t>();
    // 可视化参数
    be_visualized = node["Visualization"]["be_visualized"].as<bool>();
    port = node["Visualization"]["port"].as<size_t>();
    // 读取其他参数
    verbose = node["verbose"].as<bool>();
}

void Config::setConfig(ConfigMsg &cfg_msg)
{
    // 修改成员变量
    mode_id = cfg_msg.mode_id;
    proc_freq = cfg_msg.proc_freq;
    max_dist = cfg_msg.max_dist;
    dist_threshold = cfg_msg.dist_threshold;
    num_threshold = cfg_msg.num_threshold;
    alarm_volume = cfg_msg.alarm_volume;
}

void Config::getConfig(ConfigMsg &cfg_msg)
{
    cfg_msg.mode_id = mode_id;
    cfg_msg.proc_freq = proc_freq;
    cfg_msg.max_dist = max_dist;
    cfg_msg.dist_threshold = dist_threshold;
    cfg_msg.num_threshold = num_threshold;
    cfg_msg.alarm_volume = alarm_volume;
}

void Config::defaultConfig(ConfigMsg &config_msg)
{
    node = YAML::LoadFile(default_path);
    loadYAML();
    getConfig(config_msg);
}

void Config::saveConfig(ConfigMsg &config_msg)
{
    setConfig(config_msg);

    // 修改node特定值
    node["LiDAR"]["mode_id"] = mode_id;
    node["Analysis"]["proc_freq"] = proc_freq;
    node["Analysis"]["max_dist"] = max_dist;
    node["Analysis"]["dist_threshold"] = dist_threshold;
    node["Analysis"]["num_threshold"] = num_threshold;
    node["Alarm"]["volume"] = alarm_volume;

    // 保存文件
    ofstream ofs(path, ios::out | ios::trunc);
    ofs << node;
    ofs.close();
}
