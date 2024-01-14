#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "socket_type.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "monitor.h"
#include "config.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

class Monitor; // 声明监视器，两个类互相包含，需要声明

class Visualization
{
public:
    Visualization(const Config &config);
    ~Visualization();
    bool createServer();
    void setMonitor(Monitor *monitor); // 设置监视器

private:
    void serverThread();
    void sendCloud();
    void copyData(PointCloudPtr cloud, PointCloudData &data); // 将点云数据拷贝到数据结构中

private:
    // 变量
    int listen_socket_, client_socket_;
    int port_;                                              // 端口号
    std::thread server_;                                    // 服务器线程
    std::mutex srv_mtx_;                                    // 同步锁
    PointCloudData static_data_, safe_data_, warning_data_; // 数据信息
    ConfigMsg cfg_msg_;                                     // 配置信息
    bool is_end_;
    bool is_connected_;
    bool is_first_;                                        // 是否是第一次发送
    PointCloudPtr static_cloud, safe_cloud, warning_cloud; // 点云

    // 模块
    Monitor *monitor_;
};

#endif