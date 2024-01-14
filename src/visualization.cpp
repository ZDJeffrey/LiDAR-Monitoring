#include "visualization.h"

Visualization::Visualization(const Config &config): static_cloud(new PointCloud<PointXYZ>),
                                                    safe_cloud(new PointCloud<PointXYZ>),
                                                    warning_cloud(new PointCloud<PointXYZ>)
{
    port_ = config.port;
    is_end_ = false;
    is_connected_ = false;
    is_first_ = true;
}

Visualization::~Visualization()
{
    close(listen_socket_);
}

bool Visualization::createServer()
{
    // 创建监听socket
    listen_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_socket_ == -1)
    {
        std::cout << "create listen socket error" << std::endl;
        return false;
    }

    struct sockaddr_in bindaddr;
    bindaddr.sin_family = AF_INET;
    bindaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bindaddr.sin_port = htons(port_);
    if (bind(listen_socket_, (struct sockaddr *)&bindaddr, sizeof(bindaddr)) == -1)
    {
        std::cout << "bind listen socket error" << std::endl;
        close(listen_socket_);
        return false;
    }

    server_ = std::thread(&Visualization::serverThread, this); // 开启服务器线程
    return true;
}

void Visualization::setMonitor(Monitor *monitor)
{
    monitor_ = monitor;
}

void Visualization::serverThread()
{
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr); // accept函数第三个参数需要的类型

    // 监听
    if (listen(listen_socket_, 1) == -1) // SOMAXCONN 表示监听队列数默认为128
    {
        fprintf(stderr,"listen error.\n");
        return;
    }

    // 接受客户端的连接
    client_socket_ = accept(listen_socket_, (struct sockaddr *)&client_addr, &client_len);
    is_connected_ = true;
    if (client_socket_ == -1)
    {
        fprintf(stderr,"accept error.\n");
        close(listen_socket_);
        return;
    }

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    // 设置超时时间
    if (setsockopt(client_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
    {
        fprintf(stderr,"setsockopt error\n");
        close(client_socket_);
        close(listen_socket_);
        return;
    }

    is_first_ = true; // 第一次连接申请数据
    ControlMsg control; // 接收的控制信息
    while (true)
    {
        // 检测是否结束
        {
            std::lock_guard<std::mutex> lock(srv_mtx_);
            if (is_end_)
                return;
        }
        if (recv(client_socket_, &control, sizeof(control), 0) > 0) // 获取控制信息
        {
            switch (control)
            {
            case ControlMsg::SENDCLOUD:
                sendCloud();
                break;
            case ControlMsg::INITIALIZATION:
                monitor_->initialize();
                break;
            case ControlMsg::START:
                monitor_->start();
                break;
            case ControlMsg::STOP:
                monitor_->stop();
                break;
            case ControlMsg::SETCONFIG:
                recv(client_socket_, &cfg_msg_,sizeof(ConfigMsg), 0);
                monitor_->setConfig(cfg_msg_);
                break;
            case ControlMsg::GETCONFIG:
                monitor_->getConfig(cfg_msg_);
                send(client_socket_, &cfg_msg_, sizeof(ConfigMsg), 0);
                break;
            case ControlMsg::SAVECONFIG:
                recv(client_socket_, &cfg_msg_,sizeof(ConfigMsg), 0);
                monitor_->saveConfig(cfg_msg_);
                break;
            case ControlMsg::DEFAULTCONFIG:
                monitor_->defaultConfig(cfg_msg_);
                send(client_socket_, &cfg_msg_, sizeof(ConfigMsg), 0);
                break;
            }
        }
        else
        {
            std::cout << "client disconnect." << std::endl;
            // 接受客户端的连接
            is_connected_ = false;
            close(client_socket_);
            client_socket_ = accept(listen_socket_, (struct sockaddr *)&client_addr, &client_len);
            is_connected_ = true;
            is_first_ = true;
        }
    }
}

void Visualization::sendCloud()
{
    bool ret;
    // 获取点云传输类型与点云数据
    UpdateDataType type;
    monitor_->updatePointcloud(type, static_cloud, safe_cloud, warning_cloud);
    if(is_first_)
    {
        type.static_update = true;
        is_first_ = false;
    }

    // 告知客户端接收点云数量
    send(client_socket_, &type, sizeof(UpdateDataType), 0);
    recv(client_socket_, &ret, sizeof(bool), 0);

    // 发送点云
    if (type.static_update)
    {
        copyData(static_cloud, static_data_);
        // 发送点云数量
        send(client_socket_, &static_data_.len, sizeof(unsigned int), 0);
        recv(client_socket_, &ret, sizeof(bool), 0);
        unsigned int num = ceil((float)static_data_.len / (float)POINT_SEGMENT);
        unsigned int start = 0;
        // 分段发送
        for(int i=0;i<num;i++)
        {
            unsigned int len = min((unsigned int)POINT_SEGMENT, static_data_.len - start);
            send(client_socket_, &static_data_.points[start], len * sizeof(Point), 0);
            recv(client_socket_, &ret, sizeof(bool), 0);
            start += len;
        }
    }
    if (type.safe_update)
    {
        copyData(safe_cloud, safe_data_);
        // 发送点云数量
        send(client_socket_, &safe_data_.len, sizeof(unsigned int), 0);
        recv(client_socket_, &ret, sizeof(bool), 0);
        unsigned int num = ceil((float)safe_data_.len / (float)POINT_SEGMENT);
        unsigned int start = 0;
        // 分段发送
        for(int i=0;i<num;i++)
        {
            unsigned int len = min((unsigned int)POINT_SEGMENT, safe_data_.len - start);
            send(client_socket_, &safe_data_.points[start], len * sizeof(Point), 0);
            recv(client_socket_, &ret, sizeof(bool), 0);
            start += len;
        }
    }
    if (type.warning_update)
    {
        copyData(warning_cloud, warning_data_);
        // 发送点云数量
        send(client_socket_, &warning_data_.len, sizeof(unsigned int), 0);
        recv(client_socket_, &ret, sizeof(bool), 0);
        unsigned int num = ceil((float)warning_data_.len / (float)POINT_SEGMENT);
        unsigned int start = 0;
        // 分段发送
        for(int i=0;i<num;i++)
        {
            unsigned int len = min((unsigned int)POINT_SEGMENT, warning_data_.len - start);
            send(client_socket_, &warning_data_.points[start], len * sizeof(Point), 0);
            recv(client_socket_, &ret, sizeof(bool), 0);
            start += len;
        }
    }
}

void Visualization::copyData(PointCloudPtr cloud, PointCloudData &data)
{
    uint num = min((uint)cloud->size(), (uint)POINT_NUM); // 计算拷贝点云数量
    data.len = num;
    for (uint i = 0; i < num; i++) // 拷贝
    {
        data.points[i].x = cloud->points[i].x;
        data.points[i].y = cloud->points[i].y;
    }
}
