#include "scan.h"
#include "sdkcommon.h"

Scan::Scan(const Config &config)
{
    driver_ = NULL;
    capacity_ = config.buffer_capacity;
    buffer = new Node_t[capacity_];
    port_name_ = config.lidar_port_name;
    baud_rate_ = config.lidar_baud_rate;
    mode_id_ = config.mode_id;
}

Scan::~Scan()
{
    delete buffer;
    stop();
    deleteDriver();
}

bool Scan::createDriver()
{
    // 创建驱动
    driver_ = *createLidarDriver();
    if (!driver_)
        return false;

    // 使用串口连接
    sl_lidar_response_device_info_t dev_info;
    bool connected = false;
    IChannel *channel = *createSerialPortChannel(port_name_, baud_rate_);
    if (SL_IS_OK(driver_->connect(channel)))
    {
        result_ = driver_->getDeviceInfo(dev_info);
        if (SL_IS_OK(result_)) // 连接成功
            connected = true;
    }

    if (!connected) // 连接失败
    {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s, baud rate: %zu.\n",
                port_name_.c_str(), baud_rate_);
        deleteDriver();
        return false;
    }

    // 输出雷达设备信息
    printf("LiDAR S/N: ");
    for (int i = 0; i < sizeof(dev_info.serialnum) / sizeof(dev_info.serialnum[0]); i++)
        printf("%02X", dev_info.serialnum[i]);
    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n",
           dev_info.firmware_version >> 8,
           dev_info.firmware_version & 0xff,
           (int)dev_info.hardware_version);

    // 检查设备状态
    _sl_lidar_response_device_health_t health_info;
    result_ = driver_->getHealth(health_info);
    if (SL_IS_OK(result_))
    {
        switch (health_info.status)
        {
        case SL_LIDAR_STATUS_OK:
            printf("LiDAR health status: OK\n");
            break;
        case SL_LIDAR_STATUS_WARNING:
            printf("LiDAR health status: WARNING\n");
            break;
        case SL_LIDAR_STATUS_ERROR:
            fprintf(stderr, "Error, LiDAR internal error detected. Please reboot the device to retry.");
            deleteDriver();
            return false;
        }
    }
    else
    {
        fprintf(stderr, "Error, cannot retrieve the LiDAR health code: %x\n", result_);
        deleteDriver();
        return false;
    }
    return true;
}

void Scan::deleteDriver()
{
    if (driver_)
    {
        delete driver_;
        driver_ = NULL;
    }
}

void Scan::start()
{
    if (driver_->isConnected())
        driver_->startScanExpress(false, mode_id_);
}

bool Scan::scan(vector<Node_t> &nodes, size_t num)
{
    size_t size = 0, count;
    for (size_t i = 0; i < num; i++)
    {
        if (size >= capacity_)
            break;
        count = capacity_ - size;
        result_ = driver_->grabScanDataHq(buffer + size, count);
        if (!SL_IS_OK(result_))
            return false;
        size += count;
    }
    nodes = vector<Node_t>(buffer, buffer + size);
    return true;
}

void Scan::stop()
{
    driver_->stop();
    driver_->setMotorSpeed(0);
}

void Scan::updateConfig(const Config &config)
{
    mode_id_ = config.mode_id;
}
