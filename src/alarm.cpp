#include "alarm.h"

Alarm::Alarm(const Config &config)
{
    volume_ = config.alarm_volume;
    is_on_ = false;
    is_end_ = false;
    verbose_ = config.verbose;
}

Alarm::~Alarm()
{
    if (fd_ != -1)
        close(fd_);
}

bool Alarm::connectAlarm(const string &port)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ == -1)
    {
        std::cerr << "Error: Unable to open serial port" << std::endl;
        return false;
    }

    memset(&options_, 0, sizeof(options_));
    cfsetispeed(&options_, B9600);
    cfsetospeed(&options_, B9600);
    options_.c_cflag |= (CLOCAL | CREAD);
    options_.c_cflag &= ~PARENB;
    options_.c_cflag &= ~CSTOPB;
    options_.c_cflag |= CS8;
    options_.c_cc[VMIN] = 1;
    options_.c_cc[VTIME] = 1;
    tcflush(fd_, TCIOFLUSH);
    tcsetattr(fd_, TCSANOW, &options_);
    fcntl(fd_, F_SETFL, 0); // 阻塞模式

    uint tmp_volume = volume_;
    volume_ = 31; // 确保能够发送音量设置信息
    return setVolume(tmp_volume);
}

bool Alarm::setVolume(uint volume)
{
    volume = std::min(volume, (uint)30);
    if (volume_ == volume) // 音量相同不发送
        return true;;

    volume_ = volume;
    volume_msg[5] = (u_char)volume_; // 设置音量
    // 设置校验码
    volume_msg[6] = volume_crc[volume_][0];
    volume_msg[7] = volume_crc[volume_][1];
    if(write(fd_, volume_msg, 8) == -1) // 发送音量设置命令
    {
        if(verbose_)
            printf("Unable to set volume\n");
        return false;
    }
    return true;
}

bool Alarm::startAlarm()
{
    if (fd_ == -1)
        return false;
    if (is_on_) // 检查是否播放完毕
    {
        auto elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - start_time_); // 间隔时间
        if (elapsed_time >= period_)
            is_on_ = false;
    }

    if (!is_on_)
    {
        if(write(fd_, alarm_msg, 8)==-1)
        {
            if(verbose_)
                printf("Unable to alarm\n");
            return false;
        }
        start_time_ = chrono::high_resolution_clock::now();
        is_on_ = true;
        period_ = chrono::milliseconds(2000);
    }
    return true;
}