#ifndef ALARM_H
#define ALARM_H

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include "rate.h"
#include "config.h"

class Alarm
{
public:
    Alarm(const Config &config);
    ~Alarm();
    bool connectAlarm(const string &port);
    bool setVolume(uint volume);
    bool startAlarm();

private:
    int fd_;             // 串口文件描述符
    struct termios options_;
    uint volume_; // 音量
    bool is_on_;  // 是否报警
    bool is_end_; // 是否停止监听线程
    bool verbose_; // 是否打印调试信息
    chrono::high_resolution_clock::time_point start_time_;
    u_char volume_msg[8] = {0x01, 0x06, 0x00, 0x06, 0x00, 0x00, 0x69, 0xCB};
    u_char alarm_msg[8] = {0x01, 0x06, 0x00, 0x03, 0x00, 0x01, 0xB8, 0x0A};
    u_char volume_crc[31][2] = {{0x69,0xCB},
                                {0xA8,0x0B},
                                {0xE8,0x0A},
                                {0x29,0xCA},
                                {0x68,0x08},
                                {0xA9,0xC8},
                                {0xE9,0xC9},
                                {0x28,0x09},
                                {0x68,0x0D},
                                {0xA9,0xCD},
                                {0xE9,0xCC},
                                {0x28,0x0C},
                                {0x69,0xCE},
                                {0xA8,0x0E},
                                {0xE8,0x0F},
                                {0x29,0xCF},
                                {0x68,0x07},
                                {0xA9,0xC7},
                                {0xE9,0xC6},
                                {0x28,0x06},
                                {0x69,0xC4},
                                {0xA8,0x04},
                                {0xE8,0x05},
                                {0x29,0xC5},
                                {0x69,0xC1},
                                {0xA8,0x01},
                                {0xE8,0x00},
                                {0x29,0xC0},
                                {0x68,0x02},
                                {0xA9,0xC2},
                                {0xE9,0xC3}}; // 音量校验码
    chrono::milliseconds period_;
};

#endif