#ifndef RATE_H
#define RATE_H

#include <chrono>
#include <thread>
#include <cstdio>
using namespace std;

class Rate
{
public:
    Rate(size_t freq, bool verbose = false);
    void initialize(size_t freq, bool verbose = false);
    void start();
    void sleep();

private:
    bool verbose_;
    chrono::milliseconds period_;
    chrono::high_resolution_clock::time_point last_time_;
};

#endif