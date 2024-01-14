#include "rate.h"

Rate::Rate(size_t freq, bool verbose) : period_(1000 / freq), verbose_(verbose)
{
    last_time_ = chrono::high_resolution_clock::now();
}

void Rate::initialize(size_t freq, bool verbose)
{
    period_ = chrono::milliseconds(1000 / freq);
    verbose_ = verbose;
    last_time_ = chrono::high_resolution_clock::now();
}

void Rate::start()
{
    last_time_ = chrono::high_resolution_clock::now();
}

void Rate::sleep()
{
    auto now = chrono::high_resolution_clock::now();
    auto elapsed_time = chrono::duration_cast<chrono::milliseconds>(now - last_time_);
    if (elapsed_time < period_)
        this_thread::sleep_for(period_ - elapsed_time);
    else if (verbose_)
        printf("time out for %ld ms\n", (elapsed_time - period_).count());
    last_time_ = chrono::high_resolution_clock::now();
}
