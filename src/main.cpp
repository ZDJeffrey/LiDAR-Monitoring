#include "monitor.h"
#include "config.h"
#include <condition_variable>
#include <csignal>

std::condition_variable cv;
std::mutex cv_mtx;

bool exit_requested = false;

void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        std::unique_lock<std::mutex> lock(cv_mtx);
        exit_requested = true;
        cv.notify_all();
    }
}

int main()
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::unique_lock<std::mutex> lock(cv_mtx);

    Config config("/home/jeffrey/Code/LiDAR_monitoring/cfg/dynamic_parameters.yaml",
                  "/home/jeffrey/Code/LiDAR_monitoring/cfg/static_parameters.yaml");
    Monitor monitor(config);

    cv.wait(lock, []()
            { return exit_requested; });
    if (config.verbose)
        printf("Main thread terminated\n");
    return 0;
}