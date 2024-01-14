#include "monitor.h"

void Monitor::updateConfig()
{
    alarm_->setVolume(config_->alarm_volume);
    analysis_->updateConfig(*config_);
    if (proc_freq_ != config_->proc_freq || scan_->mode_id_ != config_->mode_id)
    {
        proc_freq_ = config_->proc_freq;
        scan_->mode_id_ = config_->mode_id;
        stop();
        start();
    }
    else
    {
        proc_freq_ = config_->proc_freq;
        scan_->mode_id_ = config_->mode_id;
    }
}

void Monitor::scan()
{
    vector<Node_t> buffer;
    int i = 0;
    while (true)
    {
        {
            lock_guard<mutex> lock(flag_mtx_);
            if (!is_scanning_)
                break;
        }
        if (scan_->scan(buffer, 1))
        {
            lock_guard<mutex> lock(data_mtx_);
            scan_data_[i].clear();
            scan_data_[i].swap(buffer); // 获取buffer内容
            is_new_scan_ = true;
            i = (i + 1) % scan_num_;
        }
    }
}

bool Monitor::getPointcloud()
{
    lock_guard<mutex> lock(data_mtx_);
    if (!is_new_scan_) // 未获取新的扫描内容
        return false;
    is_new_scan_ = false;
    size_t total_size = 0;
    for (int i = 0; i < scan_num_; i++)
        total_size += scan_data_[i].size();
    nodes_.clear();
    nodes_.reserve(total_size);
    for (int i = 0; i < scan_num_; i++) // 合并scan_data_数据
        nodes_.insert(nodes_.end(), scan_data_[i].begin(), scan_data_[i].end());
    return true;
}

void Monitor::processPointcloud()
{
    Rate rate(proc_freq_, verbose_); // 定时器
    while (true)
    {
        {
            lock_guard<mutex> lock(flag_mtx_);
            if (!is_scanning_)
            {
                scan_->stop();
                break;
            }
            if (!is_initialized_)
            {
                if (scan_->scan(nodes_, initialize_scan_num_)) // 初始化成功
                {
                    analysis_->setStaticCloud(nodes_);
                    is_initialized_ = true;
                }
                else // 初始化失败
                    continue;
            }
        }
        if (getPointcloud())
        {
            bool is_alarm = analysis_->setDynamicCloud(nodes_);
            if (is_alarm)
                alarm_->startAlarm();
        }
        else if (verbose_)
            printf("Did not get new scan data.\n");
        rate.sleep();
    }
}

Monitor::Monitor(Config &config) : is_initialized_(false),
                                   is_scanning_(false),
                                   is_new_scan_(false),
                                   be_visualized_(config.be_visualized),
                                   verbose_(config.verbose),
                                   initialize_scan_num_(config.initialize_scan_num),
                                   scan_num_(config.scan_num),
                                   scan_data_(config.scan_num),
                                   proc_freq_(config.proc_freq)
{
    // 配置文件
    config_ = &config;

    // 启动报警器
    alarm_ = new Alarm(config);
    if (!alarm_->connectAlarm(config.alarm_port_name))
    {
        if (verbose_)
            printf("Alarm create error\n");
        exit(-1);
    }

    // 启动激光雷达
    scan_ = new Scan(config);
    if (!scan_->createDriver()) // 创建驱动
    {
        if (verbose_)
            printf("LiDAR create error!\n");
        exit(-1);
    }

    // 启动数据分析模块
    analysis_ = new Analysis(config);

    // 可视化模块
    if (be_visualized_)
    {
        visualization_ = new Visualization(config);
        visualization_->setMonitor(this);
        if (!visualization_->createServer())
        {
            if (verbose_)
                printf("Visualization server error!\n");
            exit(-1);
        }
    }

    start(); // 启动扫描
}

Monitor::~Monitor()
{
    stop();
    if (alarm_)
        delete alarm_;
    if (scan_)
        delete scan_;
    if (analysis_)
        delete analysis_;
    if (visualization_)
        delete visualization_;
}

void Monitor::start()
{
    {
        lock_guard<mutex> lock(flag_mtx_);
        if (is_scanning_)
            return;
        is_scanning_ = true;
    }

    scan_->start();                                          // 开始扫描
    scan_thread_ = thread(&Monitor::scan, this);             // 扫描线程
    set_thread_ = thread(&Monitor::processPointcloud, this); // 处理线程
}

void Monitor::stop()
{
    {
        lock_guard<mutex> lock(flag_mtx_);
        if (!is_scanning_)
            return;
        is_scanning_ = false;
    }
    if (set_thread_.joinable())
        set_thread_.join();
    if (scan_thread_.joinable())
        scan_thread_.join();
}

void Monitor::initialize()
{
    is_initialized_ = false;
}

void Monitor::updatePointcloud(UpdateDataType &type, PointCloudPtr &static_cloud, PointCloudPtr &safe_cloud, PointCloudPtr &warning_cloud)
{
    analysis_->updatePointcloud(type, static_cloud, safe_cloud, warning_cloud);
}
