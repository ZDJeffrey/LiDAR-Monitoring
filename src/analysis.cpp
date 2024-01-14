#include "analysis.h"

void Analysis::nodes2Cloud(const vector<Node_t> &nodes, PointCloud<PointXYZ>::Ptr cloud)
{
    cloud->clear();
    cloud->reserve(nodes.size());
    for (const auto &node : nodes)
    {
        double dist = getDist(node);
        if (dist <= 0 || dist > max_dist_)
            continue;
        double angle = getAngleRad(node);
        cloud->push_back(PointXYZ(dist * cos(angle), dist * sin(angle), 0.0f));
    }
}

Analysis::Analysis(const Config &config) : static_cloud_(new PointCloud<PointXYZ>),
                                           safe_cloud_(new PointCloud<PointXYZ>),
                                           warning_cloud_(new PointCloud<PointXYZ>),
                                           static_buffer_(new PointCloud<PointXYZ>),
                                           safe_buffer_(new PointCloud<PointXYZ>),
                                           warning_buffer_(new PointCloud<PointXYZ>)
{
    // 设置阈值
    max_dist_ = config.max_dist;
    sqr_dist_threshold_ = pow(config.dist_threshold, 2);
    num_threshold_ = config.num_threshold;

    // 设置滤波器
    static_sor_.setRadiusSearch(config.static_radius);
    static_sor_.setMinNeighborsInRadius(config.static_min_neighbor);
    dynamic_sor_.setRadiusSearch(config.dynamic_radius);
    dynamic_sor_.setMinNeighborsInRadius(config.dynamic_min_neighbor);

    // 设置差分器
    warning_seg_.setDistanceThreshold(sqr_dist_threshold_);
    safe_seg_.setDistanceThreshold(0);

    // 设置聚类器
    ec_.setClusterTolerance(config.cluster_tolerance);
    ec_.setMinClusterSize(config.min_cluster_size);
    ec_.setMaxClusterSize(config.max_cluster_size);

    // 其他参数
    verbose_ = config.verbose;
}

Analysis::~Analysis()
{
    update_type_.static_update = false;
    update_type_.safe_update = false;
    update_type_.warning_update = false;
}

void Analysis::setStaticCloud(const vector<Node_t> &nodes)
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    nodes2Cloud(nodes, cloud);
    static_sor_.setInputCloud(cloud);   // 设置滤波器输入
    static_sor_.filter(*static_cloud_); // 半径滤波
    {
        lock_guard<mutex> lock(update_mtx_);
        pcl::copyPointCloud(*static_cloud_, *static_buffer_); // 深拷贝
        update_type_.static_update = true;
    }
}

bool Analysis::setDynamicCloud(const vector<Node_t> &nodes)
{
    warning_cloud_->clear();
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    nodes2Cloud(nodes, cloud);
    // 差分
    PointCloud<PointXYZ>::Ptr tmp_cloud(new PointCloud<PointXYZ>);
    warning_seg_.setTargetCloud(static_cloud_); // 设置相减目标
    warning_seg_.setInputCloud(cloud);          // 设置相减输入
    warning_seg_.segment(*tmp_cloud);           // 获得未滤波的报警点云

    bool is_alarm = false;
    size_t count = 0; // 报警数量

    if (tmp_cloud->size() == 0) // 无报警点云
    {
        safe_cloud_.swap(cloud);
        warning_cloud_->clear();
    }
    else
    {
        // 获取安全点云
        safe_seg_.setTargetCloud(tmp_cloud);
        safe_seg_.setInputCloud(cloud);
        safe_seg_.segment(*safe_cloud_);
        // 警告点云滤波
        dynamic_sor_.setInputCloud(tmp_cloud);
        dynamic_sor_.filter(*warning_cloud_);

        if (warning_cloud_->size() < num_threshold_) // 数量不足以报警
            warning_cloud_->clear();
        else
        {
            tmp_cloud->clear();
            // 点云聚类
            std::vector<pcl::PointIndices> cluster_indices; // 聚类结果
            ec_.setInputCloud(warning_cloud_);
            ec_.extract(cluster_indices);

            std::vector<pcl::PointXYZ> bounding_box; // 边界框
            // 遍历聚类结果，添加到警告点云中，并添加间隔
            for (auto indices : cluster_indices)
            {
                count += indices.indices.size();
                if (indices.indices.size() >= num_threshold_)
                    is_alarm = true; // 进行报警
                // 边界坐标
                float min_x = std::numeric_limits<float>::max(),
                      max_x = -std::numeric_limits<float>::max(),
                      min_y = std::numeric_limits<float>::max(),
                      max_y = -std::numeric_limits<float>::max();
                for (auto index : indices.indices) // 添加至可视化报警点云中
                {
                    pcl::PointXYZ p = warning_cloud_->points[index]; // 坐标查询
                    tmp_cloud->push_back(p);                         // 插入点云
                    // 边界坐标更新
                    if (p.x < min_x)
                        min_x = p.x;
                    if (p.x > max_x)
                        max_x = p.x;
                    if (p.y < min_y)
                        min_y = p.y;
                    if (p.y > max_y)
                        max_y = p.y;
                }
                // 边界框坐标添加
                bounding_box.push_back(pcl::PointXYZ(min_x-0.05f, min_y-0.05f, 0));
                bounding_box.push_back(pcl::PointXYZ(max_x+0.05f, max_y+0.0f, 0));
            }
            warning_cloud_.swap(tmp_cloud);                                                                          // 交换点云
            warning_cloud_->insert(warning_cloud_->end(), bounding_box.begin(), bounding_box.end());                 // 添加边界框坐标
            warning_cloud_->push_back(pcl::PointXYZ(bounding_box.size(), bounding_box.size(), bounding_box.size())); // 标识边界框坐标数量
        }
    }
    {
        lock_guard<mutex> lock(update_mtx_);
        if (safe_cloud_->size() > 0)
        {
            safe_buffer_.swap(safe_cloud_);
            update_type_.safe_update = true;
        }
        if (warning_cloud_->size() > 0)
        {
            warning_buffer_.swap(warning_cloud_);
            update_type_.warning_update = true;
        }
    }

    if (verbose_)
        printf("warning point count:%zu. %s\n", count, count > num_threshold_ ? "Alarm!!!" : "");

    return is_alarm;
}

void Analysis::updatePointcloud(UpdateDataType &type, PointCloudPtr &static_cloud, PointCloudPtr &safe_cloud, PointCloudPtr &warning_cloud)
{
    lock_guard<mutex> lock(update_mtx_);
    type = update_type_;
    update_type_.static_update = update_type_.safe_update = update_type_.warning_update = false;
    if (type.static_update)
        static_cloud.swap(static_buffer_);
    if (type.safe_update)
        safe_cloud.swap(safe_buffer_);
    if (type.warning_update)
        warning_cloud.swap(warning_buffer_);
}

void Analysis::updateConfig(const Config & config)
{
    max_dist_ = config.max_dist;
    sqr_dist_threshold_ = pow(config.dist_threshold, 2);
    num_threshold_ = config.num_threshold;
    warning_seg_.setDistanceThreshold(sqr_dist_threshold_);
}
