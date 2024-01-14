#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <cmath>
#include <cstdio>
#include <vector>
#include <mutex>
#include <limits>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/extract_clusters.h>

#include "scan.h"
#include "config.h"
#include "rate.h"
#include "socket_type.h"

using namespace pcl;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

class Analysis
{
public:
    Analysis(const Config &config);
    ~Analysis();
    void setStaticCloud(const vector<Node_t> &nodes);
    bool setDynamicCloud(const vector<Node_t> &nodes);
    void updatePointcloud(UpdateDataType &type, PointCloudPtr &static_cloud, PointCloudPtr &safe_cloud, PointCloudPtr &warning_cloud);
    void updateConfig(const Config &config);
private:
    void nodes2Cloud(const vector<Node_t> &nodes, PointCloud<PointXYZ>::Ptr cloud);

private:
    // 变量
    PointCloudPtr static_cloud_, static_buffer_;              // 静态点云
    PointCloudPtr safe_cloud_, safe_buffer_;                  // 安全点云
    PointCloudPtr warning_cloud_, warning_buffer_;            // 警告点云
    RadiusOutlierRemoval<PointXYZ> static_sor_, dynamic_sor_; // 半径滤波器
    SegmentDifferences<PointXYZ> warning_seg_, safe_seg_;     // 点云差分
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;        // 点云聚类
    mutex update_mtx_;
    UpdateDataType update_type_;

    // 参数
    double max_dist_;           // 最大距离
    double sqr_dist_threshold_; // 偏移距离阈值平方
    size_t num_threshold_;      // 偏离点数量阈值
    bool verbose_;              // 是否输出详细信息
};

#endif