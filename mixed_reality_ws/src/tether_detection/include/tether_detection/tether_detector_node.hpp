#include "tether_detection/line_extraction.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <rclcpp/rclcpp.hpp>

namespace tether_detection
{

class LineExtractor
{
public:
  explicit LineExtractor(const DetectionConfig& cfg)
  : config_(cfg) {}

  // Extract line segments from point cloud (simplified for demo)
  std::vector<LineSegment> extract_lines(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);

private:
  DetectionConfig config_;

  // Helper: find points aligned in a line (RANSAC-based)
  std::vector<LineSegment> find_line_candidates(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

}  // namespace tether_detection
