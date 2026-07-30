#include "erasor2/RerunLogger.hpp"

#include <atomic>

namespace erasor2::viz {
namespace {
std::atomic<bool> g_enabled{false};
}

void init(const std::string&, bool, const std::string&) {
  // Built without Rerun support. Keep visualization disabled.
  g_enabled.store(false);
}

void shutdown() { g_enabled.store(false); }

void setEnabled(bool enabled) {
  // Logging cannot be enabled when the SDK is not part of this build.
  (void)enabled;
  g_enabled.store(false);
}

bool isEnabled() { return false; }

void setFrame(int64_t) {}

void logCloud(std::string_view,
              const pcl::PointCloud<pcl::PointXYZI>&,
              const std::vector<std::array<float, 3>>&) {}

void logCloud(std::string_view, const pcl::PointCloud<pcl::PointXYZRGB>&) {}

void logPath(std::string_view, const std::vector<Eigen::Matrix4f>&) {}

void logPose(std::string_view, const Eigen::Matrix4f&) {}

void logGridMapLayer(std::string_view, const erasor2::GridMap&, const std::string&) {}

void logTextAt(std::string_view,
               const Eigen::Vector3f&,
               const std::string&,
               const std::array<float, 3>&) {}

void logCircle(std::string_view, const Eigen::Vector3f&, float, int) {}

void TextArrayPublisher::publishScores(
    const std::vector<std::pair<Eigen::Matrix<float, 4, 1>, float>>&,
    const std::array<float, 3>&) const {}

}  // namespace erasor2::viz
