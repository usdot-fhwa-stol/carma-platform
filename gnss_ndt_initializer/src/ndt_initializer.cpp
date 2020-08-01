#include <gnss_ndt_initializer/ndt_initializer.h>

namespace ndt_initializer {
  
  boost::optional<Eigen::Matrix4f> NDTInitializer::computeBestTransform(const Eigen::Matrix4f& initial_guess) {
    // TODO
  }

  void NDTInitializer::setConfig(const NDTConfig& config) {
    ndt_solver_.setResolution(config.resolution);
    ndt_solver_.setMaximumIterations(config.max_iter);
    ndt_solver_.setStepSize(config.step_size);
    ndt_solver_.setTransformationEpsilon(config.trans_eps);
  }

  void NDTInitializer::setMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map) {
    ndt_solver_.setInputTarget(map);
  }

  void NDTInitializer::setScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan) {
    ndt_solver_.setInputSource(scan);
  }

  // TODO setters for optimization

}