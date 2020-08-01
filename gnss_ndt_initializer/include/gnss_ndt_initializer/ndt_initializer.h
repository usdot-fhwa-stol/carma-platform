#pragma once
#include <string>
#include <iostream>
#include <boost/optional.hpp>
#include <pcl/registration/ndt.h>
#include "ndt_config.h"

namespace ndt_initializer {
class NDTInitializer {

  private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_solver_;

  public:
  
    /**
     * \brief Uses NDT to determine the optimal transform to use to minimize the fitness score of the NDT result. 
     *        If the fitness score could not be reduced below the threshold then this will return empty. 
     * 
     * \param initial_guess The initial guess to use for the minimization
     * \return An optional transform. If the transform is set it is guaranteed to generate a fitness score less than the threshold.
     * 
     */ 
    boost::optional<Eigen::Matrix4f> computeBestTransform(const Eigen::Matrix4f& initial_guess);

    /**
     * \brief Sets the parameters of the internal NDT solver used for minimization.
     * 
     * \param config The config to set
     */ 
    void setConfig(const NDTConfig& config);

    /**
     * \brief Sets the base map to use for optimization
     * 
     * \param map The map to set
     */ 
    void setMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map);

    /**
     * \brief Sets the scan to match agains the base map
     * 
     * \param The scan to match
     */ 
    void setScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan);

    // TODO setters for optimization

};
}