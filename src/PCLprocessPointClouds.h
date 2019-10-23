#ifndef PCLPROCESSPOINTCLOUDS_H_
#define PCLPROCESSPOINTCLOUDS_H_

#include "processPointClouds.h"

template<typename PointT>
class PCLProcessPointClouds : public ProcessPointClouds<PointT> {
public:

    PCLProcessPointClouds();

    virtual ~PCLProcessPointClouds();

    virtual std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    virtual std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
};
#endif /* PCL_PROCESSPOINTCLOUDS_H_ */