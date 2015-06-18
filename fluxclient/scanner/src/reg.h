#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh_omp.h>
#include <Eigen/Core>


// Types
typedef pcl::PointNormal PointNT; // float x, y, z; float normal[3], curvature
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<PointNT>::Ptr PointCloudTPtr;
typedef pcl::FPFHSignature33 FeatureT;

typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::PointCloud<FeatureT>::Ptr FeatureCloudTPtr;

PointCloudTPtr createPointCloudPointNormal();
int loadPointCloudPointNormal(const char* file, PointCloudTPtr cloud);
void dumpPointCloudPointNormal(const char* file, PointCloudTPtr cloud);

int downsample(PointCloudTPtr object, float leaf);
int NE_OMP(PointCloudTPtr object,float radius);
int FE(PointCloudTPtr object, FeatureCloudTPtr object_features, float radius);
int SCP();
