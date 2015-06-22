#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGB;

PointCloudXYZRGB createPointCloudXYZRGB();
void push_backPoint(PointCloudXYZRGB cloud, float x, float y, float z, uint32_t rgb);
int loadPointCloudXYZRGB(const char* file, PointCloudXYZRGB cloud);
void dumpPointCloudXYZRGB(const char* file,
                          PointCloudXYZRGB cloud);

int SOR(PointCloudXYZRGB cloud, int neighbors, float threshold);
int VG(PointCloudXYZRGB cloud);
