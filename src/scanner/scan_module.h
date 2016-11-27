#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Core>


// noise del
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

PointCloudXYZRGBPtr createPointCloudXYZRGB();
void push_backPoint(PointCloudXYZRGBPtr cloud, float x, float y, float z, uint32_t r, uint32_t g, uint32_t b);
int loadPointCloudXYZRGB(const char* file, PointCloudXYZRGBPtr cloud);
void dumpPointCloudXYZRGB(const char* file, PointCloudXYZRGBPtr cloud);
int get_item(PointCloudXYZRGBPtr cloud, int key, std::vector<float> &point);
size_t get_w(PointCloudXYZRGBPtr cloud);

int SOR(PointCloudXYZRGBPtr cloud, int neighbors, float threshold);
int Euclidean_Cluster(PointCloudXYZRGBPtr cloud, float thres_dist, std::vector< std::vector<int> > &output);

//normal estimation
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointXYZRGBNormalPtr;

NormalPtr createNormalPtr();
PointXYZRGBNormalPtr createPointXYZRGBNormalPtr();
int ne(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius);
int ne_viewpoint(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius);
PointXYZRGBNormalPtr concatenatePointsNormal(PointCloudXYZRGBPtr cloud, NormalPtr normals);

// registration
// Types
typedef pcl::PointXYZRGBNormal PointNT; // float x, y, z; float normal[3], curvature, rgb
// typedef pcl::PointCloud<PointNT> PointCloudT;
// typedef pcl::PointCloud<PointNT>::Ptr PointCloudTPtr;
typedef pcl::FPFHSignature33 FeatureT;

typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::PointCloud<FeatureT>::Ptr FeatureCloudTPtr;
typedef Eigen::Matrix4f M4f;

// PointCloudTPtr createPointCloudPointNormal();
int loadPointNT(const char* file, PointXYZRGBNormalPtr cloud);
void dumpPointNT(const char* file, PointXYZRGBNormalPtr cloud);

int downsample(PointXYZRGBNormalPtr cloud, PointXYZRGBNormalPtr cloud_clone, float leaf);
FeatureCloudTPtr createFeatureCloudTPtr();
int FE(PointXYZRGBNormalPtr cloud, FeatureCloudTPtr cloud_features, float radius);
// int SCP(PointXYZRGBNormalPtr object, FeatureCloudTPtr object_features, PointXYZRGBNormalPtr scene, FeatureCloudTPtr scene_features, M4f &transformation, float leaf);
int SCP(PointXYZRGBNormalPtr object, FeatureCloudTPtr object_features, PointXYZRGBNormalPtr scene, FeatureCloudTPtr scene_features, PointXYZRGBNormalPtr object_aligned, float leaf);

typedef pcl::PolygonMesh::Ptr MeshPtr;
MeshPtr createMeshPtr();
int POS(PointXYZRGBNormalPtr cloud_with_normals, MeshPtr triangles, PointCloudXYZRGBPtr cloud, float smooth);
int GPT(PointXYZRGBNormalPtr cloud_with_normals, MeshPtr triangles, PointCloudXYZRGBPtr cloud);
int STL_to_List(MeshPtr triangles, std::vector<std::vector< std::vector<float> > > &data);
// int STL_to_Faces(MeshPtr triangles, std::vector< std::vector<int> > &data);
int apply_transform(PointCloudXYZRGBPtr cloud, NormalPtr normals, PointXYZRGBNormalPtr both, float x, float y, float z, float rx, float ry, float rz);

int clone(PointCloudXYZRGBPtr obj, PointCloudXYZRGBPtr obj2);
int clone(NormalPtr normalObj, NormalPtr normalObj2);
int clone(PointXYZRGBNormalPtr bothobj, PointXYZRGBNormalPtr bothobj2);
int clone(MeshPtr meshobj, MeshPtr meshobj2);

PointCloudXYZRGBPtr add(PointCloudXYZRGBPtr obj, PointCloudXYZRGBPtr obj2);
NormalPtr add(NormalPtr normalObj, NormalPtr normalObj2);
PointXYZRGBNormalPtr add(PointXYZRGBNormalPtr bothobj, PointXYZRGBNormalPtr bothobj2);

int split(PointXYZRGBNormalPtr bothobj, PointCloudXYZRGBPtr obj, NormalPtr normalObj);
int cut(PointCloudXYZRGBPtr input, PointCloudXYZRGBPtr output, int mode, int direction, float value);
