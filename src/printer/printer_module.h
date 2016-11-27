#include <stdio.h>
#include <math.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>

typedef pcl::PolygonMesh::Ptr MeshPtr;
MeshPtr createMeshPtr();

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;
CloudPtr createCloudPtr(std::vector< std::vector<float> > points);

int setCloud(MeshPtr triangles, CloudPtr cloud);
int setPoints(MeshPtr triangles, std::vector< std::vector<float> > points);
int push_backFace(MeshPtr triangles, int v0, int v1, int v2);
int add_on(MeshPtr base, MeshPtr new_mesh);
int STL_to_List(MeshPtr triangles, std::vector<std::vector< std::vector<float> > > &data);
int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float sc_x, float sc_y, float sc_z);
int bounding_box(MeshPtr triangles, std::vector<float> &b_box);
int bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<float> &b_box);
int cut(MeshPtr input_mesh, MeshPtr out_mesh, float floor_v);
int mesh_len(MeshPtr triangles);

void xnormal(std::vector< std::vector<float> > &v, float* result);
void xnormalize(float *v);
int write_stl_binary(MeshPtr triangles, const char* filename);
