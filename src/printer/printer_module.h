#include <vector>

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/vtk_lib_io.h>

typedef pcl::PolygonMesh::Ptr MeshPtr;
MeshPtr createMeshPtr();

int set_point(MeshPtr triangles, std::vector< std::vector<float> > points);
int push_backFace(MeshPtr triangles, int v0, int v1, int v2);
int add_on(MeshPtr base, MeshPtr new_mesh);
int STL_to_List(MeshPtr triangles, std::vector<std::vector< std::vector<float> > > &data);
int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float sc_x, float sc_y, float sc_z);
int bounding_box(MeshPtr triangles, std::vector<float> &b_box);
int bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<float> &b_box);
int cut(MeshPtr input_mesh, MeshPtr out_mesh, float floor_v);

int add_support(MeshPtr input_mesh, MeshPtr out_mesh);
int find_support_point(MeshPtr triangles, float alpha, float sample_rate, pcl::PointCloud<pcl::PointXYZ>::Ptr P);
struct cone;
double cone_intersect(cone a, cone b, cone &c);
