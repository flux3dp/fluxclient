#include "printer_module.h"

struct cone;
struct tri_data;
class SupportTree;
int add_support(MeshPtr input_mesh, MeshPtr out_mesh, float alpha);
int find_support_point(MeshPtr triangles, float alpha, float sample_rate, pcl::PointCloud<pcl::PointXYZ>::Ptr P);
double cone_mesh_intersect(cone a, MeshPtr triangles, std::vector<tri_data> &preprocess_tri, Eigen::Vector3f &p);
double cone_intersect(cone a, cone b, cone &c);
int genearte_strut(pcl::PointCloud<pcl::PointXYZ>::Ptr P, SupportTree &support_tree, MeshPtr &strut_stl);
Eigen::Matrix3f find_strut_tri(float R, float shrink_d, Eigen::Vector3f start, Eigen::Vector3f end);
int preprocess(MeshPtr input_mesh, std::vector<tri_data> &preprocess_tri);
