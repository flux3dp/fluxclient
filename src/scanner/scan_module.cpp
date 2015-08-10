#include <iostream>
#include <limits>

#include "scan_module.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/surface/poisson.h>
#include <pcl/conversions.h>

#include <pcl/console/print.h>

PointCloudXYZRGBPtr createPointCloudXYZRGB() {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (
    new pcl::PointCloud<pcl::PointXYZRGB>);
  return cloud;
}

int loadPointCloudXYZRGB(const char* file, PointCloudXYZRGBPtr cloud) {
  return pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud);
}

void dumpPointCloudXYZRGB(const char* file, PointCloudXYZRGBPtr cloud) {
  pcl::io::savePCDFileASCII (file, *cloud);
}

int get_item(PointCloudXYZRGBPtr cloud, int key, std::vector<float> &point){
  pcl::PointXYZRGB p = (*cloud)[key];

  point[0] = p.x;
  point[1] = p.y;
  point[2] = p.z;
  point[3] = (uint32_t(p.rgb) >> 16) & 0x0000ff;
  point[4] = (uint32_t(p.rgb) >> 8) & 0x0000ff;
  point[5] = (uint32_t(p.rgb)) & 0x0000ff;

  return 1;
}

int get_w(PointCloudXYZRGBPtr cloud){
  return (*cloud).size();
}

void push_backPoint(PointCloudXYZRGBPtr cloud, float x, float y, float z, uint32_t rgb){
  pcl::PointXYZRGB p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.rgb = rgb;
  cloud -> push_back(p);
}

int SOR(PointCloudXYZRGBPtr cloud, int neighbors, float threshold) {
    // statistical_outlier_removal

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setMeanK (neighbors);  //number of neighbors
  sor.setStddevMulThresh (threshold); //how many deviation out of n
  sor.setInputCloud (cloud);
  sor.filter (*cloud);

  return 0;
}

NormalPtr createNormalPtr(){
  NormalPtr normals (new pcl::PointCloud<pcl::Normal>);
  return normals;
}

PointXYZRGBNormalPtr createPointXYZRGBNormalPtr(){
  PointXYZRGBNormalPtr all (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  return all;
}

int ne(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius){
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> nest;
  nest.setNumberOfThreads(4);
  nest.setRadiusSearch (radius);
 // nest.setViewPoint(0.0, 170.0, 90);
  nest.setInputCloud (cloud);
  nest.compute (*normals);
  return 1;
}

inline int check(std::vector<float> normal, std::vector<float> position_v){
  if (normal[0] * position_v[0] + normal[1] * position_v[1] + normal[1] * position_v[1] > 0)
    return 1;
  else
    return 0;
}

int ne_viewpoint(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius,  std::vector<std::vector<float> >viewp, std::vector<int> step){
// int ne_viewpoint(PointCloudXYZRGBPtr cloud, NormalPtr normals, std::vector< float > viewp, std::vector<int> step){
// int ne_viewpoint(PointCloudXYZRGBPtr cloud, NormalPtr normals, std::vector<int> step){

  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> nest;
  nest.setNumberOfThreads(4);
  nest.setRadiusSearch (radius);
  nest.compute (*normals);

  std::vector<float> normal(3, 0);
  std::vector<float> position_v(3, 0);

  for (uint vp = 0; vp < viewp.size(); vp += 1){
    for (uint i = 0; i < step.size() - 1; i += 1){
      normal[0]  = (*normals).points[i].normal_x;
      normal[1]  = (*normals).points[i].normal_y;
      normal[2]  = (*normals).points[i].normal_z;

      position_v[0] = viewp[vp][0] - cloud->points[i].x;
      position_v[1] = viewp[vp][1] - cloud->points[i].y;
      position_v[2] = viewp[vp][2] - cloud->points[i].z;
      // position_v[0] = viewp[vp + 0] - cloud->points[i].x;
      // position_v[1] = viewp[vp + 1] - cloud->points[i].y;
      // position_v[2] = viewp[vp + 2] - cloud->points[i].z;

      if (check(normal,position_v)){
        continue;
      }
      else{
        (*normals).points[i].normal_x *= -1;
        (*normals).points[i].normal_y *= -1;
        (*normals).points[i].normal_z *= -1;
      }
    }
  }
  return 1;
}

PointXYZRGBNormalPtr concatenatePointsNormal(PointCloudXYZRGBPtr cloud, NormalPtr normals){
  PointXYZRGBNormalPtr final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (*cloud, *normals, *final);
  return final;
}

int loadPointNT(const char* file, PointXYZRGBNormalPtr cloud){
  return pcl::io::loadPCDFile<PointNT> (file, *cloud);
}

void dumpPointNT(const char* file, PointXYZRGBNormalPtr cloud){
  pcl::io::savePCDFileASCII(file, *cloud);
}

int downsample(PointXYZRGBNormalPtr cloud, PointXYZRGBNormalPtr cloud_clone, float leaf){

  pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(cloud);
  grid.filter(*cloud_clone);

  return 0;
}

FeatureCloudTPtr createFeatureCloudTPtr(){
  FeatureCloudTPtr obj_f (new FeatureCloudT);
  return obj_f;
}

int FE(PointXYZRGBNormalPtr cloud, FeatureCloudTPtr cloud_features, float radius){
  FeatureEstimationT fest;
  fest.setRadiusSearch (10);
  fest.setInputCloud (cloud);
  fest.setInputNormals (cloud);
  fest.compute (*cloud_features);
  return 0;
}

int SCP(PointXYZRGBNormalPtr scene, FeatureCloudTPtr scene_features, PointXYZRGBNormalPtr object, FeatureCloudTPtr object_features, PointXYZRGBNormalPtr aligned, float leaf){
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;

  PointXYZRGBNormalPtr scene_clone(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  PointXYZRGBNormalPtr object_clone(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Down sampling
  downsample(scene, scene_clone, leaf);
  downsample(object, object_clone, leaf);

  pcl::console::print_info (" before-> cloud size:%d, cloud2 size:%d \n",object->points.size(), scene->points.size());
  pcl::console::print_info (" after-> cloud size:%d, cloud2 size:%d \n",object_clone->points.size(), scene_clone->points.size());

  FE(scene_clone, scene_features, 10);
  FE(object_clone, object_features, 10);

  align.setInputTarget(scene_clone);
  align.setTargetFeatures(scene_features);
  align.setInputSource(object_clone);
  align.setSourceFeatures(object_features);
  align.setMaximumIterations(1500); // Number of RANSAC iterations
  align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(10); // Number of nearest features to use
  align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(1.5f * leaf); // Inlier threshold
  align.setInlierFraction(0.3f); // Required inlier fraction for accepting a pose hypothesis
  align.align (*aligned);

  M4f transformation = align.getFinalTransformation();
  // M4f transformation = Eigen::Matrix4f::Identity();
  std::cout<< "1:"<<aligned->points.size() << std::endl;
  std::cout<< "0810:" << std::endl;

  pcl::transformPointCloud (*object, *aligned, transformation);
  ////////
  pcl::PointXYZRGBNormal p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;
  p.rgb = 0.0;
  aligned->push_back(p);
  ////////
  std::cout<< "2:"<<aligned->points.size() << std::endl;
  return align.hasConverged();
}

int loadPointCloudPointNormal(const char* file, PointXYZRGBNormalPtr cloud) {
  return pcl::io::loadPCDFile<PointNT> (file, *cloud);
}

void dumpPointCloudPointNormal(const char* file, PointXYZRGBNormalPtr cloud) {
  pcl::io::savePCDFileASCII(file, *cloud);
}

MeshPtr createMeshPtr(){
  MeshPtr mesh(new pcl::PolygonMesh);
  return mesh;
}

PointCloudXYZRGBPtr POS(PointXYZRGBNormalPtr cloud_with_normals, MeshPtr triangles){
  puts("poisson computing");

  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;

  // poisson.setConfidence(true);
  // poisson.setScale(1.0); // from 1.1 to 1.0
  poisson.setDepth (8);
  poisson.setIsoDivide(7);
  poisson.setInputCloud (cloud_with_normals);
  poisson.performReconstruction (*triangles);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  fromPCLPointCloud2(triangles->cloud, *cloud);
  return cloud;
}

int STL_to_List(MeshPtr triangles, std::vector<std::vector< std::vector<float> > > &data){
  // point's data
  // data =[
  //           t1[p1[x, y, z], p2[x, y, z], p3[x, y, z]],
  //           t2[p1[x, y, z], p2[x, y, z], p3[x, y, z]],
  //           t3[p1[x, y, z], p2[x, y, z], p3[x, y, z]],
  //             ...
  //       ]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  fromPCLPointCloud2(triangles->cloud, *cloud);
  int v0, v1, v2;

  for (size_t i = 0; i < triangles->polygons.size(); i += 1){
    v0 = triangles->polygons[i].vertices[0];
    v1 = triangles->polygons[i].vertices[1];
    v2 = triangles->polygons[i].vertices[2];

    data[i][0][0] = (*cloud)[v0].x;
    data[i][0][1] = (*cloud)[v0].y;
    data[i][0][2] = (*cloud)[v0].z;

    data[i][1][0] = (*cloud)[v1].x;
    data[i][1][1] = (*cloud)[v1].y;
    data[i][1][2] = (*cloud)[v1].z;

    data[i][2][0] = (*cloud)[v2].x;
    data[i][2][1] = (*cloud)[v2].y;
    data[i][2][2] = (*cloud)[v2].z;
  }

  // return pcl::io::savePolygonFileSTL (file, *triangles);  // can't compile
  return 0;
}

int STL_to_Faces(MeshPtr triangles, std::vector< std::vector<int> > &data){
  // index of faces
  // data = [ f1[p1_index, p2_index, p3_index],
  //          f2[p1_index, p2_index, p3_index], ...
  //        ]

  data.resize(triangles->polygons.size());
  for (size_t i = 0; i < triangles->polygons.size(); i += 1){
    data[i].resize(3);

    data[i][0] = triangles->polygons[i].vertices[0];
    data[i][1] = triangles->polygons[i].vertices[1];
    data[i][2] = triangles->polygons[i].vertices[2];
  }
  return 0;
}

int bounding_box(PointCloudXYZRGBPtr cloud, std::vector<float> &b_box){
  float minx = std::numeric_limits<double>::infinity(), miny = std::numeric_limits<double>::infinity(), minz = std::numeric_limits<double>::infinity();
  float maxx = -1 * std::numeric_limits<double>::infinity(), maxy = -1 * std::numeric_limits<double>::infinity(), maxz = -1 * std::numeric_limits<double>::infinity();

  for (uint i = 0; i < cloud->size(); i += 1){
    if((*cloud)[i].x > maxx){
      maxx = (*cloud)[i].x;
    }
    if((*cloud)[i].y > maxy){
      maxy = (*cloud)[i].y;
    }
    if((*cloud)[i].z > maxz){
      maxz = (*cloud)[i].z;
    }

    if((*cloud)[i].x < minx){
      minx = (*cloud)[i].x;
    }
    if((*cloud)[i].y < miny){
      miny = (*cloud)[i].y;
    }
    if((*cloud)[i].z < minz){
      minz = (*cloud)[i].z;
    }
  }
  b_box.resize(6);
  b_box[0] = minx;
  b_box[1] = miny;
  b_box[2] = minz;
  b_box[3] = maxx;
  b_box[4] = maxy;
  b_box[5] = maxz;

  return 0;
}

int apply_transform(PointCloudXYZRGBPtr cloud, float x, float y, float z, float rx, float ry, float rz){
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  std::vector<float> b_box;
  b_box.resize(3);
  std::vector<float> center;
  center.resize(3);

  // move to origin
  bounding_box(cloud, b_box);
  for (int i = 0; i < 3; i += 1){
    center[i] = (b_box[i] + b_box[i + 3]) / 2;
  }
  transform = Eigen::Matrix4f::Identity();
  transform(0, 3) = - center[0];
  transform(1, 3) = - center[1];
  transform(2, 3) = - center[2];
  pcl::transformPointCloud(*cloud, *cloud, transform);

  // rotate
  transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tmpM = Eigen::Matrix4f::Identity();
  float theta;
  theta = rx; // The angle of rotation in radians
  tmpM(1, 1) = cos (theta); //x
  tmpM(1, 2) = -sin(theta);
  tmpM(2, 1) = sin (theta);
  tmpM(2, 2) = cos (theta);
  transform *= tmpM;

  tmpM = Eigen::Matrix4f::Identity();
  theta = ry;
  tmpM(0, 0) = cos (theta); //y
  tmpM(2, 0) = -sin(theta);
  tmpM(0, 2) = sin (theta);
  tmpM(2, 2) = cos (theta);
  transform *= tmpM;

  tmpM = Eigen::Matrix4f::Identity();
  theta = rz;
  tmpM(0, 0) = cos (theta); //z
  tmpM(0, 1) = -sin(theta);
  tmpM(1, 0) = sin (theta);
  tmpM(1, 1) = cos (theta);
  transform *= tmpM;
  pcl::transformPointCloud(*cloud, *cloud, transform);

  // move to proper position
  transform = Eigen::Matrix4f::Identity();
  transform(0, 3) = x;
  transform(1, 3) = y;
  transform(2, 3) = z;
  pcl::transformPointCloud(*cloud, *cloud, transform);


  return 0;

}

int clone(PointCloudXYZRGBPtr obj, PointCloudXYZRGBPtr obj2){
  *obj2 = *obj;
  return 0;
}
int clone(NormalPtr normalObj, NormalPtr normalObj2){
  *normalObj2 = *normalObj;
  return 0;
}
int clone(PointXYZRGBNormalPtr bothobj, PointXYZRGBNormalPtr bothobj2){
  *bothobj2 = *bothobj;
  return 0;
}
int clone(MeshPtr meshobj, MeshPtr meshobj2){
  *meshobj2 = *meshobj;
  return 0;
}


PointCloudXYZRGBPtr add(PointCloudXYZRGBPtr obj, PointCloudXYZRGBPtr obj2){
  PointCloudXYZRGBPtr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
  clone(obj, tmp);
  *tmp += *obj2;
  return tmp;
}
NormalPtr add(NormalPtr normalObj, NormalPtr normalObj2){
  NormalPtr tmp(new pcl::PointCloud<pcl::Normal>);;
  clone(normalObj, tmp);
  *tmp += *normalObj2;
  return tmp;
}
PointXYZRGBNormalPtr add(PointXYZRGBNormalPtr bothobj, PointXYZRGBNormalPtr bothobj2){
  PointXYZRGBNormalPtr tmp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  clone(bothobj, tmp);
  *tmp += *bothobj2;
  return tmp;
}
// MeshPtr add(MeshPtr meshobj, MeshPtr meshobj2){
//   MeshPtr tmp;
//   clone(meshobj, tmp);
//   *tmp += *meshobj2;
//   return tmp;
// }


int split(PointXYZRGBNormalPtr bothobj, PointCloudXYZRGBPtr obj, NormalPtr normalObj){
  copyPointCloud(*bothobj, *obj);
  copyPointCloud(*bothobj, *normalObj);
  return 0;
}
