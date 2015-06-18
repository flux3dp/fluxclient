#include "scan_module.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/surface/poisson.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>




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

int VG(PointCloudXYZRGBPtr cloud) {
    // voxel_grid

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setMinimumPointsNumberPerVoxel(0); //set min points need to in a voxel
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.5f, 0.5f, 0.5f);
    vg.filter (*cloud);

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

   for (int vp = 0; vp < viewp.size(); vp += 1){
      for (int i = 0; i < step.size() - 1; i += 1){
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
int downsample(PointXYZRGBNormalPtr cloud, float leaf){
  pcl::VoxelGrid<PointNT> grid;
  // const float leaf = 2.0f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud);
  return 1;
}

int FE(PointXYZRGBNormalPtr cloud, FeatureCloudTPtr cloud_features, float radius){
  FeatureEstimationT fest;
  fest.setRadiusSearch (10);
  fest.setInputCloud (cloud);
  fest.setInputNormals (cloud);
  fest.compute (*cloud_features);
  return 1;
}

int SCP(PointXYZRGBNormalPtr object, FeatureCloudTPtr object_features, PointXYZRGBNormalPtr scene, FeatureCloudTPtr scene_features, Eigen::Matrix4f &transformation, float leaf){
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;

  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (15000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (10); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.3f); // Required inlier fraction for accepting a pose hypothesis
  // {
  //   pcl::ScopeTime t("Alignment");
  //   align.align (*object_aligned);
  // }
  transformation = align.getFinalTransformation ();
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

int POS(PointXYZRGBNormalPtr cloud_with_normals, MeshPtr triangles){
  puts("poisson computing");

  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;

  // poisson.setConfidence(true);
  // poisson.setScale(1.0); // from 1.1 to 1.0
  poisson.setDepth (12);
  poisson.setIsoDivide(7);
  poisson.setInputCloud (cloud_with_normals);
  poisson.performReconstruction (*triangles);

  return 0;
}
int dumpSTL(const char* file, MeshPtr triangles){
  // (*triangles)
  return pcl::io::savePolygonFileSTL (file, *triangles);


}

