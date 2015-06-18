#include "reg.h"

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>


PointCloudTPtr createPointCloudPointNormal() {
    PointCloudTPtr cloud (new pcl::PointCloud<PointNT>);
    return cloud;
}

int loadPointCloudPointNormal(const char* file, PointCloudTPtr cloud) {
    return pcl::io::loadPCDFile<PointNT> (file, *cloud);
}

void dumpPointCloudPointNormal(const char* file, PointCloudTPtr cloud) {
    pcl::io::savePCDFileASCII(file, *cloud);
}

int downsample(PointCloudTPtr object, float leaf){
  pcl::VoxelGrid<PointNT> grid;
  // const float leaf = 2.0f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  return 1;
}

int NE_OMP(PointCloudTPtr object,float radius){
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (radius);
  nest.setInputCloud (object);
  nest.compute (*object);
  return 1;
}

int FE(PointCloudTPtr object, FeatureCloudTPtr object_features, float radius){
  FeatureEstimationT fest;
  fest.setRadiusSearch (10);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  return 1;
}
int SCP(PointCloudTPtr object, FeatureCloudTPtr object_features, PointCloudTPtr scene, FeatureCloudTPtr scene_features, Eigen::Matrix4f &transformation, float leaf){
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;

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

// // Align a rigid object to a scene with clutter and occlusions
// int main (int argc, char **argv)
// {
//   pcl::ScopeTime t_main("registration");
//   // Point clouds
//   PointCloudT::Ptr object (new PointCloudT);
//   PointCloudT::Ptr object_aligned (new PointCloudT);
//   PointCloudT::Ptr scene (new PointCloudT);
//   FeatureCloudTPtr object_features (new FeatureCloudT);
//   FeatureCloudTPtr scene_features (new FeatureCloudT);


//   // Get input object and scene
//   if (argc != 3)
//   {
//     pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
//     return (1);
//   }

//   // Load object and scene
//   pcl::console::print_highlight ("Loading point clouds...\n");
//   if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
//       pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
//   {
//     pcl::console::print_error ("Error loading object/scene file!\n");
//     return (1);
//   }
//   pcl::console::print_info (" cloud size:%d, cloud2 size:%d \n",object->points.size (), scene->points.size ());

//   object, scene
//   // Downsample
//   pcl::console::print_highlight ("Downsampling...\n");
//   // pcl::VoxelGrid<PointNT> grid;
//   // const float leaf = 2.0f;
//   // grid.setLeafSize (leaf, leaf, leaf);
//   // grid.setInputCloud (object);
//   // grid.filter (*object);
//   // grid.setInputCloud (scene);
//   // grid.filter (*scene);

//   pcl::console::print_info (" cloud size:%d, cloud2 size:%d \n",object->points.size (), scene->points.size ());


//   // Estimate normals for scene
//   // pcl::console::print_highlight ("Estimating scene normals...\n");
//   // pcl::NormalEstimationOMP<PointNT,PointNT> nest;
//   // nest.setRadiusSearch (4);
//   // nest.setInputCloud (scene);
//   // nest.compute (*scene);
//   // nest.setInputCloud (object);
//   // nest.compute (*object);

//   // Estimate features
//   pcl::console::print_highlight ("Estimating features...\n");
//   // FeatureEstimationT fest;
//   // fest.setRadiusSearch (10);
//   // fest.setInputCloud (object);
//   // fest.setInputNormals (object);
//   // fest.compute (*object_features);

//   // fest.setInputCloud (scene);
//   // fest.setInputNormals (scene);
//   // fest.compute (*scene_features);

//   // Perform alignment
//   pcl::console::print_highlight ("Starting alignment...\n");

//   pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;

//   align.setInputSource (object);
//   align.setSourceFeatures (object_features);
//   align.setInputTarget (scene);
//   align.setTargetFeatures (scene_features);
//   align.setMaximumIterations (15000); // Number of RANSAC iterations
//   align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//   align.setCorrespondenceRandomness (10); // Number of nearest features to use
//   align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
//   align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
//   align.setInlierFraction (0.3f); // Required inlier fraction for accepting a pose hypothesis
//   {
//     pcl::ScopeTime t("Alignment");
//     align.align (*object_aligned);
//   }




//   if (align.hasConverged())
//   {
//     // Print results
//     printf ("\n");
//     Eigen::Matrix4f transformation = align.getFinalTransformation ();
//     pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
//     pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
//     pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
//     pcl::console::print_info ("\n");
//     pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
//     pcl::console::print_info ("\n");
//     pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_orig (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_orig (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *object_orig);
//     pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *scene_orig);


//     pcl::transformPointCloud (*object_orig, *object_orig, transformation);
//     // *object_orig = *object_orig + *scene_orig;
//     *object_orig = *scene_orig;

//     std::cout<<"hi:"<< object_orig->width<<std::endl;

//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
//     std::cout<<"hi:"<< object_orig->points[0]<<std::endl;
//     ne.setRadiusSearch (8);
//     ne.setInputCloud (object_orig);
//     ne.compute (*final);

//     pcl::concatenateFields (*object_orig, *final, *final);

//     std::cout<<"hi:"<< final->points[0]<<std::endl;


//     // pcl::io::savePCDFileASCII ("object_orig.pcd",  *object_orig);
//     // pcl::io::savePCDFileASCII ("scene_orig.pcd", *scene_orig);

//     pcl::io::savePCDFileASCII ("wth.pcd",  *final);

//     // pcl::io::savePCDFileASCII ("wth.pcd",  *object_aligned);
//     // pcl::io::savePCDFileASCII ("wth2.pcd", *scene);

//   }
//   else
//   {
//     pcl::console::print_error ("Alignment failed!\n");
//     return (1);
//   }

//   return (0);
// }
