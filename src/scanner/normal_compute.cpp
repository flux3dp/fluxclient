#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <vector>
using namespace std;

NormalPtr createNormalPtr(){
	NormalPtr normals (new pcl::PointCloud<pcl::Normal>);
	return normals;
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

inline int check(vector<float> normal, vector<float> position_v){
   if (normal[0] * position_v[0] + normal[1] * position_v[1] + normal[1] * position_v[1] > 0)
      return 1;
   else
      return 0;
}

int ne_viewpoint(PointCloudXYZRGBPtr cloud, NormalPtr normals, float radius, vector<vector<int> >viewp, vector<int> step){

   pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> nest;
   nest.setNumberOfThreads(4);
   nest.setRadiusSearch (radius);
   nest.compute (*normals);

   vector<float> normal(3, 0);
   vector<float> position_v(3, 0);

   for (int vp = 0; vp < viewp.size(); vp += 1){
      for (int i = 0; i < step.size() - 1; i += 1){
         normal[0]  = (*normals).points[i].normal_x;
         normal[1]  = (*normals).points[i].normal_y;
         normal[2]  = (*normals).points[i].normal_z;
         position_v[0] = viewp[vp][0] - cloud->points[i].x;
         position_v[1] = viewp[vp][1] - cloud->points[i].y;
         position_v[2] = viewp[vp][2] - cloud->points[i].z;

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
   // nest.setViewPoint(0.0, 170.0, 90);
   // nest.setInputCloud (cloud);
   // nest.compute (*normals);
   return 1;
}

PointXYZRGBNormalPtr concatenatePointsNormal(PointCloudXYZRGBPtr cloud, NormalPtr normals){
   PointXYZRGBNormalPtr final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::concatenateFields (*cloud, *normals, *final);
   return final;
}


// int main (int argc, char** argv){
//          // load point cloud
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//    pcl::io::loadPCDFile ("26_a.pcd", *cloud);
//    ne(cloud, normals);

//    pcl::concatenateFields (*cloud, *normals, *final);
//    // pcl::io::savePLYFileASCII (argv[1], *final);
//    pcl::io::savePCDFileASCII (argv[1], *final);
//    return 0;
// }

