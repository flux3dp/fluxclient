
#include "noise_del.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

PointCloudXYZRGB createPointCloudXYZRGB() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (
        new pcl::PointCloud<pcl::PointXYZRGB>);
    return cloud;
}

int loadPointCloudXYZRGB(const char* file, PointCloudXYZRGB cloud) {
    return pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud);
}

void dumpPointCloudXYZRGB(const char* file, PointCloudXYZRGB cloud) {
    pcl::io::savePCDFileASCII (file, *cloud);
}

void push_back(PointCloudXYZRGB cloud, float x, float y, float z, uint32_t rgb){
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.rgb = rgb;
    cloud -> push_back(p);
}


int SOR(PointCloudXYZRGB cloud, int neighbors, float threshold) {
    // statistical_outlier_removal

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setMeanK (neighbors);  //number of neighbors
    sor.setStddevMulThresh (threshold); //how many deviation out of n

    sor.setInputCloud (cloud);
    sor.filter (*cloud);

    return 0;
}

int VG(PointCloudXYZRGB cloud) {
    // voxel_grid

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setMinimumPointsNumberPerVoxel(0); //set min points need to in a voxel
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.5f, 0.5f, 0.5f);
    vg.filter (*cloud);

    return 0;
}
