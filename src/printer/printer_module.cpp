#include <vector>
#include "printer_module.h"

int set_point(MeshPtr triangles, std::vector< std::vector<float> > points;){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < points.size(); i += 1){
    pcl::PointXYZRGB p;
    p.x = points[i][0];
    p.y = points[i][1];
    p.z = points[i][2];
    cloud -> push_back(p);
  }
  pcl::PCLPointCloud2 cloud2;
  toPCLPointCloud2(cloud, cloud2);
  triangles->cloud = cloud2;
  return 0;
}

int push_backFace(MeshPtr triangles, int v0, int v1, int v2){
  pxl::Vertices v;
  v.vertices.resize(3);
  v.vertices[0] = v0;
  v.vertices[1] = v1;
  v.vertices[2] = v2;
  triangles->polygons.push_back(v);
  return 0;
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
    std::cout << "  polygons[" << i << "]: " <<std::endl;
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
