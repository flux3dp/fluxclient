#include <limits>

#include "printer_module.h"


MeshPtr createMeshPtr(){
  MeshPtr mesh(new pcl::PolygonMesh);
  return mesh;
}

int set_point(MeshPtr triangles, std::vector< std::vector<float> > points){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (uint i = 0; i < points.size(); i += 1){
    pcl::PointXYZ p;
    p.x = points[i][0];
    p.y = points[i][1];
    p.z = points[i][2];
    cloud -> push_back(p);
  }

  toPCLPointCloud2(*cloud, triangles->cloud);
  return 0;
}

int push_backFace(MeshPtr triangles, int v0, int v1, int v2){
  pcl::Vertices v;
  v.vertices.resize(3);
  v.vertices[0] = v0;
  v.vertices[1] = v1;
  v.vertices[2] = v2;
  triangles->polygons.push_back(v);
  return 0;
}

int add_on(pcl::PolygonMesh::Ptr base, pcl::PolygonMesh::Ptr add_on_mesh){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(base->cloud, *cloud);
    int size_to_add_on = cloud->size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(add_on_mesh->cloud, *cloud2);

    // add cloud together
    *cloud += *cloud2;

    pcl::Vertices v;
    v.vertices.resize(3);
    // add faces, but shift the index for add on mesh
    for (uint i = 0; i < add_on_mesh->polygons.size(); i += 1){
        v.vertices[0] = add_on_mesh->polygons[i].vertices[0] + size_to_add_on;
        v.vertices[1] = add_on_mesh->polygons[i].vertices[1] + size_to_add_on;
        v.vertices[2] = add_on_mesh->polygons[i].vertices[2] + size_to_add_on;
        base->polygons.push_back(v);
    }

    toPCLPointCloud2(*cloud, base->cloud);
    return 0;
}

int bounding_box(MeshPtr triangles, std::vector<float> &b_box){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fromPCLPointCloud2(triangles->cloud, *cloud);
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

int bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<float> &b_box){
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

int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float sc_x, float sc_y, float sc_z){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fromPCLPointCloud2(triangles->cloud, *cloud);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tmpM = Eigen::Matrix4f::Identity();
  float theta; // The angle of rotation in radians

  std::vector<float> b_box;
  b_box.resize(3);
  std::vector<float> center;
  center.resize(3);

  // scale
  for (uint i = 0; i < cloud->size(); i += 1){
    (*cloud)[i].x *= sc_x;
    (*cloud)[i].y *= sc_y;
    (*cloud)[i].z *= sc_z;
  }

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
  tmpM = Eigen::Matrix4f::Identity();
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

  toPCLPointCloud2(*cloud, triangles->cloud);
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(triangles->cloud, *cloud);

    int v0, v1, v2;

    std::vector<float> tmpvv(3, 0.0);
    std::vector< std::vector<float> > tmpv(3, tmpvv);
    data.resize(triangles->polygons.size());

    for (size_t i = 0; i < triangles->polygons.size(); i += 1){
      data[i] = tmpv;

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
      // std::cout << "  polygons[" << i << "]: " <<std::endl;
    }
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
