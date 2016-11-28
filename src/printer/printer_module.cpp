#include <iostream>
#include <cmath>
#include <map>
#include <algorithm>
#include <limits>

#include "printer_module.h"


MeshPtr createMeshPtr(){
  MeshPtr mesh(new pcl::PolygonMesh);
  return mesh;
}

CloudPtr createCloudPtr(std::vector< std::vector<float> > points){
  CloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (uint32_t i = 0; i < points.size(); i += 1){
    pcl::PointXYZ p;
    p.x = points[i][0];
    p.y = points[i][1];
    p.z = points[i][2];
    cloud -> push_back(p);
  }
  return cloud;
}

int setCloud(MeshPtr triangles, CloudPtr cloud){
  toPCLPointCloud2(*cloud, triangles->cloud);
  return 1;
}

int setPoints(MeshPtr triangles, std::vector< std::vector<float> > points){
  pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
  for (uint32_t i = 0; i < points.size(); i += 1){
    pcl::PointXYZ p;
    p.x = points[i][0];
    p.y = points[i][1];
    p.z = points[i][2];
    cloud -> push_back(p);
  }

  toPCLPointCloud2(*cloud, triangles->cloud);
  delete cloud;
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

int add_on(MeshPtr base, MeshPtr add_on_mesh){
  pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
  fromPCLPointCloud2(base->cloud, *cloud);
  int size_to_add_on = cloud->size();

  pcl::PointCloud<pcl::PointXYZ>* cloud2 = new pcl::PointCloud<pcl::PointXYZ>();
  fromPCLPointCloud2(add_on_mesh->cloud, *cloud2);

    // add cloud together
  *cloud += *cloud2;

  pcl::Vertices v;
  v.vertices.resize(3);
    // add faces, but shift the index for add on mesh
  for (uint32_t i = 0; i < add_on_mesh->polygons.size(); i += 1){
    v.vertices[0] = add_on_mesh->polygons[i].vertices[0] + size_to_add_on;
    v.vertices[1] = add_on_mesh->polygons[i].vertices[1] + size_to_add_on;
    v.vertices[2] = add_on_mesh->polygons[i].vertices[2] + size_to_add_on;
    base->polygons.push_back(v);
  }

  toPCLPointCloud2(*cloud, base->cloud);
  delete cloud;
  delete cloud2;
  return 0;
}

int bounding_box(MeshPtr triangles, std::vector<float> &b_box){
  pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
  fromPCLPointCloud2(triangles->cloud, *cloud);
  float minx = std::numeric_limits<double>::infinity(), miny = std::numeric_limits<double>::infinity(), minz = std::numeric_limits<double>::infinity();
  float maxx = -1 * std::numeric_limits<double>::infinity(), maxy = -1 * std::numeric_limits<double>::infinity(), maxz = -1 * std::numeric_limits<double>::infinity();

  for (uint32_t i = 0; i < cloud->size(); i += 1){
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
  delete cloud;
  return 0;
}

int bounding_box(pcl::PointCloud<pcl::PointXYZ>* cloud, std::vector<float> &b_box){
  float minx = std::numeric_limits<double>::infinity(), miny = std::numeric_limits<double>::infinity(), minz = std::numeric_limits<double>::infinity();
  float maxx = -1 * std::numeric_limits<double>::infinity(), maxy = -1 * std::numeric_limits<double>::infinity(), maxz = -1 * std::numeric_limits<double>::infinity();

  for (uint32_t i = 0; i < cloud->size(); i += 1){
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

Eigen::Affine3f create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3f rx =
      Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
  Eigen::Affine3f ry =
      Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
  Eigen::Affine3f rz =
      Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
  return rz * ry * rx;
}

int apply_transform(MeshPtr triangles, float x, float y, float z, float rx, float ry, float rz, float sc_x, float sc_y, float sc_z){
  pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
  fromPCLPointCloud2(triangles->cloud, *cloud);

  std::vector<float> b_box;
  b_box.resize(3);
  std::vector<float> center;
  center.resize(3);

  // move to origin
  bounding_box(cloud, b_box);
  for (int i = 0; i < 3; i += 1){
    center[i] = (b_box[i] + b_box[i + 3]) / 2;
  }

  Eigen::Affine3f S(Eigen::Scaling(Eigen::Vector3f(sc_x, sc_y, sc_z)));
  Eigen::Affine3f T_0(Eigen::Translation3f(Eigen::Vector3f(-center[0] * sc_x, -center[1] * sc_y, -center[2] * sc_z)));
  Eigen::Affine3f R = create_rotation_matrix(rx, ry, rz);
  Eigen::Affine3f T_1(Eigen::Translation3f(Eigen::Vector3f(x, y, z)));

  pcl::transformPointCloud(*cloud, *cloud, (T_1 * R * T_0 * S).matrix());

  toPCLPointCloud2(*cloud, triangles->cloud);
  delete cloud;
  return 0;
}

int find_intersect(pcl::PointXYZ &a, pcl::PointXYZ &b, float floor_v, pcl::PointXYZ &p){
  // find the intersect between line a, b and plane z=floor_v
  std::vector<float> v(3);  // vetor a->b
  v[0] = b.x - a.x;
  v[1] = b.y - a.y;
  v[2] = b.z - a.z;

  float t = (floor_v - a.z) / v[2];
  p.x = a.x + t * v[0];
  p.y = a.y + t * v[1];
  p.z = a.z + t * v[2];
  return 0;
}


int cut(MeshPtr input_mesh, MeshPtr out_mesh, float floor_v){
  pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
  fromPCLPointCloud2(input_mesh->cloud, *cloud);

  pcl::Vertices v;
  v.vertices.resize(3);
  pcl::Vertices on, under;
  // consider serveral case

  for (uint32_t i = 0; i < input_mesh->polygons.size(); i += 1){
    // on.vertices.clear();
    // under.vertices.clear();
    pcl::Vertices on, under;
    for (uint32_t j = 0; j < 3; j += 1){

      if ((*cloud)[input_mesh->polygons[i].vertices[j]].z <= floor_v){
        under.vertices.push_back(input_mesh->polygons[i].vertices[j]);
      }
      else{
        on.vertices.push_back(input_mesh->polygons[i].vertices[j]);
      }
    }

    if(on.vertices.size() == 3){
      out_mesh->polygons.push_back(on);
    }
    else if(under.vertices.size() == 3){
      // do nothing
    }
    else if(under.vertices.size() == 2){
      pcl::PointXYZ upper_p;
      upper_p = (*cloud)[on.vertices[0]];
      for (int j = 0; j < 2; j += 1){
        pcl::PointXYZ lower_p, new_p;
        lower_p = (*cloud)[under.vertices[j]];
        find_intersect(upper_p, lower_p, floor_v, new_p);
        cloud -> push_back(new_p);
        on.vertices.push_back(cloud -> size() - 1);
      }
      out_mesh->polygons.push_back(on);
    }
    else if(under.vertices.size() == 1){
      pcl::PointXYZ mid;
      mid.x = ((*cloud)[on.vertices[0]].x + (*cloud)[on.vertices[1]].x) / 2;
      mid.y = ((*cloud)[on.vertices[0]].y + (*cloud)[on.vertices[1]].y) / 2;
      mid.z = ((*cloud)[on.vertices[0]].z + (*cloud)[on.vertices[1]].z) / 2;
      cloud -> push_back(mid);
      int mid_index = cloud -> size() - 1;

      pcl::PointXYZ intersect0, intersect1;
      find_intersect((*cloud)[on.vertices[0]], (*cloud)[under.vertices[0]], floor_v, intersect0);
      cloud -> push_back(intersect0);
      int intersect0_index = cloud -> size() - 1;
      find_intersect((*cloud)[on.vertices[1]], (*cloud)[under.vertices[0]], floor_v, intersect1);
      cloud -> push_back(intersect1);
      int intersect1_index = cloud -> size() - 1;

      pcl::Vertices v1, v2, v3;
      v1.vertices.push_back(on.vertices[0]);
      v1.vertices.push_back(intersect0_index);
      v1.vertices.push_back(mid_index);
      out_mesh->polygons.push_back(v1);

      v2.vertices.push_back(mid_index);
      v2.vertices.push_back(intersect0_index);
      v2.vertices.push_back(intersect1_index);
      out_mesh->polygons.push_back(v2);

      v3.vertices.push_back(mid_index);
      v3.vertices.push_back(intersect1_index);
      v3.vertices.push_back(on.vertices[1]);
      out_mesh->polygons.push_back(v3);
    }

  }

  toPCLPointCloud2(*cloud, out_mesh->cloud);
  delete cloud;
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
  pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
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
  delete cloud;
  return 0;
}

// int STL_to_Faces(MeshPtr triangles, std::vector< std::vector<int> > &data){
//   // index of faces
//   // data = [ f1[p1_index, p2_index, p3_index],
//   //          f2[p1_index, p2_index, p3_index], ...
//   //        ]
//   data.resize(triangles->polygons.size());
//   for (size_t i = 0; i < triangles->polygons.size(); i += 1){
//     data[i].resize(3);
//     data[i][0] = triangles->polygons[i].vertices[0];
//     data[i][1] = triangles->polygons[i].vertices[1];
//     data[i][2] = triangles->polygons[i].vertices[2];
//   }
//   return 0;
// }

int mesh_len(MeshPtr triangles){
  return triangles->polygons.size();
}


void xnormal(float v[3][3], float* result){
    float a[3] = {v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2]};  // std::vector v0 -> v1
    float b[3] = {v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2]};  // std::vector v0 -> v2
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];  // cross product -> surface normal std::vector
}

void xnormalize(float *v){
    float l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if(l != 0){
        v[0] /= l;
        v[1] /= l;
        v[2] /= l;
    }
}

int write_stl_binary(MeshPtr triangles, const char* filename) {
    int v0, v1, v2, face_count = triangles->polygons.size();
    short padding = 0;

    FILE* ptr_stl = fopen(filename, "wb");

    fprintf(stderr, "Writing STL Binary %s\n", filename);
    fprintf(ptr_stl, "%-80s", "FLUX 3d printer: flux3dp.com, 2015");

    pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
    fromPCLPointCloud2(triangles->cloud, *cloud);

    fwrite(&face_count, sizeof(int), 1, ptr_stl);
    fprintf(stderr, "Triangle Faces %d\n", face_count);

    for (int i = 0; i < face_count; i += 1){

      v0 = triangles->polygons[i].vertices[0];
      v1 = triangles->polygons[i].vertices[1];
      v2 = triangles->polygons[i].vertices[2];

      float n[3];
      // output normal

      float vecs[3][3] = {
        {(*cloud)[v0].x, (*cloud)[v0].y, (*cloud)[v0].z},
        {(*cloud)[v1].x, (*cloud)[v1].y, (*cloud)[v1].z},
        {(*cloud)[v2].x, (*cloud)[v2].y, (*cloud)[v2].z}
      };

      xnormal(vecs, n);
      xnormalize(n);
      fwrite(n, sizeof(float), 3, ptr_stl);

      fwrite(vecs[0], sizeof(float), 3, ptr_stl);
      fwrite(vecs[1], sizeof(float), 3, ptr_stl);
      fwrite(vecs[2], sizeof(float), 3, ptr_stl);

      fwrite(&padding, sizeof(short), 1, ptr_stl);
    }

    delete cloud;
    fclose(ptr_stl);
}