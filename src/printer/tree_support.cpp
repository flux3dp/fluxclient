#include "tree_support.h"
#include <pcl/filters/voxel_grid.h>
//fake
#include <pcl/io/pcd_io.h>
// ref: http://hpcg.purdue.edu/bbenes/papers/Vanek14SGP.pdf

float d_v3(Eigen::Vector3f &a, Eigen::Vector3f &b){
  // compute distance between two 3d vector
  return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
}

struct cone{
  // a right circular cone where its vertex exist at "pos"
  // and theta degrees between axis and Hypotenuse
  Eigen::Vector3f pos;
  float theta;
  cone(){
    pos = Eigen::Vector3f(0, 0, 0);
    theta = 0;
  }
  cone(float x, float  y, float z, float t){
    pos = Eigen::Vector3f(x, y, z);
    theta = t;
  }
  cone(cone &cone2){
    pos[0] = cone2.pos[0];
    pos[1] = cone2.pos[1];
    pos[2] = cone2.pos[2];
    theta = cone2.theta;
  }
  cone(const cone &cone2){
    pos[0] = cone2.pos[0];
    pos[1] = cone2.pos[1];
    pos[2] = cone2.pos[2];
    theta = cone2.theta;
  }
};

struct tri_data{
  // store data for each triangle in stl
  // ok:whether it's a valid triangle
  // tri_after: where the triangle at after transform to xy-plane
  // tri_trans: the transform matrix
  // L: key =
  bool ok;
  Eigen::Matrix3f tri_after;
  Eigen::Affine3f tri_trans;
  std::map<int, Eigen::Vector3f> L;
  std::map<int, bool> in_tri;
};

struct tree_node{
  // a tree node structure that consist its children's index
  // left: the left-side node index
  // note that if left = -1: points needed support, leaf of the tree
  //                   = -2: points that connect to stl mesh
  //                   = -3: points that connect to the tray
  // right: the right-side node index
  // index: index of it's self
  // height: not really height but the total number of childs on top of this node
  int left, right, index, height;
  tree_node(int l, int r, int i, int h){
    left = l;
    right = r;
    index = i;
    height = h;
  }
};

inline std::ostream& operator<< (std::ostream& stream, const tree_node& n){
  stream << n.left << " " << n.right << " " << n.index << " " << n.height << " " << std::endl;
  return stream;
}

class SupportTree{
  // class that store a support structure's points and data

public:
  SupportTree(pcl::PointCloud<pcl::PointXYZ>::Ptr P);
  ~SupportTree();
  void out_as_js();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<tree_node> tree;
};

SupportTree::SupportTree(pcl::PointCloud<pcl::PointXYZ>::Ptr P){
  cloud = P;
}

SupportTree::~SupportTree(){
}

void SupportTree::out_as_js(){
  // output a tree as a json list
  std::cout<< "tree = [";
  for (size_t i = 0; i < tree.size(); i += 1){
    if(tree[i].left != -1){
      std::cout<< "[";

      std::cout << "[" << (*cloud)[tree[i].index].x << "," << (*cloud)[tree[i].index].y << "," << (*cloud)[tree[i].index].z << "]";
      std::cout<< ",";

      if(tree[i].left >= 0){
        std::cout << "[" << (*cloud)[tree[i].left].x << "," << (*cloud)[tree[i].left].y << "," << (*cloud)[tree[i].left].z << "]";
        std::cout<< ",";
      }
      else{
      }
      std::cout << "[" << (*cloud)[tree[i].right].x << "," << (*cloud)[tree[i].right].y << "," << (*cloud)[tree[i].right].z << "]";
      std::cout<< ",";

      std::cout<< tree[i].height;

      std::cout<< "],";
      // std::cout<< std::endl;
    }
  }
  std::cout<< "]";
  return;
}

inline std::ostream& operator<< (std::ostream& stream, const SupportTree& t){
  for (size_t i = 0; i < t.tree.size(); i += 1){
    stream << t.tree[i].left << ", " << t.tree[i].right << ", " << t.tree[i].index << ", " << t.tree[i].height << ", " << std::endl;
  }
  return stream;
}

bool sort_by_z(const pcl::PointXYZ a,const pcl::PointXYZ b)
{
   return a.z < b.z;
}
bool sort_by_second(const std::pair<int, float> a, const std::pair<int, float> b){
  if(a.second != b.second){
    return a.second < b.second;
  }
  else{
    return a.first < b.first;
  }
}

Eigen::Vector3f sol_2(float x1, float y1, float x2, float y2){
  // solve the equation pass through (x1, y1), (x2, y2)
  // ax + by + c= 0
  // return (a, b, c)
  return Eigen::Vector3f(y1 - y2, x2 - x1, x1 * y2 - x2 * y1);
}

float sub_in(float y, float z, Eigen::Vector3f f){
  // substitute point into equation
  // return ax + by + c
  return f(0) * y + f(1) * z + f(2);
}

bool check_valid_tri(float a, float b, float c){
  // check whether a triangle is valid by checking the legth of 3 edge
  if(a + b <= c || a + c <= b || b + c <= a){
    return false;
  }
  else{
    return true;
  }
}

int add_support(MeshPtr input_mesh, MeshPtr out_mesh, float alpha){
  // Eigen::Matrix3f tmp;
  // tmp.setZero();
  // Eigen::Vector3f start(1,2,3);
  // tmp.block<3, 1>(0, 0) = start;
  // std::cerr<< "tmp " << tmp << std::endl;
  // return 0;
  ////////////////// TODO: read paper to find out what's this /////////////////
  double m_threshold = std::numeric_limits<double>::infinity();
  //////////////////////////////////////////////////////

  pcl::PointCloud<pcl::PointXYZ>::Ptr P(new pcl::PointCloud<pcl::PointXYZ>);  // recording every point's xyz data
  find_support_point(input_mesh, alpha, 1, P);

  std::vector<tri_data> preprocess_tri;
  preprocess(input_mesh, preprocess_tri);

  std::cerr<< "P size need to be supported:"<< P->size() << std::endl;
  SupportTree support_tree(P);

  sort(P -> points.begin(), P -> points.end(), sort_by_z);
  std::map<int, cone> C;  // recording every cone, key = index of P

  std::vector< std::pair<int, float> > P_v;  // main index list, first = index of P, second = z-coordinate
  for (size_t i = 0; i < P -> points.size(); i += 1){
    C[i] = cone((float)P -> points[i].x, (float)P -> points[i].y, (float)P -> points[i].z, alpha);
    P_v.push_back(std::pair<int, float>(i, P -> points[i].z));
  }
  sort(P_v.begin(), P_v.end(), sort_by_second);
  for (size_t i = 0; i < P_v.size(); i += 1){
    support_tree.tree.push_back(tree_node(-1, -1, P_v[i].first, 1));
  }
  // std::cout<< "tree " << support_tree << std::endl;

  std::vector<std::pair <int, float> > S;  // candidates for intersetino point, first:index of P_v,second: distance

  std::map<int, cone> tmp_C;  // cones, key = index of P_v
  int current_point; // current index of P
  while(P_v.size() != 0){
    current_point = P_v.back().first;  // index of P
    P_v.pop_back();  // erase highest p, can slightly skip some if statement

    S.clear();
    tmp_C.clear();
    // cone-cone intersection
    for (size_t i = 0; i < P_v.size(); i += 1){
      cone tmp_cone;
      S.push_back(std::pair<int, double>(i, cone_intersect(C[current_point], C[P_v[i].first], tmp_cone)));
      tmp_C[i] = tmp_cone;
    }

    // cone-plate intersection
    S.push_back(std::pair<int, double>(-1, P -> points[current_point].z));

    // cone-mesh intersection, use -2 to indicate it's connected to mesh
    Eigen::Vector3f cm_point;
    S.push_back(std::pair<int, double>(-2, cone_mesh_intersect(C[current_point], input_mesh, preprocess_tri, cm_point)));

    // choose the right candidate
    std::pair<int, float> m(S[0]);  // first: index of P_v, second: distance
    for (size_t i = 0; i < S.size(); i += 1){
      if(S[i].second < m.second){
        m = S[i];
      }
      // std::cout<< "S:" << S[i].first << " " << S[i].second << std::endl;
    }
    if(m.second > m_threshold){
        // C.erase(current_point);
    }
    else{
      if(m.first >= 0){  // cone-cone
        // tmp_C[m.first]
        // std::cout<< "m.first " << m.first << std::endl;
        cone c(tmp_C[m.first]);
        C[P->points.size()] = c;

        P -> points.push_back(pcl::PointXYZ(c.pos[0], c.pos[1], c.pos[2]));
        std::pair<int, double> new_pv(P->size() - 1, c.pos[2]);

        support_tree.tree.push_back(tree_node(current_point, P_v[m.first].first, new_pv.first, support_tree.tree[current_point].height + support_tree.tree[P_v[m.first].first].height));
        P_v.erase(P_v.begin() + m.first);

        std::vector< std::pair<int, float> >::iterator low;

        low = std::lower_bound (P_v.begin(), P_v.end(), new_pv, sort_by_second);
        P_v.insert(low, new_pv);
      }
      else if(m.first == -1){ // plate-cone
        // support_tree
        P -> points.push_back(pcl::PointXYZ(P -> points[current_point].x, P -> points[current_point].y, 0));
        support_tree.tree.push_back(tree_node(-3, current_point, P->size() - 1, support_tree.tree[current_point].height));
      }
      else if(m.first == -2){ // mesh-cone
        //////////////////// TODO ////////////////////////////////////////
        P -> points.push_back(pcl::PointXYZ(cm_point[0], cm_point[1], cm_point[2]));
        // std::cerr<< "P " << P->points.back() << std::endl;
        support_tree.tree.push_back(tree_node(-2, current_point, P->size() - 1, support_tree.tree[current_point].height));
        ////////////////////////////////////////////////////////////
      }
      else{
        std::cout<< "GG, this shouldn't  happen"<< std::endl;
        assert(false);
      }
    }
  }

  // std::cout<< "tree " << support_tree << std::endl;
  // std::cout<< "support_tree.cloud " << support_tree.cloud -> size() << std::endl;
  genearte_strut(P, support_tree, out_mesh);
  support_tree.out_as_js();
  return 0;
}

int preprocess(MeshPtr input_mesh, std::vector<tri_data> &preprocess_tri){
  // ref:http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.479.8237&rep=rep1&type=pdf
  // preprocess the triangls, compute transform matrix that can transform them to xy-plane
  // and some other data such as the eautations for 3 edge and line that pass through vertex and perpendicular to each edge
  // 9 lines in total
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fromPCLPointCloud2(input_mesh->cloud, *cloud);
  for (size_t i = 0; i < input_mesh -> polygons.size(); i += 1){
  // for (size_t i = 825; i < 826; i += 1){
    tri_data a;
    Eigen::Matrix3f tri_before;
    Eigen::Matrix3f tri_after;
    tri_after.setZero();
    for (size_t j = 0; j < 3; j += 1){
      tri_before(0, j) = (*cloud)[(input_mesh->polygons[i]).vertices[j]].x;
      tri_before(1, j) = (*cloud)[(input_mesh->polygons[i]).vertices[j]].y;
      tri_before(2, j) = (*cloud)[(input_mesh->polygons[i]).vertices[j]].z;
    }

    float d12 = sqrt(pow(tri_before(0, 0) - tri_before(0, 1), 2) + pow(tri_before(1, 0) - tri_before(1, 1), 2) + pow(tri_before(2, 0) - tri_before(2, 1), 2));
    float d13 = sqrt(pow(tri_before(0, 0) - tri_before(0, 2), 2) + pow(tri_before(1, 0) - tri_before(1, 2), 2) + pow(tri_before(2, 0) - tri_before(2, 2), 2));
    float d23 = sqrt(pow(tri_before(0, 1) - tri_before(0, 2), 2) + pow(tri_before(1, 1) - tri_before(1, 2), 2) + pow(tri_before(2, 1) - tri_before(2, 2), 2));
    a.ok = check_valid_tri(d12, d13, d23);
    if(a.ok){
      tri_after(2, 1) = d12;
      tri_after(2, 2) = (pow(d13, 2) - pow(d23, 2) + pow(d12, 2)) / 2 / d12;
      tri_after(1, 2) = sqrt(pow(d13, 2) - pow(tri_after(2, 2), 2));
      a.tri_after = tri_after;

      for (size_t i = 0; i < 3; i += 1){
        for (size_t j = 0; j < 3; j += 1){
          tri_after(i, j) -= tri_before(i, 0);
        }
      }
      Eigen::Matrix4f tri_trans;
      tri_trans.setIdentity();
      tri_trans.block<3,3>(0,0) = tri_after * tri_before.inverse();
      for (size_t j = 0; j < 3; j += 1){
        tri_trans(j, 3) = tri_before(j, 0);
      }
      a.tri_trans = tri_trans;
      tri_after = a.tri_after;
      // std::cerr<< "tri_before " << tri_before << std::endl;
      // std::cerr<< "a " << a.tri_trans * tri_before << std::endl;
      // std::cerr<< "tri_after " << tri_after << std::endl;

      float tmp;

      a.L[13] = sol_2(tri_after(1, 0), tri_after(2, 0), tri_after(1, 2), tri_after(2, 2));
      // std::cerr<< "a[13] " << a.L[13] << std::endl;
      a.in_tri[13] = sub_in(tri_after(1, 1), tri_after(2, 1), a.L[13]) >= 0;
      // std::cerr<< "t " << sub_in(tri_after(1, 1), tri_after(2, 1), a.L[13]) << std::endl;
      // std::cerr<< "in 13 " << a.in_tri[13] << std::endl;

      tmp = -sub_in(tri_after(1, 0), tri_after(2, 0) , Eigen::Vector3f(a.L[13](1), -a.L[13](0), 0));
      a.L[111] = Eigen::Vector3f(a.L[13](1), -a.L[13](0), tmp);
      a.in_tri[111] = sub_in(tri_after(1, 2), tri_after(2, 2), a.L[111]) >= 0;

      tmp = -sub_in(tri_after(1, 2), tri_after(2, 2) , Eigen::Vector3f(a.L[13](1), -a.L[13](0), 0));
      a.L[331] = Eigen::Vector3f(a.L[13](1), -a.L[13](0), tmp);
      a.in_tri[331] = sub_in(tri_after(1, 0), tri_after(2, 0), a.L[331]) >= 0;


      a.L[23] = sol_2(tri_after(1, 1), tri_after(2, 1), tri_after(1, 2), tri_after(2, 2));
      // std::cerr<< "a[23] " << a.L[23] << std::endl;
      a.in_tri[23] = sub_in(tri_after(1, 0), tri_after(2, 0), a.L[23]) >= 0;

      tmp = -sub_in(tri_after(1, 1), tri_after(2, 1) , Eigen::Vector3f(a.L[23](1), -a.L[23](0), 0));
      a.L[223] = Eigen::Vector3f(a.L[23](1), -a.L[23](0), tmp);
      a.in_tri[223] = sub_in(tri_after(1, 2), tri_after(2, 2), a.L[223]) >= 0;

      tmp = -sub_in(tri_after(1, 2), tri_after(2, 2) , Eigen::Vector3f(a.L[23](1), -a.L[23](0), 0));
      a.L[332] = Eigen::Vector3f(a.L[23](1), -a.L[23](0), tmp);
      a.in_tri[332] = sub_in(tri_after(1, 1), tri_after(2, 1), a.L[332]) >= 0;


      a.L[12] = sol_2(tri_after(1, 0), tri_after(2, 0), tri_after(1, 1), tri_after(2, 1));
      // std::cerr<< "a[12] " << a.L[12] << std::endl;
      a.in_tri[12] = sub_in(tri_after(1, 2), tri_after(2, 2), a.L[12]) >= 0;

      tmp = -sub_in(tri_after(1, 0), tri_after(2, 0) , Eigen::Vector3f(a.L[12](1), -a.L[12](0), 0));
      a.L[112] = Eigen::Vector3f(a.L[12](1), -a.L[12](0), tmp);
      a.in_tri[112] = sub_in(tri_after(1, 1), tri_after(2, 1), a.L[112]) >= 0;

      tmp = -sub_in(tri_after(1, 1), tri_after(2, 1) , Eigen::Vector3f(a.L[12](1), -a.L[12](0), 0));
      a.L[221] = Eigen::Vector3f(a.L[12](1), -a.L[12](0), tmp);
      a.in_tri[221] = sub_in(tri_after(1, 0), tri_after(2, 0), a.L[221]) >= 0;
    }
    preprocess_tri.push_back(a);
  }
  return 0;
}

int find_support_point(MeshPtr triangles, float alpha, float sample_rate, pcl::PointCloud<pcl::PointXYZ>::Ptr P){
  // ref: http://hpcg.purdue.edu/bbenes/papers/Vanek14SGP.pdf
  // MeshPtr triangles[in]: input stl
  // float alpha[in]: angle that >= alpha need to be supported
  // float sample_rate[in]: sample reate (grid)
  // P[out]: out put the points that need to be supported

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fromPCLPointCloud2(triangles->cloud, *cloud);
  std::cerr << "input points " << cloud -> size() << std::endl;
  std::cerr << "input faces " << triangles->polygons.size() << std::endl;
  // std::vector<float> normal_p;
  // std::vector<float> normal_vertical;
  // normal_vertical.push_back(0);
  // normal_vertical.push_back(0);
  // normal_vertical.push_back(1);
  float cos_alpha = cos((M_PI / 2.0) - alpha);
  float a[3], b[3], normal_p[3];
  float la, lb;
  const float normal_vertical[3] = {0, 0, -1};
  std::vector<int> rec;
  for (size_t i = 0; i < triangles->polygons.size(); i += 1){
  // for (size_t i = 0; i < 1; i += 1){
    a[0] = (*cloud)[(triangles->polygons[i]).vertices[1]].x - (*cloud)[(triangles->polygons[i]).vertices[0]].x;
    a[1] = (*cloud)[(triangles->polygons[i]).vertices[1]].y - (*cloud)[(triangles->polygons[i]).vertices[0]].y;
    a[2] = (*cloud)[(triangles->polygons[i]).vertices[1]].z - (*cloud)[(triangles->polygons[i]).vertices[0]].z;

    b[0] = (*cloud)[(triangles->polygons[i]).vertices[2]].x - (*cloud)[(triangles->polygons[i]).vertices[0]].x;
    b[1] = (*cloud)[(triangles->polygons[i]).vertices[2]].y - (*cloud)[(triangles->polygons[i]).vertices[0]].y;
    b[2] = (*cloud)[(triangles->polygons[i]).vertices[2]].z - (*cloud)[(triangles->polygons[i]).vertices[0]].z;

    normal_p[0] = a[1] * b[2] - a[2] * b[1];
    normal_p[1] = a[2] * b[0] - a[0] * b[2];
    normal_p[2] = a[0] * b[1] - a[1] * b[0];

    // std::cout << normal_p[2] << std::endl;
    float cos_theta = (normal_p[0] * normal_vertical[0] + normal_p[1] * normal_vertical[1] + normal_p[2] * normal_vertical[2]) / (sqrt(normal_p[0] * normal_p[0] + normal_p[1] * normal_p[1] + normal_p[2] * normal_p[2]) * sqrt(normal_vertical[0] * normal_vertical[0] + normal_vertical[1] * normal_vertical[1] + normal_vertical[2] * normal_vertical[2]));
    // std::cout << cos_theta << std::endl;
    // faces that need to be supported
    if(cos_theta >= cos_alpha){  //cosine, larger angle, smaller cosine
      rec.push_back(i);
      // std::cout << "> "<<i << std::endl;
      b[0] = (*cloud)[(triangles->polygons[i]).vertices[2]].x - (*cloud)[(triangles->polygons[i]).vertices[1]].x;
      b[1] = (*cloud)[(triangles->polygons[i]).vertices[2]].y - (*cloud)[(triangles->polygons[i]).vertices[1]].y;
      b[2] = (*cloud)[(triangles->polygons[i]).vertices[2]].z - (*cloud)[(triangles->polygons[i]).vertices[1]].z;
      la = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      lb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
      size_t ia = (size_t)(la / (sample_rate / 10.));
      size_t ib = (size_t)(lb / (sample_rate / 10.));
      // std::cout<< "ia:"<< ia << " " << ib << std::endl;

      for (size_t j = 0; j < ia; j += 1){
        for (size_t k = 0; k < (float)j / ia * ib; k += 1){
          pcl::PointXYZ point;
          point.x = (*cloud)[(triangles->polygons[i]).vertices[0]].x + a[0] * j / ia + b[0] * k / ib;
          point.y = (*cloud)[(triangles->polygons[i]).vertices[0]].y + a[1] * j / ia + b[1] * k / ib;
          point.z = (*cloud)[(triangles->polygons[i]).vertices[0]].z + a[2] * j / ia + b[2] * k / ib;
          if(point.z != 0){
            P -> push_back(point);
          }
        }
      }
    }
  }
  std::cerr<< "face need:"<< rec.size() << std::endl;
  std::cerr<< "P size before:"<< P->size() << std::endl;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(sample_rate, sample_rate, sample_rate);
  grid.setInputCloud(P);
  grid.filter(*P);
  std::cerr<< "P size after:"<< P->size() << std::endl;
  ////////////////////fake code //////////////////////////////
  pcl::io::savePCDFileASCII ("tmp.pcd", *P + *cloud);
  // pcl::io::savePCDFileASCII ("tmp.pcd", *P);
  ////////////////////////////////////////////////////////////

  return 0;
}

double cone_mesh_intersect(cone a, MeshPtr triangles, std::vector<tri_data> &preprocess_tri, Eigen::Vector3f &p){
  // intersect a cone with a mesh
  // by finding the nearst point on each triangle
  // and check whether it's inside the cone
  // return the distance
  float tan_a = tan(a.theta);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  fromPCLPointCloud2(triangles->cloud, *cloud);

  float m = std::numeric_limits<float>::infinity();
  std::vector<int> main_list(3);
  main_list[0] = 12;
  main_list[1] = 13;
  main_list[2] = 23;

  float d;
  Eigen::Vector3f tmp_p;
  int index, tmp_sum;
  for (size_t i = 0; i < triangles->polygons.size(); i += 1){
    if(!preprocess_tri[i].ok){
      continue;
    }
    Eigen::Vector3f p_trans = preprocess_tri[i].tri_trans * a.pos;
    // Eigen::Vector3f p_yz(0, p_trans(1), p_trans(2));
    std::vector<int> record;
    for (size_t j = 0; j < 3; j += 1){
      if (bool(sub_in(p_trans(1), p_trans(2), preprocess_tri[i].L[main_list[j]]) >= 0) == preprocess_tri[i].in_tri[main_list[j]]){
        record.push_back(main_list[j]);
      }
    }
    if (record.size() == 3){  // in the triangle
      d = std::abs(p_trans(0));
      tmp_p = Eigen::Vector3f(0, p_trans(1), p_trans(2));
    }
    else if(record.size() == 2){ // nearst to another line
      tmp_sum = record[0] + record[1];
      if(tmp_sum == 12 + 13){
        index = 23;
      }
      else if(tmp_sum == 12 + 23){
        index = 13;
      }
      else if(tmp_sum == 13 + 23){
        index = 12;
      }
      float c_n = (preprocess_tri[i].L[index](1) * p_trans(1)+ preprocess_tri[i].L[index](0) * p_trans(2)) * -1;
      float a_sq_plus_b_sq = pow(preprocess_tri[i].L[index](0), 2) + pow(preprocess_tri[i].L[index](1), 2);
      tmp_p = Eigen::Vector3f(0,
        (-preprocess_tri[i].L[index](0) * preprocess_tri[i].L[index](2) - preprocess_tri[i].L[index](1) * c_n) / a_sq_plus_b_sq,
        (preprocess_tri[i].L[index](0) * c_n - preprocess_tri[i].L[index](1) * preprocess_tri[i].L[index](2)) / a_sq_plus_b_sq);
      d = d_v3(tmp_p, p_trans);
    }
    else if(record.size() == 1){// nearst to another point
      if(record[0] == 12){
        index = 3 - 1;
      }
      else if(record[0] == 13){
        index = 2 - 1;
      }
      else if(record[0] == 23){
        index = 1 - 1;
      }
      tmp_p = Eigen::Vector3f(preprocess_tri[i].tri_after(0, index), preprocess_tri[i].tri_after(1, index), preprocess_tri[i].tri_after(2, index));
      d = d_v3(tmp_p, p_trans);
      // line_point
    }
    else{
      // d+=1;
      std::cerr<< "i " << i << " "<< record.size() << std::endl;
      std::cerr<< "p_trans " << p_trans << std::endl;
      std::cerr<< "1 f\n " << preprocess_tri[i].L[main_list[0]] << std::endl;
      std::cerr<< "2 f\n " << preprocess_tri[i].L[main_list[1]] << std::endl;
      std::cerr<< "3 f\n " << preprocess_tri[i].L[main_list[2]] << std::endl;

      std::cerr<< "n 1\n " << sub_in(p_trans(1), p_trans(2), preprocess_tri[i].L[main_list[0]]) << std::endl;
      std::cerr<< "n 2\n " << bool(sub_in(p_trans(1), p_trans(2), preprocess_tri[i].L[main_list[1]]) >=0) << std::endl;
      std::cerr<< "n 3\n " << sub_in(p_trans(1), p_trans(2), preprocess_tri[i].L[main_list[2]]) << std::endl;

      std::cerr<< " a1\n " << preprocess_tri[i].in_tri[main_list[0]] << std::endl;
      std::cerr<< " a2\n " << preprocess_tri[i].in_tri[main_list[1]] << std::endl;
      std::cerr<< " a3\n " << preprocess_tri[i].in_tri[main_list[2]] << std::endl;
      if (bool(sub_in(p_trans(1), p_trans(2), preprocess_tri[i].L[main_list[1]]) >=0) == preprocess_tri[i].in_tri[main_list[1]])
      {
          std::cerr<< "ok " << std::endl;
      }
      exit(1);
    }
    tmp_p = preprocess_tri[i].tri_trans.inverse() * tmp_p;

    //check whether it's in the cone
    float h = a.pos[2] - tmp_p(2);
    if(h >= 0){
      if(tan_a * h >= sqrt(pow(tmp_p[0] - a.pos[0], 2) + pow(tmp_p[1] - a.pos[1], 2))){
        if(d < m){
          m = d;
          p = tmp_p;
          // std::cerr<< "i " << i << " "<< record.size() << std::endl;
        }
      }
    }

  }
  // std::cerr<< "m " << m << std::endl;
  // std::cerr<< "tmp_p " << tmp_p << std::endl;
  // std::cerr<< "p " << p << std::endl;

  return m;
}

double cone_intersect(cone a, cone b, cone &c){
  // computer the cone-cone intersection
  // usint inner division point
  // return distance between cone a's position and interseciton point

  float dz = b.pos[2] - a.pos[2];
  float dxy = sqrt(pow(a.pos[0] - b.pos[0], 2) + pow(a.pos[1] - b.pos[1], 2));
  if(dxy == 0){
    if(dz == 0){
      c = a;
      return 0;
    }
    return -1;
  }
  float h;
  // std::cout<< "tan " << tan(a.theta) << std::endl;
  // std::cout<< "dxy " << dxy << std::endl;
  // std::cout<< "dz " << dz << std::endl;
  h = (dxy / tan(a.theta) - dz) / 2;
  // std::cout<< "h:" << h << std::endl;

  c.pos[0] = ((h + dz) * a.pos[0] + h * b.pos[0]) / (h + dz + h),
  c.pos[1] = ((h + dz) * a.pos[1] + h * b.pos[1]) / (h + dz + h),
  c.pos[2] = a.pos[2] - h;
  c.theta = a.theta;
  return d_v3(a.pos, c.pos);
  // return sqrt(pow(a.pos[0] - c.pos[0], 2) + pow(a.pos[1] - c.pos[1], 2) + pow(a.pos[2] - c.pos[2], 2));
}

Eigen::Matrix3f find_strut_tri(float R, float shrink_d, Eigen::Vector3f start, Eigen::Vector3f end){
  Eigen::Matrix3f tri;
  shrink_d = 0.1 * d_v3(start, end);
  tri(0, 0) = R;
  tri(1, 0) = 0;
  tri(2, 0) = 0;

  tri(0, 1) = R * cos(M_PI * 2 / 3 * 2);
  tri(1, 1) = R * sin(M_PI * 2 / 3 * 2);
  tri(2, 1) = 0;

  tri(0, 2) = R * cos(M_PI * 2 / 3);
  tri(1, 2) = R * sin(M_PI * 2 / 3);
  tri(2, 2) = 0;


  // Eigen::Vector3f v(0, 0, 1);
  // Eigen::Quaternionf a;
  // // std::cerr<< "end - start " << end - start << std::endl;
  // a = a.FromTwoVectors(v, end - start);
  // v = (end - start).normalized();
  // // std::cerr<< "tri " << tri << std::endl;
  // // std::cerr<< "av " << a * v << std::endl;
  // // std::cerr<< "cc " << a * tri<< std::endl;

  Eigen::Vector3f v = (end - start).normalized() * shrink_d;

  // tri = a * tri;
  for (size_t i = 0; i < 3; i += 1){
    for (size_t j = 0; j < 3; j += 1){
      tri(j, i) += start(j) + v(j);
    }
  }
  // if(change){
  //   Eigen::Vector3f tmp = tri.col(1);
  //   tri.col(1) = tri.col(2);
  //   tri.col(2) = tmp;
  // }
  return tri;
}

int connect_tri(Eigen::Matrix3f &tri1, Eigen::Matrix3f &tri2, pcl::PointCloud<pcl::PointXYZ>::Ptr strut_point, MeshPtr &strut_stl){
  int index = strut_point->size();
  for (size_t i = 0; i < 3; i += 1){
    strut_point->push_back(pcl::PointXYZ(tri1(0, i), tri1(1, i), tri1(2, i)));
  }

  for (size_t i = 0; i < 3; i += 1){
    strut_point->push_back(pcl::PointXYZ(tri2(0, i), tri2(1, i), tri2(2, i)));
  }
  int index_mapping[6][3] = {
    {0, 2, 3},
    {1, 4, 5},
    {1, 5, 2},
    {2, 5, 3},
    {0, 3, 4},
    {0, 4, 1}
  };
  for (size_t i = 0; i < 6; i += 1){
    pcl::Vertices v;
    v.vertices.resize(3);
    for (size_t j = 0; j < 3; j += 1){
      v.vertices[j] = index + index_mapping[i][j];
    }
    strut_stl->polygons.push_back(v);
  }
  return 0;
}

int add_triangle(Eigen::Matrix3f &tri, pcl::PointCloud<pcl::PointXYZ>::Ptr strut_point, MeshPtr &strut_stl){
  // simply add a triangle into mesh
  for (size_t i = 0; i < 3; i += 1){
    strut_point->push_back(pcl::PointXYZ(tri(0, i), tri(1, i), tri(2, i)));
  }
  pcl::Vertices v;
  v.vertices.resize(3);
  v.vertices[0] = strut_point -> size() - 3;
  v.vertices[1] = strut_point -> size() - 2;
  v.vertices[2] = strut_point -> size() - 1;

  strut_stl->polygons.push_back(v);
  return 0;
}

int re_strut(std::vector<int> input, int from, Eigen::Matrix3f tri, SupportTree &support_tree, MeshPtr &strut_stl, pcl::PointCloud<pcl::PointXYZ>::Ptr strut_point, pcl::PointCloud<pcl::PointXYZ>::Ptr P){
  // recursively generate strut
  float R = 1, shrink_d = 2;
  Eigen::Matrix3f new_tri1, new_tri2;
  for (size_t i = 0; i < input.size(); i += 1){
    if (support_tree.tree[input[i]].left >= 0){ // split into 2


      // big triangle
      new_tri1(0, 0) = P->points[support_tree.tree[input[i]].index].x + R;
      new_tri1(1, 0) = P->points[support_tree.tree[input[i]].index].y + 0;
      new_tri1(2, 0) = P->points[support_tree.tree[input[i]].index].z;

      //  fake code
      // new_tri1(0, 1) = P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3 );
      // new_tri1(1, 1) = P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3 );
      // new_tri1(2, 1) = P->points[support_tree.tree[input[i]].index].z;

      // new_tri1(0, 2) = P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3 * 2);
      // new_tri1(1, 2) = P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3 * 2);
      // new_tri1(2, 2) = P->points[support_tree.tree[input[i]].index].z;
      // add_triangle(new_tri1, strut_point, strut_stl);
      //

      new_tri1(0, 1) = P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3 * 2);
      new_tri1(1, 1) = P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3 * 2);
      new_tri1(2, 1) = P->points[support_tree.tree[input[i]].index].z;

      new_tri1(0, 2) = P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3);
      new_tri1(1, 2) = P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3);
      new_tri1(2, 2) = P->points[support_tree.tree[input[i]].index].z;

      connect_tri(tri, new_tri1, strut_point, strut_stl);

      Eigen::Vector3f left(P->points[support_tree.tree[input[i]].left].x, P->points[support_tree.tree[input[i]].left].y, P->points[support_tree.tree[input[i]].left].z);
      Eigen::Vector3f right(P->points[support_tree.tree[input[i]].right].x, P->points[support_tree.tree[input[i]].right].y, P->points[support_tree.tree[input[i]].right].z);
      Eigen::Vector3f t(P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3), P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3), P->points[support_tree.tree[input[i]].index].z);

      // determine which connect to which, swap if needed
      if (d_v3(left, t) > d_v3(right, t)){
        int tmp;
        tmp = support_tree.tree[input[i]].left;
        support_tree.tree[input[i]].left = support_tree.tree[input[i]].right;
        support_tree.tree[input[i]].right = tmp;
      }


      // split triangle 1
      new_tri1(0, 0) = P->points[support_tree.tree[input[i]].index].x + R;
      new_tri1(1, 0) = P->points[support_tree.tree[input[i]].index].y + 0;
      new_tri1(2, 0) = P->points[support_tree.tree[input[i]].index].z;

      new_tri1(0, 1) = P->points[support_tree.tree[input[i]].index].x - 0.5 * R;
      new_tri1(1, 1) = P->points[support_tree.tree[input[i]].index].y;
      new_tri1(2, 1) = P->points[support_tree.tree[input[i]].index].z;

      new_tri1(0, 2) = P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3);
      new_tri1(1, 2) = P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3);
      new_tri1(2, 2) = P->points[support_tree.tree[input[i]].index].z;
      std::vector<int> new_input1;
      new_input1.push_back(support_tree.tree[input[i]].left);
      re_strut(new_input1, input[i], new_tri1, support_tree, strut_stl, strut_point, P);

      // split triangle 2
      new_tri2(0, 0) = P->points[support_tree.tree[input[i]].index].x + R;
      new_tri2(1, 0) = P->points[support_tree.tree[input[i]].index].y + 0;
      new_tri2(2, 0) = P->points[support_tree.tree[input[i]].index].z;

      new_tri2(0, 1) = P->points[support_tree.tree[input[i]].index].x + R * cos(M_PI * 2 / 3 * 2);
      new_tri2(1, 1) = P->points[support_tree.tree[input[i]].index].y + R * sin(M_PI * 2 / 3 * 2);
      new_tri2(2, 1) = P->points[support_tree.tree[input[i]].index].z;

      new_tri2(0, 2) = P->points[support_tree.tree[input[i]].index].x - 0.5 * R;
      new_tri2(1, 2) = P->points[support_tree.tree[input[i]].index].y;
      new_tri2(2, 2) = P->points[support_tree.tree[input[i]].index].z;
      std::vector<int> new_input2;
      new_input2.push_back(support_tree.tree[input[i]].right);
      re_strut(new_input2, input[i], new_tri2, support_tree, strut_stl, strut_point, P);
    }
    else if(support_tree.tree[input[i]].left == -1){ // meet support point
      Eigen::Vector3f start = Eigen::Vector3f(P->points[support_tree.tree[input[i]].index].x, P->points[support_tree.tree[input[i]].index].y, P->points[support_tree.tree[input[i]].index].z);
      new_tri1 = find_strut_tri(R, shrink_d, start, Eigen::Vector3f(P->points[support_tree.tree[from].index].x, P->points[support_tree.tree[from].index].y, P->points[support_tree.tree[from].index].z));
      connect_tri(tri, new_tri1, strut_point, strut_stl);
      Eigen::Vector3f tmp;
      int tmp_index = strut_point -> size();

      strut_point -> push_back(pcl::PointXYZ(start(0), start(1), start(2)));
      for (size_t j = 0; j < 3; j += 1){
        tmp = new_tri1.block<3, 1>(0, j);
        strut_point -> push_back(pcl::PointXYZ(tmp(0), tmp(1), tmp(2)));
      }

      for (size_t j = 0; j < 3; j += 1){
        pcl::Vertices v;
        v.vertices.resize(3);
        v.vertices[0] = tmp_index;
        v.vertices[1] = tmp_index + ((j + 1) % 3) + 1;
        v.vertices[2] = tmp_index + ((j + 0) % 3) + 1;
        strut_stl->polygons.push_back(v);
      }

    }
  }
  return 0;
}

int genearte_strut(pcl::PointCloud<pcl::PointXYZ>::Ptr P, SupportTree &support_tree, MeshPtr &strut_stl){
  std::vector<int> input;
  pcl::PointCloud<pcl::PointXYZ>::Ptr strut_point(new pcl::PointCloud<pcl::PointXYZ>);
  float R, shrink_d = 2;
  for (size_t i = 0; i < support_tree.tree.size(); i += 1){
    // (*cloud)[(triangles->polygons[i]).vertices[1]].x
    if(support_tree.tree[i].left == -3 || support_tree.tree[i].left == -2){
      input.push_back(support_tree.tree[i].right);
      Eigen::Matrix3f tri;
      if (support_tree.tree[i].left == -3){// dealing with strut start from plate
        R = 1; // TODO: how to determine R, radius
        tri(0, 0) = P->points[support_tree.tree[i].index].x + R;
        tri(1, 0) = P->points[support_tree.tree[i].index].y + 0;
        tri(2, 0) = 0;

        tri(0, 1) = P->points[support_tree.tree[i].index].x + R * cos(M_PI * 2 / 3 * 2);
        tri(1, 1) = P->points[support_tree.tree[i].index].y + R * sin(M_PI * 2 / 3 * 2);
        tri(2, 1) = 0;

        tri(0, 2) = P->points[support_tree.tree[i].index].x + R * cos(M_PI * 2 / 3);
        tri(1, 2) = P->points[support_tree.tree[i].index].y + R * sin(M_PI * 2 / 3);
        tri(2, 2) = 0;
        add_triangle(tri, strut_point, strut_stl);

      }
      else{ // dealing with strut start from mesh
        Eigen::Vector3f start = Eigen::Vector3f(P->points[support_tree.tree[i].index].x, P->points[support_tree.tree[i].index].y, P->points[support_tree.tree[i].index].z);
        tri = find_strut_tri(R, shrink_d, start, Eigen::Vector3f(P->points[support_tree.tree[i].right].x, P->points[support_tree.tree[i].right].y, P->points[support_tree.tree[i].right].z));

        Eigen::Vector3f tmp;
        int tmp_index = strut_point -> size();

        strut_point -> push_back(pcl::PointXYZ(start(0), start(1), start(2)));
        for (size_t j = 0; j < 3; j += 1){
          tmp = tri.block<3, 1>(0, j);
          strut_point -> push_back(pcl::PointXYZ(tmp(0), tmp(1), tmp(2)));
        }

        for (size_t j = 0; j < 3; j += 1){
          pcl::Vertices v;
          v.vertices.resize(3);
          v.vertices[0] = tmp_index;
          v.vertices[1] = tmp_index + ((j + 0) % 3) + 1;
          v.vertices[2] = tmp_index + ((j + 1) % 3) + 1;

          strut_stl->polygons.push_back(v);
        }
      }
      re_strut(input, i, tri, support_tree, strut_stl, strut_point, P);
      input.clear();
    }
  }
  toPCLPointCloud2(*strut_point, strut_stl->cloud);
  return 0;
}
