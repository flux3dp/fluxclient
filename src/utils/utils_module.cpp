#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <list>
#include <numeric>

#include "utils_module.h"

std::string path_to_js(std::vector< std::vector< std::vector<float> > > path){
  char buf[50];
  std::string c_string("[");
  for (size_t layer = 0; layer < path.size(); layer += 1){
    c_string += "[";
      for (size_t point = 0; point < path[layer].size(); point += 1){
        // sprintf(buf, "[%g,%g,%g,%d]", path[layer][point][0], path[layer][point][1], path[layer][point][2], (int)path[layer][point][3]);
        sprintf(buf, "[%.2f,%.2f,%.2f,%d]", path[layer][point][0], path[layer][point][1], path[layer][point][2], (int)path[layer][point][3]);
        c_string += buf;
        c_string += ",";
      }
      c_string.erase(c_string.end() - 1);
    c_string += "],";
  }
  c_string.erase(c_string.end() - 1);
  c_string += "]";
  return c_string;
}

std::string path_to_js_cpp(vector< vector< PathVector > >* path){
  char buf[50];
  std::string sb("[");
  int m_reserve = 1024;
  sb.reserve(m_reserve);
  for (size_t layer = 0; layer < path->size(); layer += 1){
    sb+= "[";
    int layer_size = (*path)[layer].size();
    m_reserve += layer_size*50;
    sb.reserve(m_reserve);
    for (int i = 0; i < layer_size; i++){
      // if(((*path)[layer][i].x) < 0.01 && ((*path)[layer][i].y) < 0.01){
      //   if(i == layer_size-1){
      //     sprintf(buf, "[%.2f,%.2f,%.2f,%d]",(*path)[layer][i].x,(*path)[layer][i].y,(*path)[layer][i].z,(*path)[layer][i].path_type);
      //     sb+= buf;
      //   }
      //   continue;
      // }
      // sprintf(buf, "[%g,%g,%g,%d]",(*path)[layer][i][0],(*path)[layer][i][1],(*path)[layer][i][2], (int)(*fc->native_path)[layer][i][3]);
      sprintf(buf, "[%.2f,%.2f,%.2f,%d]",(*path)[layer][i].x,(*path)[layer][i].y,(*path)[layer][i].z,(*path)[layer][i].path_type - 1);
      sb+= buf;
      if(i != layer_size-1) sb+= ",";
    }

    sb+= "]";
    if(layer != path->size()-1){
      sb += ",";
    }
  }
  sb+= "]";
  fprintf(stderr, "path_to_js_cpp end, size = %d\n", sb.length());
  return sb;
}
