#include "path_vector.h"
#include <vector>
#include <string>
#include "g2f_module.h"

std::string path_to_js(std::vector< std::vector< std::vector<float> > > output);
std::string path_to_js_cpp(std::vector< std::vector< PathVector > >* output);