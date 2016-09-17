#include <vector>
#include <string>
#include "path_vector.h"

using namespace std;

#ifndef G2FHeader

#define G2FHeader

typedef FILE* FilePtr;
FilePtr open_gcode(char* gcode_path);

typedef enum{
  TYPE_NEWLAYER,
  TYPE_INFILL,
  TYPE_PERIMETER,
  TYPE_SUPPORT,
  TYPE_MOVE,
  TYPE_SKIRT,
  TYPE_INNERWALL,
  TYPE_RAFT,
  TYPE_SKIN
} PathType;

typedef struct{
  void* ptr;
} Cursor;


typedef struct{
  int tool; 
  char absolute;
  char extrude_absolute;
  float unit;
  float current_speed;
  float G92_delta[7];
  float time_need;
  float distance;
  float max_range[4];
  float filament[3];
  float previous[3];
  float current_pos[7];
  char* HEAD_TYPE;
  int layer_now;
 
  PathType path_type;
  char is_cura;
  vector< vector<PathVector> >* native_path;

  int counter_between_layers;
  float record_z;

  char record_path;

  int index;
  //image
  //config = None  # config dict(given from fluxstudio)

} FCode;

FCode* createFCodePtr();
int convert_to_fcode_by_line(char* line, FCode* fc, char* fcode_output);
void trim_ends_cpp(vector< vector< PathVector > >* output);

#endif