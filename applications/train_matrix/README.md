train_matrix
=================
#train projection matrix:
functions copied from src/maplab/algorithms/loopclosure/descriptor-projection/src/train-projection-matrix.cc
use one or more vi-maps to train a projection matrix and save it at specified path
##from single vi-map
```
rosrun train_matrix train_matrix --vi_map_folder /path/to/list/map --lc_projection_matrix_filename /path/to/matrix/projection_matrix --feature_descriptor_type freak
```
##from multiple vi-maps
```
rosrun train_matrix train_matrix --vi_map_list /path/to/list/map_list.txt --lc_projection_matrix_filename /path/to/matrix/projection_matrix --feature_descriptor_type freak
```
map_list.txt is a file which contains all the vimaps's path like:
```
/persist/data/cla/cla_floor_f_freak
/persist/data/cla/cla_floor_g_freak
/persist/data/cla/cla_floor_h_freak
/persist/data/cla/cla_floor_j_freak
```
