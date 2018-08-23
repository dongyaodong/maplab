train_quantizer
=================
#train project quantizer:
functions copied from /src/maplab/algorithms/loopclosure/matching-based-loopclosure/src/train-vocabulary.cc
use one or more vi-maps to train a vocabulary and save it at specified path
##from single vi-map
```
rosrun train_quantizer train_quantizer --vi_map_folder /path/to/map/vi_map --lc_projection_matrix_filename /path/to/projection_matrix --feature_descriptor_type=brisk --lc_projected_quantizer_filename /path/to/save/vocabulary --lc_detector_engine=inverted_multi_index_product_quantization --lc_product_quantization_num_components=5 --lc_product_quantization_num_dim_per_component=1 --lc_product_quantization_num_words=16  --lc_num_descriptors_to_train=7000000
```
##from multiple vi-maps
```
rosrun train_quantizer train_quantizer --vi_map_list /path/to/maplist/map_list.txt --lc_projection_matrix_filename /path/to/projection_matrix --feature_descriptor_type=brisk --lc_projected_quantizer_filename /path/to/save/vocabulary --lc_detector_engine=inverted_multi_index_product_quantization --lc_product_quantization_num_components=5 --lc_product_quantization_num_dim_per_component=1 --lc_product_quantization_num_words=16  --lc_num_descriptors_to_train=7000000
```
map_list.txt is a file which contains all the vimaps's path like:
```
/persist/data/cla/cla_floor_f_brisk
/persist/data/cla/cla_floor_g_brisk
/persist/data/cla/cla_floor_h_brisk
/persist/data/cla/cla_floor_j_brisk
```