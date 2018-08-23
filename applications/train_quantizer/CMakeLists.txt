cmake_minimum_required(VERSION 2.8.3)
project(train_quantizer)

set(CATKIN_DEVEL_PREFIX $ENV{HOME}/maplab_ws/devel)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CATKIN_DEVEL_PREFIX}/share/")


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


#######
# APP #
#######
cs_add_executable(train_quantizer
  app/train_quantizer-app.cc
)


############
## EXPORT ##
############
cs_install()
cs_export()
