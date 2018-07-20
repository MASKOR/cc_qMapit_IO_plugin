cmake_minimum_required(VERSION 3.0)

option( INSTALL_QMAPIT_IO_PLUGIN "Check to install qMapitIO plugin" OFF )

if (INSTALL_QMAPIT_IO_PLUGIN)
  project( QMAPIT_IO_PLUGIN )
	set( CC_IS_IO_PLUGIN 1 )

#  set_target_properties(${PROJECT_NAME}  PROPERTIES
#      CXX_STANDARD 14
#  )

# For handeling pointclouds
  find_package(PCL QUIET REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

# To connect to the repo
  find_package(mapit)

  include_directories(
    src
    )
  add_subdirectory( src )

	include( ../../CMakePluginTpl.cmake )

# To use the PCL utils for pointclouds convertion
  find_package(QPCL_PLUGIN)
  include_directories(${QPCL_PLUGIN_UTILS_LIB_SOURCE_DIR}/utils)

  target_link_libraries(${PROJECT_NAME}
    ${mapit_interface}
    ${mapit_interface_cpp}
    ${mapit_standard_repository_factory}
    layertype_pointcloud2
    ${PCL_LIBRARIES}
    QPCL_PLUGIN_UTILS_LIB
    )
endif()
