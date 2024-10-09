cmake_minimum_required(VERSION 3.14)

project(rqt_human_radar)

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Svg REQUIRED)

if(WIN32)
  message(STATUS "rqt_human_radar is not yet supported on Windows. Package will not be built.")
  ament_package()
  return()
endif()

###########
# sources #
###########
set(rqt_human_radar_SRCS
        src/rqt_human_radar/Radar.cpp
        src/rqt_human_radar/RadarScene.cpp
        src/rqt_human_radar/RadarCanvas.cpp
        )
###########


###########
# headers #
###########
set(rqt_human_radar_HDRS
        include/rqt_human_radar/Radar.hpp
        include/rqt_human_radar/RadarScene.hpp
        include/rqt_human_radar/RadarCanvas.hpp
        )
###########


############
# ui files #
############
set(rqt_human_radar_UIS
         resource/radar_tabs.ui
         )
############

qt5_wrap_cpp(rqt_human_radar_MOCS ${rqt_human_radar_HDRS})

qt5_wrap_ui(rqt_human_radar_UIS_H ${rqt_human_radar_UIS})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
        ${rqt_human_radar_SRCS}
        ${rqt_human_radar_MOCS}
        ${rqt_human_radar_UIS_H}
        )
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Svg)

ament_export_libraries(${PROJECT_NAME})


find_package(class_loader)
class_loader_hide_library_symbols(rqt_human_radar)

# Additional installs
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY res
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_auto_package()
