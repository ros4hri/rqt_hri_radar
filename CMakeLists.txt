# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    src/rqt_human_radar/SimUi.cpp
    src/rqt_human_radar/InteractiveView.cpp
    src/rqt_human_radar/SimScene.cpp
    src/rqt_human_radar/SemanticObject.cpp
    src/rqt_human_radar/SimItem.cpp
    src/rqt_human_radar/PersonItem.cpp
    src/rqt_human_radar/LocalObjectItem.cpp
    src/rqt_human_radar/RemoteObjectItem.cpp
    src/rqt_human_radar/RemotePersonItem.cpp
    src/rqt_human_radar/LocalPersonItem.cpp
    src/rqt_human_radar/FieldOfViewItem.cpp
)
###########


###########
# headers #
###########

# only the QObject derived classes need to be listed here
set(rqt_human_radar_HDRS
    include/rqt_human_radar/Radar.hpp
    include/rqt_human_radar/SimUi.hpp
    include/rqt_human_radar/InteractiveView.hpp
    include/rqt_human_radar/SimScene.hpp
    # include/rqt_human_radar/SemanticObject.hpp
    # include/rqt_human_radar/SimItem.hpp
    # include/rqt_human_radar/PersonItem.hpp
    include/rqt_human_radar/LocalObjectItem.hpp
    # include/rqt_human_radar/RemoteObjectItem.hpp
    # include/rqt_human_radar/RemotePersonItem.hpp
    # include/rqt_human_radar/LocalPersonItem.hpp
    # include/rqt_human_radar/FieldOfViewItem.hpp
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

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
