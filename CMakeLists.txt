cmake_minimum_required(VERSION 3.0.2)
project(rqt_human_radar)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        rqt_gui
        rqt_gui_cpp
        hri
        tf
        )

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Svg REQUIRED)
else ()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif ()

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

set(rqt_human_radar_INCLUDE_DIRECTORIES
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        )

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        rqt_gui
        rqt_gui_cpp
        DEPENDS
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    qt5_wrap_cpp(rqt_human_radar_MOCS ${rqt_human_radar_HDRS})
    qt5_wrap_ui(rqt_human_radar_UIS_H ${rqt_human_radar_UIS})
else ()
    qt4_wrap_cpp(rqt_human_radar_MOCS ${rqt_human_radar_HDRS})
    qt4_wrap_ui(rqt_human_radar_UIS_H ${rqt_human_radar_UIS})
endif ()

include_directories(
        ${rqt_human_radar_INCLUDE_DIRECTORIES}
        ${catkin_INCLUDE_DIRS}
)


add_library(rqt_human_radar
        ${rqt_human_radar_SRCS}
        ${rqt_human_radar_MOCS}
        ${rqt_human_radar_UIS_H}
        )

target_link_libraries(rqt_human_radar
        ${catkin_LIBRARIES}
        )

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Svg)
else ()
    target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif ()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_human_radar)


install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS rqt_human_radar
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
