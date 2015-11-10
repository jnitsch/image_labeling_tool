cmake_minimum_required(VERSION 2.8.3)
project(label_tool)

find_package(catkin REQUIRED COMPONENTS
  catkin_simple
)
catkin_simple()

set(QT_QMAKE_EXECUTABLE /usr/bin/qmake-qt4)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(HEADERS
  include/${PROJECT_NAME}/label_tool_rqt_plugin.h
  include/${PROJECT_NAME}/rqt/scene.h
)
include(${QT_USE_FILE})
include_directories( include
${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_BINARY_DIR}
)

set(SOURCES
  src/label_tool_rqt_plugin.cpp
  src/rqt/scene.cpp
  )


qt4_wrap_cpp(label_tool_rqt include/${PROJECT_NAME}/label_tool_rqt_plugin.h include/${PROJECT_NAME}/rqt/scene.h)
qt4_wrap_ui(label_tool_rqt_ui src/rqt/label_tool.ui)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS}  ${label_tool_rqt_ui} ${label_tool_rqt})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
add_dependencies(${PROJECT_NAME} label_tool_rqt_ui label_tool_rqt)






