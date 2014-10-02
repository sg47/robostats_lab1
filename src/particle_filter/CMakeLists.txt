cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

find_package(Armadillo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
      roscpp
)


catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
            roscpp
      DEPENDS ARMADILLO
)

include_directories(include
      ${catkin_INCLUDE_DIRS}
      ${ARMADILLO_INCLUDE_DIRS}
)

link_directories(
      ${catkin_LIBRARY_DIRS}
      ${ARMADILLO_LIBRARY_DIRS}
)

add_definitions(
  ${catkin_DEFINITIONS}
  ${ARMADILLO_DEFINITIONS}
)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
      add_definitions(-DARMA_NO_DEBUG)
endif()

add_library(${PROJECT_NAME}
      src/ParticleFilter.cpp
)

target_link_libraries(${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${ARMADILLO_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
      src/${PROJECT_NAME}.cpp
)

set_target_properties(${PROJECT_NAME}_node
    PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
      ${PROJECT_NAME}
)


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
)
