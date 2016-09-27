# This file is part of MultiCol-SLAM
#
# Copyright (C) 2015-2016 Steffen Urban <rurbste at googlemail.com>
# For more information see <https://github.com/urbste/MultiCol-SLAM>
#
# MultiCol-SLAM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MultiCol-SLAM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.

project (MultiCol-SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

IF(WIN32)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -openmp -W0 -MP -MTd ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -openmp -W0 -MP -MTd")
ELSE()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -O3 -march=native -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O3 -march=native -std=c++11")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.1 REQUIRED)
find_package(Pangolin REQUIRED)

# ==============================================================================
# Search OpenGV
# ============================================================================== 
FIND_PACKAGE(OpenGV QUIET HINTS ${OpenGV_DIR_HINTS})
IF(NOT OpenGV_FOUND)
  SET(OpenGV_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/include)
  SET(OpenGV_LIBRARIES opengv)
  MESSAGE("OpenGV include DIR ${OpenGV_INCLUDE_DIR}")
  INCLUDE_DIRECTORIES(${OpenGV_INCLUDE_DIR})
ENDIF()

# ==============================================================================
# Search g2o includes
# ============================================================================== 
FIND_PACKAGE(g2o QUIET HINTS ${g2o_DIR_HINTS})
IF(NOT g2o_FOUND)
  SET(g2o_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/g2o)
  SET(g2o_LIBRARIES g2o)
  MESSAGE("g2o include DIR ${g2o_INCLUDE_DIR}")
  INCLUDE_DIRECTORIES(${g2o_INCLUDE_DIR})
ENDIF()

# ==============================================================================
# Search Eigen, borrowed from openMVG
# this makes it a lot smoother for Windows
# ==============================================================================
# - internal by default, (Eigen version 3.2.9)
# - external if EIGEN_INCLUDE_DIR_HINTS is defined
# ============================================================================== 
IF(NOT DEFINED EIGEN_INCLUDE_DIR_HINTS)
  SET(EIGEN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/Eigen)
  MESSAGE("Eigen include DIR ${EIGEN_INCLUDE_DIR}")
  #SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ThirdParty/Eigen/cmake)
  #FIND_PACKAGE(Eigen3 REQUIRED)
  #MESSAGE("-- Eigen version ${EIGEN3_VERSION}: ${EIGEN3_INCLUDE_DIR}")
  #INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
  SET(MultiCol-SLAM_USE_INTERNAL_EIGEN ON)
  INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
ENDIF()


SET(MultiColHeaders
include/cam_model_omni.h
include/cam_system_omni.h
include/cConverter.h
include/cMultiFrame.h
include/cMultiFramePublisher.h
include/cMultiKeyFrame.h
include/cConverter.h
include/cMapPoint.h
include/cMap.h
include/cMapPublisher.h
include/cLocalMapping.h
include/cLoopClosing.h
include/cOptimizer.h
include/cORBVocabulary.h
include/cORBmatcher.h
include/cMultiFrame.h
include/cMultiKeyFrameDatabase.h
include/cSim3Solver.h
include/cSystem.h
include/cTracking.h
include/cMultiInitializer.h
include/cViewer.h
include/mdBRIEFextractorOct.h
include/misc.h
#include/MultiCol_cayley_jacobians.h
include/g2o_MultiCol_vertices_edges.h
include/g2o_MultiCol_sim3_expmap.h
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/ThirdParty
${PROJECT_SOURCE_DIR}/ThirdParty/g2o
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/cam_model_omni.cpp
src/cam_system_omni.cpp
src/cConverter.cpp
src/cMultiFrame.cpp
src/cMultiFramePublisher.cpp
src/cMultiKeyFrame.cpp
src/cConverter.cpp
src/cMapPoint.cpp
src/cMap.cpp
src/cMapPublisher.cpp
src/cOptimizer.cpp
src/cOptimizerLoopStuff.cpp
src/cMultiFrame.cpp
src/cMultiKeyFrameDatabase.cpp
src/cSim3Solver.cpp
src/cSystem.cpp
src/cMultiInitializer.cpp
src/mdBRIEFextractorOct.cpp
src/cTracking.cpp
src/cLocalMapping.cpp
src/cLoopClosing.cpp
src/cViewer.cpp
src/cORBmatcher.cpp
src/g2o_MultiCol_sim3_expmap.cpp
src/g2o_MultiCol_vertices_edges.cpp
src/misc.cpp
${MultiColHeaders}
)

IF(WIN32)
SET (G2OLIBS_DEBUG ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/Debug/g2o.lib)
SET (DBOW2LIBS_DEBUG  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/Debug/DBoW2.lib)
SET (OPENGVLIBS_DEBUG  ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/lib/Debug/opengv.lib) 
SET (RANDOMGENLIBS_DEBUG ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/lib/Debug/random_generators.lib)
    
SET (G2OLIBS ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/Release/g2o.lib)
SET (DBOW2LIBS ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/Release/DBoW2.lib)
SET (OPENGVLIBS ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/lib/Release/opengv.lib) 
SET (RANDOMGENLIBS ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/lib/Release/random_generators.lib)
ELSE()
SET (G2OLIBS ${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o.so)
SET (DBOW2LIBS ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW2/lib/libDBoW2.so)
SET (OPENGVLIBS ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/build/lib/librandom_generators.a
                ${PROJECT_SOURCE_DIR}/ThirdParty/OpenGV/build/lib/libopengv.a)
ENDIF()

IF(WIN32)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
optimized ${G2OLIBS}
optimized ${DBOW2LIBS}
optimized ${OPENGVLIBS}
optimized ${RANDOMGENLIBS}
debug ${G2OLIBS_DEBUG}
debug ${DBOW2LIBS_DEBUG}
debug ${OPENGVLIBS_DEBUG}
debug ${RANDOMGENLIBS_DEBUG})
ELSE()
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${G2OLIBS}
${DBOW2LIBS}
${OPENGVLIBS}
${RANDOMGENLIBS})
ENDIF()

# Build examples
MESSAGE("${PROJECT_NAME}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Lafida)
add_executable(multi_col_slam_lafida Examples/Lafida/mult_col_slam_lafida.cpp)
target_link_libraries(multi_col_slam_lafida ${PROJECT_NAME})