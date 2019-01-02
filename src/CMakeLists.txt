#	"MaxMinDistSet.cpp"
#	"MaxMinDistSet.h"

set(SOURCE_FILES 
    "corSkinCluster.cpp" 
	"corSkinCluster.h"
	"Grid.cpp"
	"Grid.h"
	"Cluster.cpp"
	"Cluster.h"
	"FindANN.cpp"
	"FindANN.h"
	"corSkinCmd.cpp"
	"corSkinCmd.h"
	"pluginMain.cpp"
)

# setup CGAL with config mode
#if(NOT DEFINED CGAL_DIR)
	#set(CGAL_DIR "C:\\Dev\\CGAL-4.12.1" CACHE STRING "CGALConfig.cmake location")
#endif()

#if(NOT DEFINED Eigen3_DIR)
#	set(Eigen3_DIR "C:\\Dev\\eigen-eigen-b3f3d4950030\\cmake" CACHE STRING "Eigen3Config.cmake locations")
#endif()

find_package(Maya REQUIRED)
find_package(OpenCL REQUIRED)
#set(MMDS_INC "${CMAKE_INSTALL_PREFIX}/MaxMinDistSet/include")
#set(MMDS_LIB "${CMAKE_INSTALL_PREFIX}/MaxMinDistSet/lib/static")
#find_package(CGAL REQUIRED)
find_package(ANN REQUIRED)
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${MAYA_INCLUDE_DIR} ${ANN_INCLUDE_DIR})
#include_directories(${MAYA_INCLUDE_DIR} ${CGAL_INCLUDE_DIR} ${ANN_INCLUDE_DIR})
#include_directories(${MAYA_INCLUDE_DIR} ${MMDS_INC})

link_directories(${MAYA_LIBRARY_DIR} ${ANN_LIBRARY_DIR})
#link_directories(${MAYA_LIBRARY_DIR} ${CGAL_LIBRARY_DIR} ${ANN_LIBRARY_DIR})
#link_directories(${MAYA_LIBRARY_DIR} ${MMDS_LIB})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES} ${ANN_LIBRARY} )
#target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES} ${CGAL_LIBRARIES} ${ANN_LIBRARY} Eigen3::Eigen)
#target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES} libMaxMinDistSet)

MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins)

