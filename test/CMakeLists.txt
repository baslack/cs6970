include_directories("${PROJECT_SOURCE_DIR}/src")

# First Simple Google Test
add_executable(simple simple.cpp)
target_link_libraries(simple ${PROJECT_NAME} gtest gmock_main)
add_test(NAME simple_test COMMAND simple)

#MinDistTest
add_executable(test_GridCluster test_GridCluster.cpp)
target_link_libraries(test_GridCluster ${PROJECT_NAME} gtest gmock_main)
add_test (NAME test_GridCluster COMMAND test_GridCluster)