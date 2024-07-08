cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gtsam_test)

add_compile_options(-O3)
add_compile_options(-pg)

# find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
include_directories(${GTSAM_INCLUDE_DIR})

# For OpenMP
find_package(OpenMP REQUIRED)

if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable(gtsam_test src/main.cpp)
target_include_directories(gtsam_test PRIVATE ./include)
target_link_libraries(gtsam_test Eigen3::Eigen ${Python3_LIBRARIES} pybind11::embed matplotlibcpp17::matplotlibcpp17 gtsam)