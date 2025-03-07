cmake_minimum_required(VERSION 3.8)
project(kalman_filter)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)
## Debugging Flags
#add_compile_options(-march=native -g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer)
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
## Release Flags
add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE)
## Profiling Flags
#add_compile_options(-fno-inline -fno-omit-frame-pointer -O3 -march=native -DNDEBUG)
#add_link_options(-Wl,--no-as-needed -lprofiler -Wl,--as-needed)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(matplotlibcpp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(LAPACK REQUIRED)
find_package(TBB REQUIRED)
set(TBB_USE_TBBMALLOC OFF)

###########
## Build ##
###########

## Rocket Sled Demo
add_executable(rocket_sled_demo
  src/rocket_sled_demo.cpp
)
target_include_directories(rocket_sled_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rocket_sled_demo
  rclcpp
  matplotlibcpp
  OpenMP
  Eigen3
  TBB
  Boost
)
target_link_libraries(rocket_sled_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Finite Time LQR Evaluation
add_executable(finite_time_lqr_evaluation
  src/finite_time_lqr_evaluation.cpp
)
target_include_directories(finite_time_lqr_evaluation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(finite_time_lqr_evaluation
  rclcpp
  matplotlibcpp
  OpenMP
  Eigen3
  TBB
  Boost
)
target_link_libraries(finite_time_lqr_evaluation
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Gauss Markov Fitting Demo
add_executable(gauss_markov_fitting_demo
  src/gauss_markov_fit_demo.cpp
)
target_include_directories(gauss_markov_fitting_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gauss_markov_fitting_demo
  rclcpp
  matplotlibcpp
  OpenMP
  Eigen3
  TBB
  Boost
)
target_link_libraries(gauss_markov_fitting_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

############
## Export ##
############

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  rclcpp
  matplotlibcpp
  Eigen3
  OpenMP
  TBB
  Boost
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  rocket_sled_demo
  finite_time_lqr_evaluation
  gauss_markov_fitting_demo
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(${PROJECT_NAME}_tests
    normal_distribution_test
    feature_bearing_test
    feature_range_test
    dubins_airplane_imu_test
    dubins_airplane_model_test
    dubins_airplane_mapping_test
    dubins_airplane_controller_test
    differential_pressure_test
  )

  ament_add_gtest(normal_distribution_test test/noise/normal_distribution_test.cpp)
  ament_add_gtest(feature_bearing_test test/sensors/measurements/feature_bearing_test.cpp)
  ament_add_gtest(feature_range_test test/sensors/measurements/feature_range_test.cpp)
  ament_add_gtest(differential_pressure_test test/sensors/measurements/differential_pressure_test.cpp)
  ament_add_gtest(dubins_airplane_imu_test test/sensors/inertial_measurements/dubins_airplane_imu_test.cpp)
  ament_add_gtest(dubins_airplane_model_test test/dynamics/dubins_airplane_model_test.cpp)
  ament_add_gtest(dubins_airplane_mapping_test test/mappings/dubins_airplane_mapping_test.cpp)
  ament_add_gtest(dubins_airplane_controller_test test/controllers/dubins_airplane_controller_test.cpp)

  foreach(test_it ${${PROJECT_NAME}_tests})
    target_include_directories(${test_it} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(${test_it}
      rclcpp
      matplotlibcpp
      Eigen3
      OpenMP
      TBB
      LAPACK
    )
    target_link_libraries(${test_it}
      OpenMP::OpenMP_CXX
      TBB::tbb
    )
  endforeach()
endif()

ament_package()

