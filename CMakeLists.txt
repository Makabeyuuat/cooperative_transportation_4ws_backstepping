cmake_minimum_required(VERSION 2.8.3)
project(cooperative_transportation_4ws_backstepping)

# -------- コンパイラ設定 --------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")

# -------- 依存 --------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  gazebo_plugins
)

# -------- catkin_package --------
#  - include/ を公開
#  - ライブラリ: kinematics_solver（骨格）と mathematica（200本のカーネル）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinematics_solver mathematica
  CATKIN_DEPENDS roscpp rospy std_msgs tf tf2 tf2_ros gazebo_plugins
)

# -------- インクルードパス --------
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  /usr/include/eigen3
)

# -------- 200本の .cpp → “mathematica” ライブラリに集約 --------
file(GLOB_RECURSE MATHEMATICA_SRCS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/mathematica/*.cpp")

add_library(mathematica STATIC ${MATHEMATICA_SRCS})
target_include_directories(mathematica PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
# 共有ライブラリに取り込まれても安全なようにPICを付与
set_target_properties(mathematica PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mathematica
  ${catkin_LIBRARIES}
)

# -------- ソルバ本体（ディスパッチ/骨格） --------
add_library(kinematics_solver
  src/kinematics_solver.cpp
)
target_include_directories(kinematics_solver PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(kinematics_solver
  mathematica
  ${catkin_LIBRARIES}
)

# -------- 既存ノード --------
add_executable(cooperative_transportation_dynamics_controller
  src/cooperative_transportation_dynamics.cpp
  src/vehicle_ct.cpp
  src/callback_ct.cpp
  src/csvLogger.cpp
  src/Bezier.cpp
  src/differential_equations.cpp
  src/getInputValue.cpp
  src/mathFunc.cpp
  src/DynamicsCalc.cpp
  src/initial.cpp
  src/dynamics_integrator_matrix.cpp
)



# ← kinematics_solver を使う場合はリンク
target_link_libraries(cooperative_transportation_dynamics_controller
  kinematics_solver
  ${catkin_LIBRARIES}
)
