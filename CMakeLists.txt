cmake_minimum_required(VERSION 3.8)
project(laser_vins_republisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_uav_lib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_library(laser_uav_lib laser_uav_lib)

# Inclui diretórios de cabeçalhos
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}    
  ${laser_uav_lib_INCLUDE_DIRS}                       
)

# =====================================================
# ==================== VinsRepublisher ====================
# =====================================================

# Executável do vins_republisher
set(vins_republisher_executable_name vins_republisher)
add_executable(${vins_republisher_executable_name}
  src/vins_republisher_main.cpp
)

# Biblioteca do vins_republisher
set(vins_republisher_library_name vins_republisher_node)
add_library(${vins_republisher_library_name} SHARED
  src/vins_republisher_node.cpp
)

# Vincula dependências ao executável e à biblioteca do vins_republisher
ament_target_dependencies(${vins_republisher_executable_name}
  ${dependencies}
)
ament_target_dependencies(${vins_republisher_library_name}
  ${dependencies}
)

# Vincula a biblioteca ao executável
target_link_libraries(${vins_republisher_executable_name}
  ${vins_republisher_library_name}
  laser_uav_lib::attitude_converter                                             # Vincula a laser_uav_lib ao executável
)

# Registra o nó como um componente ROS2
rclcpp_components_register_nodes(${vins_republisher_library_name} "laser_vins_republisher::VinsRepublisherNode")

# ==================== Instalação ====================
# Instala as bibliotecas
install(TARGETS ${vins_republisher_library_name} ${vins_republisher_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instala os executáveis
install(TARGETS ${vins_republisher_executable_name} ${vins_republisher_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Instala os cabeçalhos
install(DIRECTORY include/
  DESTINATION include/
)

# Instala os diretórios de launch e params
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

# Configuração de testes (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finaliza o pacote
ament_package()
