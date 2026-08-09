cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# 리소스 디렉토리를 share/go2_description 아래로 설치
install(
  DIRECTORY urdf meshes dae config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
