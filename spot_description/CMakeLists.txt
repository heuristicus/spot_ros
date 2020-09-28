cmake_minimum_required(VERSION 3.5)
project(spot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf meshes
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_package()
