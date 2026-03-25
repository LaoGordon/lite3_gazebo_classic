cmake_minimum_required(VERSION 3.8)
project(quadruped_playground)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY worlds config models
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
