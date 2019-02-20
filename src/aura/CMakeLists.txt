cmake_minimum_required(VERSION 2.8.3)
project(aura)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        cv_bridge
        )

find_package(OpenCV REQUIRED)


add_executable(parrot1 src/controller.cpp include/aura/controller.h src/trajectory.cpp include/aura/trajectory.h src/test1.cpp)
add_executable(parrot src/controller.cpp include/aura/controller.h src/image_process.cpp
        include/aura/image_process.h src/qr_handler.cpp include/aura/qr_handler.h src/trajectory.cpp include/aura/trajectory.h src/test2.cpp)
add_executable(parrot2 src/controller.cpp include/aura/controller.h src/qr_handler.cpp include/aura/qr_handler.h
        src/trajectory.cpp include/aura/trajectory.h src/test4.cpp)
add_executable(test_code src/test.cpp)
add_executable(test_code1 src/test3.cpp)
#add_executable(image_process src/image_process.cpp include/aura/image_process.h)
#add_executable(qr_handler src/qr_handler.cpp include/aura/qr_handler.h)

target_link_libraries(parrot ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(parrot1 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(parrot2 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(test_code ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(test_code1 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(image_process ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(qr_handler ${catkin_LIBRARIES})

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES aura
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
