# the demo uses SocketCAN (Linux)
add_executable(chuds_demo chuds_demo.cpp)
target_link_libraries(chuds_demo PRIVATE chuds_socketcan chuds_warnings)
