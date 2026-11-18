# prints the ip link arguments for the bus settings in frame.hpp, used by can0-up.service
add_executable(can_link_args can_link_args.cpp)
target_link_libraries(can_link_args PRIVATE chuds chuds_warnings)

# sends and prints chuds messages by name instead of raw bytes
add_executable(chudsctl chudsctl.cpp)
target_link_libraries(chudsctl PRIVATE chuds_socketcan chuds_warnings)
