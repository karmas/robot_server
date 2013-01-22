cmake_minimum_required(VERSION 2.6)

project(SERVER)

# Source files
set(SRC server.cc)
# Program name
set(PROG server)

# Include Aria headers
include_directories(/usr/local/Aria/include/)
# Include ArNetworking headers
include_directories(/usr/local/Aria/ArNetworking/include/)
# Aria libraries location
link_directories(/usr/local/Aria/lib/)

# Create the executable
add_executable(${PROG} ${SRC})

# Required Aria libraries
set(LIBS_ARIA Aria ArNetworking pthread dl rt)
# Link libraries
target_link_libraries(${PROG} ${LIBS_ARIA})
