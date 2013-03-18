cmake_minimum_required(VERSION 2.6)

project(SERVER)

# Source files
set(SRC stereoCamera.cpp utils.cc SensorData.cc server.cc)
# Program name
set(PROG server)

# Include Aria headers
include_directories(/usr/local/Aria/include/)
# Include ArNetworking headers
include_directories(/usr/local/Aria/ArNetworking/include/)
# Include OpenCV headers
include_directories(/usr/local/include/opencv/)
# Include headers for the stereocamera
include_directories(/usr/local/BumbleBee2/newPGR/pgrlibdcstereo)
include_directories(/usr/local/triclops/include)

# Aria libraries location
link_directories(/usr/local/Aria/lib/)
# Location of libraries for stereocamera
link_directories(/usr/local/BumbleBee2/newPGR/pgrlibdcstereo)
link_directories(/usr/local/triclops/lib)

# Create the executable
add_executable(${PROG} ${SRC})

# Required Aria libraries
set(LIBS_ARIA Aria ArNetworking pthread dl rt)
# Required OpenCV libraries
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc)
# Required libraries for stereocamera
set(StereoCamera_LIBS dc1394 pgrlibdcstereo triclops pnmutils)
# Link libraries
target_link_libraries(${PROG} ${LIBS_ARIA} 
  ${OpenCV_LIBS} ${StereoCamera_LIBS})
