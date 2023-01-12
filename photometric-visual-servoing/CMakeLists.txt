

project(direct-visual-servoing)

cmake_minimum_required(VERSION 2.8.12.2)

find_package(VISP REQUIRED visp_core visp_robot visp_visual_features visp_io visp_gui visp_vs visp_vision)
if(VISP_FOUND)
	include(${VISP_USE_FILE})
endif(VISP_FOUND)

#for linking with ueye
set (CMAKE_MODULE_PATH /opt/ids/ueye/src/ids/ueyedemo/_cmake/ ${CMAKE_MODULE_PATH})
find_package(ueyeapi REQUIRED)

set(example_cpp
  photometricVisualServoing
  photometricVisualServoingWithoutVpServo
)

set(src_cpp
../src/Camera/CamuEye.cpp
  ../src/Camera/CamuEyeException.cpp
  
  ../src/Robot/C_Staubli.cpp)

#MESSAGE(${LIBUEYEAPI})


foreach(cpp ${example_cpp})
	# target 
	add_executable(${cpp} ${cpp}.cpp ${src_cpp})
	
	
	
	target_link_libraries(${cpp} ${UEYE_LIBRARY})
	
endforeach()

#visp_set_source_file_compile_flag(photometricVisualServoing.cpp -Wno-strict-overflow)
#visp_set_source_file_compile_flag(photometricVisualServoingWithoutVpServo.cpp -Wno-strict-overflow)

#add_test(photometricVisualServoing photometricVisualServoing -c -n 20 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(photometricVisualServoingWithoutVpServo photometricVisualServoingWithoutVpServo -c -n 20 ${OPTION_TO_DESACTIVE_DISPLAY})

