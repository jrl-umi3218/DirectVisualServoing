

project(photometric-visual-servoing)

cmake_minimum_required(VERSION 2.8.12.2)

find_package(VISP REQUIRED visp_core visp_robot visp_visual_features visp_io visp_gui visp_vs visp_vision)
if(VISP_FOUND)
	include(${VISP_USE_FILE})
endif(VISP_FOUND)

#for linking with ueye
if(DEFINED USE_IDS)
	set (CMAKE_MODULE_PATH /opt/ids/ueye/src/ids/ueyedemo/_cmake/ ${CMAKE_MODULE_PATH})
	find_package(ueyeapi REQUIRED)
endif()

#for compiling and linking with Flir
if(DEFINED USE_FLIR)
	INCLUDE_DIRECTORIES(/opt/spinnaker/include/)
	LINK_DIRECTORIES(/opt/spinnaker/lib/)
endif()

if(DEFINED USE_UR)
	find_package(ur_rtde REQUIRED)
endif()

set(example_cpp
  photometricVisualServoing
)

if(DEFINED USE_IDS)
	if(DEFINED USE_TX)
		set(src_cpp
  		../src/Camera/CamuEye.cpp
  		../src/Camera/CamuEyeException.cpp
  		
  		../src/Robot/C_Staubli.cpp)
  else()
  		set(src_cpp
  		../src/Camera/CamuEye.cpp
  		../src/Camera/CamuEyeException.cpp)
  endif()
endif()
if(DEFINED USE_FLIR)
	if(DEFINED USE_UR)
		set(src_cpp
		    ../src/Camera/CamFlirSpinnaker.hpp
		    
  		../src/Robot/C_UR.cpp)
  else()
  		set(src_cpp
		    ../src/Camera/CamFlirSpinnaker.hpp)
  endif()
endif()

#MESSAGE(${UEYE_LIBRARY})


foreach(cpp ${example_cpp})
	# target 
	add_executable(${cpp} ${cpp}.cpp ${src_cpp})	
	
if(DEFINED USE_IDS)
	target_link_libraries(${cpp} ${UEYE_LIBRARY})
#	MESSAGE(${UEYE_LIBRARY})
endif()

if(DEFINED USE_FLIR)
	target_link_libraries(${cpp} -lSpinnaker)
endif()

if(DEFINED USE_UR)
	target_link_libraries(${cpp} ur_rtde::rtde)
endif()
	
endforeach()

#visp_set_source_file_compile_flag(photometricVisualServoing.cpp -Wno-strict-overflow)
#visp_set_source_file_compile_flag(photometricVisualServoingWithoutVpServo.cpp -Wno-strict-overflow)

#add_test(photometricVisualServoing photometricVisualServoing -c -n 20 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(photometricVisualServoingWithoutVpServo photometricVisualServoingWithoutVpServo -c -n 20 ${OPTION_TO_DESACTIVE_DISPLAY})

