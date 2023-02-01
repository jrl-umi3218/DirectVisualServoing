#ifndef __cameraAndRobot_H__
#define __cameraAndRobot_H__

#define WITH_SIMU //WITH_SIMU will be undefined if a WITH_*_ROBOT is defined

//#define WITH_TX_ROBOT
//#define WITH_IDS_CAMERA
#define WITH_UR_ROBOT
#define WITH_FLIR_CAMERA

#ifdef WITH_TX_ROBOT
  #include "../src/Robot/C_Staubli.h"
#endif

#ifdef WITH_UR_ROBOT
  #include "../src/Robot/C_UR.h"
#endif

#if defined(WITH_TX_ROBOT) || defined(WITH_UR_ROBOT)
		#define WITHROBOT
		#undef WITH_SIMU
#endif

#ifdef WITH_IDS_CAMERA
	#include "../src/Camera/CamuEye.h"
	#include "../src/Camera/CamuEyeException.h"
#endif

#ifdef WITH_FLIR_CAMERA
//  #include "../src/Camera/CamFlirFlyCapture.hpp"
  #include "../src/Camera/CamFlirSpinnaker.hpp"
#endif

#if defined(WITH_IDS_CAMERA) || defined(WITH_FLIR_CAMERA)
		#define WITHCAMERA
#endif

#endif //__cameraAndRobot_H__

