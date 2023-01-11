
#include "CamuEye.h"
#include <iostream>
#include <ueye.h>

// if filename contains EEPROM, it loads the camera parameters saved in its own memory

CamuEye::CamuEye(int wdth, int heigh, int dpth, int camID, std::string *filename) {
        width = wdth;
        height = heigh;
        depth= dpth;
        using std::cout;
        using std::endl;
        mattie = cv::Mat(height, width, cv::traits::Type< cv::Vec<uchar, 3> >::value);//cv::DataType<cv::Vec3b>::type);
        hCam = camID;
        char* ppcImgMem;
        int pid;
        INT nAOISupported = 0;
        double on = 1;
        int retInt = IS_SUCCESS;
        retInt = is_InitCamera(&hCam, 0);

        if (retInt != IS_SUCCESS) {
        		std::cout << "init camera failed" << std::endl;
                throw CamuEyeException(hCam, retInt);
        }
        

        if(filename != NULL)
        {
        	std::cout << (*filename) << std::endl;
        
          widthMax=width;
          heightMax=height;
          
          if(filename->compare("EEPROM") == 0)
						retInt = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_LOAD_EEPROM, NULL, NULL);
					else
		  			retInt = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_LOAD_FILE, (void*)filename->c_str(), NULL);
		  	
          if(retInt  != IS_SUCCESS)
          {
          		std::cout << "load parameter file failed " << std::endl;
                throw CamuEyeException(hCam, retInt);
          }
          retInt = is_AllocImageMem(hCam, width, height, dpth, &ppcImgMem, &pid);
          if (retInt != IS_SUCCESS) {
          		std::cout << "alloc image memory failed" << std::endl;
                  throw CamuEyeException(hCam, retInt);
          }

          retInt = is_SetImageMem(hCam, ppcImgMem, pid);
          if (retInt != IS_SUCCESS) {
          		std::cout << "set image memory failed" << std::endl;
                  throw CamuEyeException(hCam, retInt);
          };

        }
        else
        {
          retInt = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          }       
      		SENSORINFO sInfo;
          is_GetSensorInfo(hCam,&sInfo);
          widthMax=sInfo.nMaxWidth;
          heightMax=sInfo.nMaxHeight;

          retInt = is_AllocImageMem(hCam, widthMax, heightMax, dpth, &ppcImgMem, &pid);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          }

          retInt = is_SetImageMem(hCam, ppcImgMem, pid);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          };
        }

        retInt = is_CaptureVideo(hCam, IS_WAIT);
		if (retInt != IS_SUCCESS) {
				std::cout << "start video capture failed" << std::endl;
                throw CamuEyeException(hCam, retInt);
        }
}

CamuEye::~CamuEye() {
        int retInt = is_ExitCamera(hCam);
        if (retInt != IS_SUCCESS) {
                throw CamuEyeException(hCam, retInt);
        }
}

cv::Mat CamuEye::getFrame() {
        getFrame(mattie);
        return mattie;
}

void CamuEye::getFrame(cv::Mat& mat) {
        VOID* pMem;
        int retInt = is_GetImageMem(hCam, &pMem);

        if (retInt != IS_SUCCESS) {
                throw CamuEyeException(hCam, retInt);
        }			

    if((heightMax != height) || (widthMax != width))
    {

		cv::Mat mat2(heightMax,widthMax,cv::traits::Type< cv::Vec<uchar, 3> >::value);//cv::DataType<cv::Vec3b>::type);
                memcpy(mat2.ptr(), pMem, widthMax * heightMax * depth *0.125);

		cv::resize(mat2,mat,cv::Size(width,height));
    }
    else
      memcpy(mat.ptr(), pMem, widthMax * heightMax * depth *0.125);
}

HIDS CamuEye::getHIDS() {
        return hCam;
}

void CamuEye::setAutoWhiteBalance(bool set) {
        double empty;
        double on = set ? 1 : 0;
        int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &on, &empty);
        if (retInt != IS_SUCCESS) {
                throw CamuEyeException(hCam, retInt);
        }
}

void CamuEye::setAutoGain(bool set) {
        double empty;
        double on = set ? 1 : 0;
        int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &on, &empty);
        if (retInt != IS_SUCCESS) {
                throw CamuEyeException(hCam, retInt);
        }
}
