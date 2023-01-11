// Romain Marie
// MIS Laboratory, Amiens, FRANCE
// Mai 2012
#pragma once

#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CamuEyeException.h"

class CamuEye {
public:

        CamuEye(int wdth, int heigh, int dpth, int camID = 0, std::string *filename=NULL);
        HIDS getHIDS();
        ~CamuEye();
        cv::Mat getFrame();
        void getFrame(cv::Mat& mat);
        void setAutoWhiteBalance(bool set=true);
        void setAutoGain(bool set=true);
private:
        HIDS hCam;
        cv::Mat mattie;
        int width;
        int widthMax;
        int height;
        int heightMax;
        int depth;
};



