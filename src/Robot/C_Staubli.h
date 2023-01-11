#ifndef __C_Staubli_H__
#define  __C_Staubli_H__

#include <visp/vpClient.h>
#include <visp/vpColVector.h>
#include <visp/vpPoseVector.h>
#include <visp/vpImage.h>
#include <visp/vpDot.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>

#include <string>

class C_Staubli
{
public:
	C_Staubli(std::string hostName, unsigned int portNum, double _L = 0.087);
    ~C_Staubli();

	int setCameraVelocity(vpColVector &v);
	int setCameraArticularPose(vpColVector &j);
	int setCameraRelativePose(vpColVector &p);
	int getCameraPoseRaw(vpColVector &p);
    
private:
	vpClient clientStaubli;
	vpColVector v;
};

#endif // __C_Staubli_H__
