#include "C_UR.h"

#include <iostream>

C_UR::C_UR(std::string hostName, unsigned int portNum, double _delta_t) : v(6), delta_t(_delta_t), rtde_control(hostName)
{
	std::cout << clientUR.connectToHostname(hostName, portNum) << std::endl;
 }

C_UR::~C_UR()
{
	vpColVector v(6);
	v = 0;
    setCameraVelocity(v);
}

int C_UR::setCameraVelocity(vpColVector &cv)
{
	if(cv.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];

    //v *= delta_t; // apply sampling time to UR

		//osChaine << "movej(p[" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ", " << v[4] << ", " << v[5] << "], a=1, v=0.5)\n";
    //osChaine << "movej(pose_trans(get_actual_tcp_pose(), p[" << v[0] << ", " << v[1] << ", " << v[2] << ," " << v[3] << ", " << v[4] << ", " << v[5] << "]), t=" << delta_t << ")\n";

    // OK !!!!!!
    std::vector<double> actual_tcp_pose = rtde_control.getActualTCPPose();

    //std::cout << actual_tcp_pose[0] << " " << actual_tcp_pose[1] << " " << actual_tcp_pose[2] << " " << actual_tcp_pose[3] << " " << actual_tcp_pose[4] << " " << actual_tcp_pose[5] << std::endl;

    vpRotationMatrix bRc(vpThetaUVector(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]));
    vpTranslationVector btc;
    vpHomogeneousMatrix bMc(btc, bRc);

//    std::cout << cMb << std::endl;

    vpVelocityTwistMatrix bVc(bMc);

    vpColVector bv = bVc * cv;

//    osChaine << "speedl([" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ", " << v[4] << ", " << v[5] << "], a=2.0, t=0.05)\n";//, t=" << delta_t << ")\n";
    osChaine << std::fixed << std::setprecision( 4 ) << "speedl([" << bv[0] << ", " << bv[1] << ", " << bv[2] << ", " << bv[3] << ", " << bv[4] << ", " << bv[5] << "], a=0.05, t=0.33)\n";//, t=" << delta_t << ")\n";

// OK, mais "stops"
//    osChaine << "movej(pose_trans(get_actual_tcp_pose(), p[" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ", " << v[4] << ", " << v[5] << "]), t=" << delta_t << ")\n"; //, r=0.0011

//    osChaine << "movej(p[" << p01[0] << ", " << p01[1] << ", " << p01[2] << ", " << p01[3] << ", " << p01[4] << ", " << p01[5] << "], t=" << delta_t << ")\n"; //, r=0.0011

		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientUR.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_UR::setCameraArticularPose(vpColVector &j)
{
  if(j.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];

		osChaine << "movej([" << j[0] << ", " << j[1] << ", " << j[2] << ", " << j[3] << ", " << j[4] << ", " << j[5] << "], a=1, v=0.5)\n";

		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientUR.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_UR::setCameraRelativePose(vpColVector &p)
{
	if(p.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];

    osChaine << "movel(pose_trans(get_actual_tcp_pose(), p[" << p[0] << ", " << p[1] << ", " << p[2] << ", " << p[3] << ", " << p[4] << ", " << p[5] << "]), a=0.1, v=0.5)\n";

		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientUR.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_UR::getCameraPoseRaw(vpColVector &p)
{
  p = rtde_control.getActualTCPPose();

	return 0;
}

int C_UR::getCameraPose(vpPoseVector &p)
{
  std::vector<double> actual_tcp_pose = rtde_control.getActualTCPPose();

  vpRotationMatrix bRc(vpThetaUVector(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]));   
  vpTranslationVector btc;
  vpHomogeneousMatrix bMc(btc, bRc);

  p.buildFrom(bMc);

	return 0;
}

