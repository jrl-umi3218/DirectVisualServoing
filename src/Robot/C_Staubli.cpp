#include "C_Staubli.h"

#include <iostream>
#include <sstream>

#include <visp/vpDisplayOpenCV.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpFeatureBuilder.h>

C_Staubli::C_Staubli(std::string hostName, unsigned int portNum, double _L) : v(6)
{
	std::cout << clientStaubli.connectToHostname(hostName, portNum) << std::endl;
}

C_Staubli::~C_Staubli()
{
	vpColVector v(6);
	v = 0;
  setCameraVelocity(v);
}

int C_Staubli::setCameraVelocity(vpColVector &v)
{
	if(v.getRows() != 6)
		return -1;
	else
	{
		std::ostringstream osChaine;
		char cChaine[1024];
		
		osChaine << "v " << v[0]*1000.0 << " " << v[1]*1000.0 << " " << v[2]*1000.0 << " " << v[3]*180.0/M_PI << " " << v[4]*180.0/M_PI << " " << v[5]*180.0/M_PI << "\r";
		
		std::string sChaine = osChaine.str();
		
		std::cout << sChaine << std::endl;
		
		const char *chaine = sChaine.c_str();
		int nbSent = clientStaubli.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;
	}

	return 0;
}

int C_Staubli::setCameraArticularPose(vpColVector &j)
{
  if(j.getRows() != 6)
		return -1;
	else
	{	
		std::ostringstream osChaine;
		char cChaine[1024];
		
		osChaine << "q " << j[0]*180.0/M_PI << " " << j[1]*180.0/M_PI << " " << j[2]*180.0/M_PI << " " << j[3]*180.0/M_PI << " " << j[4]*180.0/M_PI << " " << j[5]*180.0/M_PI << "\r";
		
		std::string sChaine = osChaine.str();
		
		std::cout << sChaine << std::endl;
		
		const char *chaine = sChaine.c_str();
		int nbSent = clientStaubli.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;	
	}
	
	return 0;
}

int C_Staubli::setCameraRelativePose(vpColVector &p)
{
	if(p.getRows() != 6)
		return -1;
	else
	{
	/*
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
			
			*/
			
		std::ostringstream osChaine;
		char cChaine[1024];
		osChaine << "l " << p[0]*1000.0 << " " << p[1]*1000.0 << " " << p[2]*1000.0 << " " << p[3]*180.0/M_PI << " " << p[4]*180.0/M_PI << " " << p[5]*180.0/M_PI << "\r";
		std::string sChaine = osChaine.str();
		std::cout << sChaine << std::endl;
		const char *chaine = sChaine.c_str();
		int nbSent = clientStaubli.send(chaine, sChaine.size());

		std::cout << "nbSent : " << nbSent << std::endl;
		if(nbSent == -1)
			return -2;	
			
	}

	return 0;
}

int C_Staubli::getCameraPoseRaw(vpColVector &p)
{
  //p = rtde_control.getActualTCPPose();
     
  std::ostringstream osChaine;
	char cChaine[1024];
	memset(cChaine, 0, 1024);
	
	//while(clientStaubli.receive(cChaine, 1024*sizeof(char)) > 0);
	
	osChaine << "r" << "\r";
	std::string sChaine = osChaine.str();
	std::cout << sChaine << std::endl;
	const char *chaine = sChaine.c_str();
	int nbSent = clientStaubli.send(chaine, sChaine.size());

	std::cout << "nbSent : " << nbSent << std::endl;
	if(nbSent == -1)
		return -2;	

	memset(cChaine, 0, 1024);

	vpTime::wait(5);
	int nbReceived;
	nbReceived = clientStaubli.receive(cChaine, 1024*sizeof(char));

	std::cout << "nbReceived : " << nbReceived << std::endl;

	std::cout << "debut message" << std::endl;
	std::cout << cChaine << std::endl;
	std::cout << "fin message" << std::endl;


	if(nbReceived == 0)
		return -1;

	if(nbReceived == -1)
		return -2;

	//while(clientStaubli.receive(cChaine, 1024*sizeof(char)) > 0);

	std::istringstream isChaine(cChaine);
	isChaine >> p[0] >> p[1]>> p[2] >> p[3]>> p[4] >> p[5];
	
	//convertir p ?

	return 0;
}


