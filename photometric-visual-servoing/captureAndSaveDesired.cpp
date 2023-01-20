/*!
 \file captureAndSaveDesired.cpp
 \brief Goes to the desired robot articular pose and capture and save the desired image for photometric VS
 
 * example command line :
./captureAndSaveDesired /home/guillaume/Data/tsukubacenter.jpg 1000 500

 \param file image file to texture a simulated environment
 \param metFac the factor to transform sceneDepth to meters
 \param sceneDepth the positive depth of the scene at desired pose (in coherent units regarding metFac)
 
 *
 \author Guillaume CARON
 \version 0.1
 \date January 2023  
 */

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

#define VERBOSE

#define INDICATORS
#define FILE_EXT "jpg"

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpSimulatorCamera.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/visual_features/vpFeatureLuminance.h>
#include <visp3/vs/vpServo.h>

#include <stdlib.h>
#include <visp3/robot/vpImageSimulator.h>
#define cZ 1

int main(int argc, const char **argv)
{
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  try {

    std::ostringstream s;
    std::string filenameOut;

    std::string filename;
    bool opt_click_allowed = true;
    bool opt_display = true;
    
    float sceneDepth = cZ;
    
    //1. Loading a perpsective camera from an XML file got from the MV calibration software
    if(argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no camera ini file given: default parameters in the camera's EEPROM will be used" << std::endl;
#endif
			filename.assign("EEPROM");
    }
    else
    	filename.assign(argv[1]);
    	
//#ifdef WITHROBOT
//Get the metric factor
  float metFac = 1.f;
  if(argc < 3)
  {
#ifdef VERBOSE
      std::cout << "no metric factor given" << std::endl;
#endif //VERBOSE
  }
  else
    metFac = atof(argv[2]);

  //Get the desired scene depth
  if(argc < 4)
  {
#ifdef VERBOSE
      std::cout << "no scene depth provided" << std::endl;
#endif //VERBOSE
  }
  else
    sceneDepth = atof(argv[3])/metFac;
  
    
  vpImage<unsigned char> Itexture;

#ifdef WITHCAMERA
	int larg = 640, haut = 512;

#ifdef WITH_IDS_CAMERA
	  cv::Mat iGrab(haut,larg,CV_8UC1);
    CamuEye grabber = CamuEye(larg,haut,8,0,&filename);
#elif defined(WITH_FLIR_CAMERA)
		vpImage<unsigned char> iGrab;
		CamFlirSpinnaker<unsigned char> grabber(larg,haut,8,0);
#endif

    //Acquisition
    grabber.getFrame(iGrab);
    vpImageConvert::convert(iGrab, Itexture);
#else
    
    vpImageIo::read(Itexture, filename);
#endif

		vpCameraParameters cam(1603, 1603, 320, 256); //px = py = 750 for simulations

#ifdef WITHROBOT
#ifdef WITH_TX_ROBOT
		C_Staubli robot("10.1.22.254", 1001); // TX2-60: 172.16.0.52?
#elif defined(WITH_UR_ROBOT)
		C_UR robot("192.168.1.3", 30002, 2.0); // UR10
#endif
#else //without robot, we keep the simulation of a free flying camera
    vpColVector X[4];
    for (int i = 0; i < 4; i++)
      X[i].resize(3);
    // Top left corner
    X[0][0] = -0.3;
    X[0][1] = -0.215;
    X[0][2] = 0;

    // Top right corner
    X[1][0] = 0.3;
    X[1][1] = -0.215;
    X[1][2] = 0;

    // Bottom right corner
    X[2][0] = 0.3;
    X[2][1] = 0.215;
    X[2][2] = 0;

    // Bottom left corner
    X[3][0] = -0.3;
    X[3][1] = 0.215;
    X[3][2] = 0;

    vpImageSimulator sim;

    sim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    sim.init(Itexture, X);
#endif

    // ----------------------------------------------------------
    // Create the framegraber (here a simulated image)
    vpImage<unsigned char> I;
    vpImage<unsigned char> Id;

#ifdef WITHROBOT
		vpColVector j_init(6);

		//0.5 m depth 2
		j_init[0] = vpMath::rad(-98.72);
		j_init[1] = vpMath::rad(-158.20);
		j_init[2] = vpMath::rad(-99.87);
		j_init[3] = vpMath::rad(-11.00);
		j_init[4] = vpMath::rad(87.87);
		j_init[5] = vpMath::rad(97.40);

		robot.setCameraArticularPose(j_init);

		vpTime::wait(5000);

#ifdef WITHCAMERA
		//Acquisition desired image
    grabber.getFrame(iGrab);
    vpImageConvert::convert(iGrab, Id);

		I = Id; //for display purpose only
#endif //WITHCAMERA

#else
		I.resize(240, 320, 0);
		//I.resize(480, 640, 0);
    // camera desired position
    vpHomogeneousMatrix cdMo;
    cdMo[2][3] = sceneDepth;

    // set the robot at the desired position
    sim.setCameraPosition(cdMo);
    sim.getImage(I, cam); // and aquire the image Id
    Id = I;
#endif //WITHROBOT


  s.str("");
  s.setf(std::ios::right, std::ios::adjustfield);
  s << "resultat/Id." << FILE_EXT;
  filenameOut = s.str();
  vpImageIo::write(Id, filenameOut);

#ifdef INDICATORS
  //save the desired pose to file
  s.str("");
  s.setf(std::ios::right, std::ios::adjustfield);
  s << "resultat/desiredPose.txt";
  filenameOut = s.str();
  std::ofstream ficDesiredPose(filenameOut.c_str());

  vpColVector p;
#ifdef WITHROBOT
  robot.getCameraPoseRaw(p);
#else
	vpPoseVector pv;
	pv.buildFrom(cdMo);
	p = pv;  
#endif //WITHROBOT
  ficDesiredPose << p.t() << std::endl;

  ficDesiredPose.close();
#endif //INDICATORS



// display the image
#if defined VISP_HAVE_X11
    vpDisplayX d;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK d;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d;
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV)
    if (opt_display) {
      d.init(I, 20, 10, "Photometric visual servoing : s");
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    if (opt_display && opt_click_allowed) {
      std::cout << "Click in the image to continue..." << std::endl;
      vpDisplay::getClick(I);
    }
#endif


    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
#endif
}
