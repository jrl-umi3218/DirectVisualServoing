/*!
 \file defocusDirectVisualServoing.cpp
 \brief Extension of photometric visual servoing \cite Collewet08c to defocus-based direct visual servoing \cite Caron21
 
 * example command line :
./defocusDirectVisualServoing /home/guillaume/Data/tsukubacenter.jpg 1000 500 5 -5 1

./defocusDirectVisualServoing EEPROM 1000 500 10 -200 5

 \param FileOrEEPROMkeyword if a filename: camera ini file or image file to texture a simulated environment / if EEPROM, loads the camera acquisition parameters from the camera EEPROM (put there from, e.g., ueyedemo or uEyeCockpit software)
 \param metFac the factor to transform shiftX to meters
 \param sceneDepth the positive depth of the scene at desired pose (in coherent units regarding metFac)
 \param shiftX the signed lateral shift (in coherent units regarding metFac)
 \param shiftZ the signed depth shift (in coherent units regarding metFac)
 \param rotY the signed vertical rotation (in degrees)
 *
 \author Guillaume CARON
 \version 0.1
 \date 2020-2021 (initial code, https://gite.lirmm.fr/jrp-2019/phovs.git), then January 2023 - (this code) 
 */

//#define WITH_TX_ROBOT
//#define WITH_IDS_CAMERA
//#define WITH_UR_ROBOT
//#define WITH_FLIR_CAMERA

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

#include "src/CCameraThinLensParameters.h"
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpSimulatorCamera.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

//#include <visp3/visual_features/vpFeatureLuminance.h>
#include "src/CFeatureDefocusedLuminance.h"
#include <visp3/vs/vpServo.h>

#include <stdlib.h>
#include <visp3/robot/vpImageSimulator.h>
#define cZ 1

//Pour images acquises de 640x512
#define ACQWIDTH 640
#define ACQHEIGHT 512
#define FACT 1.0

int main(int argc, const char **argv)
{
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  try {
#ifdef INDICATORS
    std::ostringstream s;
    std::string filenameOut;
#endif // INDICATORS

    std::string filename;
    bool opt_click_allowed = true;
    bool opt_display = true;
    int opt_niter = 1000;
    
    float sceneDepth = cZ;//0.5f;

    
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

  float shiftX = 0.f;
  if(argc < 5)
  {
#ifdef VERBOSE
      std::cout << "no shift X given" << std::endl;
#endif //VERBOSE
  }
  else
    shiftX = atof(argv[4]);

  float shiftZ = 0.f;
  if(argc < 6)
  {
#ifdef VERBOSE
      std::cout << "no shift Z given" << std::endl;
#endif //VERBOSE
  }
  else
    shiftZ = atof(argv[5]);

  float rotY = 0.f;
  if(argc < 7)
  {
#ifdef VERBOSE
      std::cout << "no rotate Y given" << std::endl;
#endif //VERBOSE
  }
  else
    rotY = atof(argv[6]);

	vpColVector p_init;
  p_init.resize(6);

  p_init[0] = shiftX/metFac;
  p_init[2] = shiftZ/metFac;
  p_init[4] = rotY/180.0; //*M_PI
  
    
  vpImage<unsigned char> Itexture;

	int larg = ACQWIDTH*FACT, haut = ACQHEIGHT*FACT;
	
	// very theorical
	double u0 = (ACQWIDTH*0.5)*FACT;
  double v0 = (ACQHEIGHT*0.5)*FACT;

#ifdef WITHCAMERA

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

		//vpCameraParameters cam(750, 750, 320, 256);
		
	  //Parametres intrinseques pour FlirCam FL3 1/1.8" 1280x1024 5.3 um (matrice: 6.784 mm x 5.4272 mm)

  	double ku = (2*5.3e-6)/FACT; // m // 2*5.3e-6 because camera configured with binning x2

  	//Parametres intrinseques pour FlirCam GS3 1" 2048x2048 5.5 um (matrice: 11.264 mm x 11.264 mm)
  
		//Parametres objectif Yakumo
		double precond = 1.0;// 0.1;//
		//ku /= precond;
		double f = precond*8.5e-3;//17e-3; // m
		double FNumber = 0.95; //no unit
		double Zf = 0.5;//0.25; // m

		CCameraThinLensParameters cam(f, ku, FNumber, Zf, u0, v0);
		

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

		j_init[0] = vpMath::rad(-51.13);
		j_init[1] = vpMath::rad(-129.05);
		j_init[2] = vpMath::rad(-96.63);
		j_init[3] = vpMath::rad(-41.79);
		j_init[4] = vpMath::rad(88.69);
		j_init[5] = vpMath::rad(84.95);

		robot.setCameraArticularPose(j_init);
		
		vpTime::wait(5000);

#ifdef WITHCAMERA
		//Acquisition desired image
    grabber.getFrame(iGrab);
    vpImageConvert::convert(iGrab, Id);
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

#ifdef INDICATORS
  s.str("");
  s.setf(std::ios::right, std::ios::adjustfield);
  s << "resultat/Id." << FILE_EXT;
  filenameOut = s.str();
  vpImageIo::write(Id, filenameOut);

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

    // ----------------------------------------------------------
    // position the robot at the initial position
    // ----------------------------------------------------------

  std::cout << "Deplacement vers pose initiale " << p_init.t() << std::endl;

#ifdef WITHROBOT
	robot.setCameraRelativePose(p_init);

  vpTime::wait(5000);

#ifdef WITHCAMERA
		//Acquisition initial image
    grabber.getFrame(iGrab);
    vpImageConvert::convert(iGrab, I);
#endif //WITHCAMERA

#else
    // camera initial position
    vpHomogeneousMatrix cMcd, cMo;
    //cMo.buildFrom(0, 0, 1.02, vpMath::rad(0.0001), vpMath::rad(0), vpMath::rad(0));
    cMcd.buildFrom(p_init[0], p_init[1], p_init[2], p_init[3], p_init[4], p_init[5]);
    cMo = cMcd * cdMo;
    vpHomogeneousMatrix wMo; // Set to identity
    vpHomogeneousMatrix wMc; // Camera position in the world frame

    // set the robot at the desired position
    sim.setCameraPosition(cMo);
    I = 0;
    sim.getImage(I, cam); // and aquire the image Id
#endif //WITHROBOT

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
    if (opt_display) {
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    if (opt_display && opt_click_allowed) {
      std::cout << "Click in the image to continue..." << std::endl;
      vpDisplay::getClick(I);
    }
#endif

    vpImage<unsigned char> Idiff;
    Idiff = I;

    vpImageTools::imageDifference(I, Id, Idiff);

// Affiche de l'image de difference
#if defined VISP_HAVE_X11
    vpDisplayX d1;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d1;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK d1;
#endif
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
    if (opt_display) {
      d1.init(Idiff, 40 + static_cast<int>(I.getWidth()), 10, "photometric visual servoing : s-s* ");
      vpDisplay::display(Idiff);
      vpDisplay::flush(Idiff);
    }
#endif

#ifndef WITHROBOT
    // create the robot (here a simulated free flying camera)
    vpSimulatorCamera freeFlyingCamera;
    freeFlyingCamera.setSamplingTime(0.04);
    wMc = wMo * cMo.inverse();
    freeFlyingCamera.setPosition(wMc);
#endif

    // ------------------------------------------------------
    // Visual feature, interaction matrix, error
    // s, Ls, Lsd, Lt, Lp, etc
    // ------------------------------------------------------

    // current visual feature built from the image
    // (actually, this is the image...)
    //vpFeatureLuminance sI;
		CFeatureDefocusedLuminance sI;
    sI.init(I.getHeight(), I.getWidth(), sceneDepth);
    sI.setCameraParameters(cam);
    sI.buildFrom(I);

    // desired visual feature built from the image
    //vpFeatureLuminance sId;
		CFeatureDefocusedLuminance sId;
    sId.init(I.getHeight(), I.getWidth(), sceneDepth);
    sId.setCameraParameters(cam);
    sId.buildFrom(Id);

    // Create visual-servoing task
    vpServo servo;
    // define the task
    // - we want an eye-in-hand control law
    // - robot or freeFlyingCamera is controlled in the camera frame
    servo.setServo(vpServo::EYEINHAND_CAMERA);
    // add current and desired visual features
    servo.addFeature(sI, sId);
    // set the gain
    servo.setLambda(1); //30
    // compute interaction matrix at the desired position
    servo.setInteractionMatrixType(vpServo::CURRENT);

#ifndef WITHROBOT
    // set a velocity control mode
    freeFlyingCamera.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
#endif

    int iter = 1;
    double normError = 0;
    vpColVector v; // camera velocity send to the robot


#ifdef INDICATORS
	double residual;
  std::vector<vpColVector> v_p;
  std::vector<double> v_residuals;
  std::vector<vpImage<unsigned char> > v_I_cur;
  std::vector<vpImage<unsigned char> > v_Idiff;
  double duree;
  std::vector<double> v_tms;
  std::vector<bool> v_servo_actif;

  double cond;
  std::vector<double> v_svd;
#endif //INDICATORS

    vpChrono chrono;
    chrono.start();
    do {
      std::cout << "--------------------------------------------" << iter++ << std::endl;

#ifdef WITHROBOT

#ifdef INDICATORS
    robot.getCameraPoseRaw(p);
#endif //INDICATORS
			
#ifdef WITHCAMERA
		//Acquisition initial image
    grabber.getFrame(iGrab);
    vpImageConvert::convert(iGrab, I);
#endif //WITHCAMERA
			
#else
#ifdef INDICATORS
    pv.buildFrom(cMo);
		p = pv;  
#endif //INDICATORS

      //  Acquire the new image
      sim.setCameraPosition(cMo);
      sim.getImage(I, cam);
#endif //WITHROBOT

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
      if (opt_display) {
        vpDisplay::display(I);
        vpDisplay::flush(I);
      }
#endif
      vpImageTools::imageDifference(I, Id, Idiff);
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
      if (opt_display) {
        vpDisplay::display(Idiff);
        vpDisplay::flush(Idiff);
      }
#endif
      // Compute current visual feature
      sI.buildFrom(I);

      v = servo.computeControlLaw(); // camera velocity send to the robot

      normError = servo.getError().sumSquare();
      std::cout << " |e| = " << normError << std::endl;
      std::cout << " |v| = " << sqrt(v.sumSquare()) << std::endl;

      // send the robot velocity
#ifdef WITHROBOT
    //if(servo_actif)
      robot.setCameraVelocity(v);

#else
      freeFlyingCamera.setVelocity(vpRobot::CAMERA_FRAME, v);
      wMc = freeFlyingCamera.getPosition();
      cMo = wMc.inverse() * wMo;
#endif

#ifdef INDICATORS
    v_p.push_back(p);
    residual = 0.5*normError;
    v_residuals.push_back(residual);
    v_I_cur.push_back(I);
    v_Idiff.push_back(Idiff);
    duree = chrono.getDurationMs();
    v_tms.push_back(duree);

    //v_servo_actif.push_back(servo_actif);

    //cond = ;
    //v_svd.push_back(cond);
#endif //INDICATORS   

    } while (normError > 10000 && iter < opt_niter);

    chrono.stop();
    std::cout << "Time to convergence: " << chrono.getDurationMs() << " ms" << std::endl;

    v = 0;
#ifdef WITHROBOT
    //if(servo_actif)
      robot.setCameraVelocity(v);

#else
    freeFlyingCamera.setVelocity(vpRobot::CAMERA_FRAME, v);
#endif

#ifdef INDICATORS
    //save pose list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/poses.txt";
    filename = s.str();
    std::ofstream ficPoses(filename.c_str());
    //save residual list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/residuals.txt";
    filename = s.str();
    std::ofstream ficResiduals(filename.c_str());
    //save the processing times
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/times.txt";
    filename = s.str();
    std::ofstream ficTimes(filename.c_str());
		/*
    //save servo actif list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "resultat/servoActif.txt";
    filename = s.str();
    std::ofstream ficServo(filename.c_str());
		*/
    for(unsigned int i = 0 ; i < v_p.size() ; i++)
    {
      ficPoses << v_p[i].t() << std::endl;
      ficResiduals << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << v_residuals[i] << std::endl;
      
      ficTimes << v_tms[i] << std::endl;
      //ficSVD << v_svd[i] << std::endl;
      //ficServo << v_servo_actif[i] << std::endl;

      s.str("");
      s.setf(std::ios::right, std::ios::adjustfield);
      s << "resultat/I/I." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
      filename = s.str();
      vpImageIo::write(v_I_cur[i], filename);

      s.str("");
      s.setf(std::ios::right, std::ios::adjustfield);
      s << "resultat/IErr/IErr." << std::setw(4) << std::setfill('0') << i << "." << FILE_EXT;
      filename = s.str();
      vpImageIo::write(v_Idiff[i], filename);
    }

    ficPoses.close();
    ficResiduals.close();
    ficTimes.close();
    //ficSVD.close();
    //ficServo.close();
#endif //INDICATORS


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
