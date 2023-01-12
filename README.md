# DirectVisualServoing

Joint work between CNRS-AIST JRL and UPJV MIS lab based on Inria/IRISA's ViSP

Authors: Guillaume Caron, Belinda Naamani, ...

Dates: from December 2022 to ...

## Prerequisites to run these codes: 
- install cmake (version 3.16.3 tested)
- install opencv (version 4.2.0 tested)
- install ViSP (version 3.4.1 tested)
- if using an IDS camera, install the IDS software suite (version 4.96.1 tested)
- if using a Flir camera, install the Flir Spinnaker SDK (version 2.6.0.157 tested, FlyCapture is also possible for older operating systems)
- if using an UR robot, install the ur_rtde library (https://gitlab.com/sdurobotics/ur_rtde, version 1.5.0 tested)
- download test data from: http://mis.u-picardie.fr/~g-caron/data/2023_direct-visual-servoing-data.zip and set its content in the 2023_direct-visual-servoing-data directory, itself in the DirectVisualServoing directory

## Installation

- create a new directory named photometric-visual-servoing-build in the DirectVisualServoing directory

- use cmake to fill the build directory in, from command line in photometric-visual-servoing-build directory: cmake ../photometric-visual-servoing -DCMAKE_BUILD_TYPE=Release (add -D USE_TX=True to use Staubli robot classes, -D USE_UR=True to use UR robot classes, -D USE_IDS=True to use IDS camera classes, -D USE_FLIR=True to use Flir camera classes. Note: for the moment only combinations of IDS+TX or FLIR+UR are possible.)

- open the project in build or use the make command in the latter directory to build the exe file

## Execution

Run the programs from the command line from the DirectVisualServoing directory, considering it includes the 2023_direct-visual-servoing-data directory, with arguments as:
     
./photometric-visual-servoing-build/photometricVisualServoing \
		./2023_direct-visual-servoing-data/texture4simu/tsukubacenter.jpg \
		1000 \
		500 \
		5 \
		-5 \
		1

for a purely simulated photometric visual servoing. Commande line arguments are in this order:
 \param FileOrEEPROMkeyword if a filename: camera ini file or image file to texture a simulated environment / if EEPROM, loads the camera acquisition parameters from the camera EEPROM (put there from, e.g., ueyedemo or uEyeCockpit software)
 \param metFac the factor to transform shiftX to meters
 \param sceneDepth the positive depth of the scene at desired pose (in coherent units regarding metFac)
 \param shiftX the signed lateral shift (in coherent units regarding metFac)
 \param shiftZ the signed depth shift (in coherent units regarding metFac)
 \param rotY the signed vertical rotation (in degrees)

The directory resultat contains the outputs (desired, current and error images + desired and current poses along iterations + residuals + processing times)

In the photometricVisualServoing.cpp, uncomment #define WITHROBOT and #define WITHCAMERA to use a real robot, for the former, and a real camera, for the latter. 
