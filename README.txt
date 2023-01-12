##########################################################################################
#
# DirectVisualServoing: joint work between CNRS-AIST JRL and UPJV MIS lab based on Inria/IRISA's ViSP
#
# Authors: Guillaume Caron, Belinda Naamani, ...
#
# Dates: from December 2022 to ...
##########################################################################################

0. Prerequisites to run these codes: 
     install cmake (version 3.16.3 tested)
     install opencv (version 4.2.0 tested)
     install ViSP (version 3.4.1 tested)
     download test data from: http://mis.u-picardie.fr/~g-caron/data/2023_direct-visual-servoing-data.zip and set its content in the 2023_direct-visual-servoing-data directory, itself in the DirectVisualServoing directory
1. create a new directory named photometric-visual-servoing-build in the DirectVisualServoing directory
2. use cmake to fill the build directory in
3. open the project in build or use the make command in the latter directory to build the exe file
5. run the programs from the command line from the DirectVisualServoing directory, considering it includes the 2023_direct-visual-servoing-data directory, with arguments as:
     
./photometric-visual-servoing-build/photometricVisualServoing \
		./2023_direct-visual-servoing-data/texture4simu/tsukubacenter.jpg \
		1000 \
		500 \
		5 \
		-5 \
		1

for a purely simulated photometric visual servoing. The directory resultat contains the outputs (desired, current and error images + desired and current poses along iterations + residuals + processing times)

In the photometricVisualServoing.cpp, uncomment #define WITHROBOT and #define WITHCAMERA to use a real robot, for the former, and a real camera, for the latter. 
