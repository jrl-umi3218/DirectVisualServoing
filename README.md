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
- if using an UR robot, install the `ur_rtde` library (`https://gitlab.com/sdurobotics/ur_rtde`, version 1.5.0 tested)
- download test data from: `http://mis.u-picardie.fr/~g-caron/data/2023_direct-visual-servoing-data.zip` and set its content in the `2023_direct-visual-servoing-data` directory, itself in the `DirectVisualServoing` directory

## Installation

- create in the `DirectVisualServoing` directory a new directory named:
	- `photometric-visual-servoing-build` for photometric visual servoing
	- `defocus-direct-visual-servoing-build` for defocus-based direct visual servoing

- use cmake (twice) to fill the build directory in, from command line (add `-D USE_TX=True` to use Staubli robot classes, `-D USE_UR=True` to use UR robot classes, `-D USE_IDS=True` to use IDS camera classes, `-D USE_FLIR=True` to use Flir camera classes. Note: for the moment only combinations of IDS+TX or FLIR+UR are possible.): 
	- in `photometric-visual-servoing-build` directory: `cmake ../photometric-visual-servoing -DCMAKE_BUILD_TYPE=Release`
	- in `defocus-direct-visual-servoing-build` directory: `cmake ../defocus-direct-visual-servoing -DCMAKE_BUILD_TYPE=Release -D USE_UR=True -D USE_FLIR=True`

- set the desired robot joint angles within the source code (temporary: check the angles on your robot):
```
		j_init[0] = vpMath::rad(-98.72);
		j_init[1] = vpMath::rad(-158.20);
		j_init[2] = vpMath::rad(-99.87);
		j_init[3] = vpMath::rad(-11.00);
		j_init[4] = vpMath::rad(87.87);
		j_init[5] = vpMath::rad(97.40);
```

- open the project in `*-build` or use the `make` command in the latter directory to build the `exe` file

## Execution

Run the programs from the command line from the `DirectVisualServoing` directory, considering it includes the `2023_direct-visual-servoing-data` directory, with arguments as:
- for photometric visual servoing (purely simulated)
```
./photometric-visual-servoing-build/photometricVisualServoing \
		./2023_direct-visual-servoing-data/texture4simu/tsukubacenter.jpg \
		1000 \
		500 \
		5 \
		-5 \
		1
```
- for defocus-based direct visual servoing
```
./defocus-direct-visual-servoing-build/defocusDirectVisualServoing \
		EEPROM \
		1000 \
		500 \
		5 \
		-5 \
		1
```

Command line arguments are in this order:
- \param FileOrEEPROMkeyword if a filename: camera ini file or image file to texture a simulated environment / if EEPROM, loads the camera acquisition parameters from the camera EEPROM (put there from, e.g., ueyedemo or uEyeCockpit software)
- \param metFac the factor to transform shiftX to meters
- \param sceneDepth the positive depth of the scene at desired pose (in coherent units regarding metFac)
- \param shiftX the signed lateral shift (in coherent units regarding metFac)
- \param shiftZ the signed depth shift (in coherent units regarding metFac)
- \param rotY the signed vertical rotation (in degrees)

The directory `resultat` contains the outputs (desired, current and error images + desired and current poses along iterations + residuals + processing times)

In `photometricVisualServoing.cpp`, `defocusDirectVisualServoing.cpp` source files, uncomment `#define WITH_*_ROBOT`, and `#define WITH_*_CAMERA` to use a real robot, for the former, and a real camera, for the latter, and comment them to use a fully simulated run. 
