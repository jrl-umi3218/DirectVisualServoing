// Guillaume Caron
// JRL, AIST, Tsukuba, Japan
// Dec. 2022
#ifndef __CamFlirSpinnaker_H__
#define  __CamFlirSpinnaker_H__

#include <visp/vpImage.h>
#include <visp/vpImageTools.h>

#include "Spinnaker.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

template<typename T>
class CamFlirSpinnaker
{
public:

  CamFlirSpinnaker(int wdth, int heigh, int dpth, int camID = 0, void *filename=NULL): width(wdth), height(heigh), depth(dpth)
  {
		// Retrieve singleton reference to system object
		system = System::GetInstance();

		// Print out current library version
		const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
		cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
		<< "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
		<< endl;

		// Retrieve list of cameras from the system
		camList = system->GetCameras();

		const unsigned int numCameras = camList.GetSize();

		// Finish if there are no cameras
		if (numCameras == 0)
		{
			// Clear camera list before releasing system
			camList.Clear();

			// Release system
			system->ReleaseInstance();

			cout << "Not enough cameras!" << endl;
			cout << "Done! Press Enter to exit..." << endl;
			getchar();
		}
		else
		{

			// Select camera
			pCam = camList.GetByIndex(camID);

			try
			{
				// Retrieve TL device nodemap and print device information
				INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

				int result = PrintDeviceInfo(nodeMapTLDevice);

				// Initialize camera
				pCam->Init();

				// Retrieve GenICam nodemap
				INodeMap& nodeMap = pCam->GetNodeMap();

				//
				// Set acquisition mode to continuous
				//

				//
				// Retrieve enumeration node from nodemap
				CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
				if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
				{
					cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
				}
				else
				{

					// Retrieve entry node from enumeration node
					CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
					if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
					{
						cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
					}
					else
					{
						// Retrieve integer value from entry node
						const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

						// Set integer value from entry node as new value of enumeration node
						ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

						cout << "Acquisition mode set to continuous..." << endl;
						
						// Retrieve Stream Parameters device nodemap
        		Spinnaker::GenApi::INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();

        		// Retrieve Buffer Handling Mode Information
        		CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        		if (!IsReadable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
				    {
				        cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << endl << endl;
				    }

				    CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
				    if (!IsReadable(ptrHandlingModeEntry))
				    {
				        cout << "Unable to get Buffer Handling mode (Entry retrieval). Aborting..." << endl << endl;
				    }

				    // Display Buffer Info
				    cout << endl << "Default Buffer Handling Mode: " << ptrHandlingModeEntry->GetDisplayName() << endl;

						ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
            ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
            cout << endl
                 << endl
                 << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;

						//
						// Begin acquiring images
						//
						// *** NOTES ***
						// What happens when the camera begins acquiring images depends on the
						// acquisition mode. Single frame captures only a single image, multi
						// frame captures a set number of images, and continuous captures a
						// continuous stream of images. Because the example calls for the
						// retrieval of 10 images, continuous mode has been set.
						//
						// *** LATER ***
						// Image acquisition must be ended when no more images are needed.
						//
						pCam->BeginAcquisition();

						cout << "Acquiring images..." << endl;
						
						processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
					}
				}

			}
			catch (Spinnaker::Exception& e)
			{
				cout << "Error: " << e.what() << endl;
			}
		
		}
    
  }
  
  ~CamFlirSpinnaker()
  {
  
   // End acquisition
        pCam->EndAcquisition();
  
  // Deinitialize camera
        pCam->DeInit();
  
  
		// Release reference to the camera
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
  }

  vpImage<T> getFrame()
  {
          getFrame(Ie);
          return Ie;
  }

  void getFrame(vpImage<T>& I)
  {
		try
		{
			//
			pResultImage = pCam->GetNextImage(500);

			//
			// Ensure image completion

			if (pResultImage->IsIncomplete())
			{
				// Retrieve and print the image status description
				cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
					 << "..." << endl
					 << endl;
			}
			else
			{
				switch(depth)
				{
					case 8:
					{
						// Convert the raw image

						//
						// Convert image to mono 8
						//
						//convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
						convertedImage = processor.Convert(pResultImage, PixelFormat_Mono8);
						
						//
					// Release image

					pResultImage->Release();
						
						I.resize(convertedImage->GetHeight(), convertedImage->GetWidth(), false);
//std::cout << convertedImage->GetBufferSize() << std::endl;
						memcpy(I.bitmap, convertedImage->GetData(), convertedImage->GetBufferSize());

						if( (I.getHeight() != height) || (I.getWidth() != width) )
						{
							vpImage<T> I_resize;
							vpImageTools::resize(I, I_resize, width, height, 								vpImageTools::INTERPOLATION_LINEAR);//vpImageTools::INTERPOLATION_AREA);
							I = I_resize;
						}

						break;
					}
					default:
					{
						cout << "Image bit depth not implemented" << endl;
						return;
					}
				}
			}

			

			cout << endl;
		}
		catch (Spinnaker::Exception& e)
		{
			cout << "Error: " << e.what() << endl;
			return;
		}
  }
  
  void setAutoWhiteBalance(bool set=true)
  {
    cout << "CamFlirSpinnaker::setAutoWhiteBalance not implemented" << endl;
  /*        double empty;
          double on = set ? 1 : 0;
          int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &on, &empty);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          }
  */
  }
  
  void setAutoGain(bool set=true)
  {
    cout << "CamFlirSpinnaker::setAutoGain not implemented" << endl;
  /*
          double empty;
          double on = set ? 1 : 0;
          int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &on, &empty);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          }
  */
  }

private:
// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
    int result = 0;
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            for (auto it = features.begin(); it != features.end(); ++it)
            {
                const CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}


	SystemPtr system;
	CameraList camList;
	CameraPtr pCam;
	ImagePtr convertedImage;
	ImagePtr pResultImage;
	ImageProcessor processor;

	vpImage<T> Ie;
	int depth;
  int width;
  int height;

};

#endif // __CamFlirSpinnaker_H__
