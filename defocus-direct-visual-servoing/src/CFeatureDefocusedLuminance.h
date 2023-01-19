/****************************************************************************
 *
 * October 2020, updated in January 2023 for integration in the DirectVisualServoing repo
 *
 * Author:
 * Guillaume Caron
 * inspired from CFeatureLuminanceOmni
 *
 *****************************************************************************/

#ifndef CFeatureDefocusedLuminance_h
#define CFeatureDefocusedLuminance_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp3/visual_features/vpFeatureLuminance.h>
#include <visp/vpImage.h>

#include "CCameraThinLensParameters.h"

/*!
  \class CDefocusedLuminance
  \brief Class that defines the defocused luminance, gradient and Laplacian of a point

  \sa CFeatureDefocusedLuminance
*/
class CDefocusedLuminance : public vpLuminance
{
public:
  double x, y;   // point coordinates (in meter)
  double I;      // pixel intensity
  double Ix, Iy; // pixel gradient
  double Z;      // pixel depth
  double Ixx ;   // pixel laplacian
};


/*!
  \class CFeatureDefocusedLuminance
  \brief Class that defines the defocused image brightness visual feature

  for more details see
  G. Caron, RAL/ICRA 2021
*/

class CFeatureDefocusedLuminance : public vpFeatureLuminance
{
protected:
/*  bool recompute_xy;

  int imWidth, imHeight, di, nbri, dj, nbrj, pas;
  int derivativeMaskHalfSize, nbNeigh;
  int nbDim;

  int nbDOF;
  bool DOF[6];
  */
  //! Store the image (as a vector with intensity and gradient I, Ix, Iy) 
  CDefocusedLuminance *pixInfo ;

public:
	CFeatureDefocusedLuminance() ;
	CFeatureDefocusedLuminance(const CFeatureDefocusedLuminance &f) ;
 
  //! Destructor.
  virtual ~CFeatureDefocusedLuminance() ;

  void buildFrom(vpImage<unsigned char> &I) ;
  //void buildFrom(vpImage<unsigned char> &I, CFeatureDefocusedLuminance *sd) ;

  CFeatureDefocusedLuminance *duplicate() const ;

	
  //void init(int _imHeight, int _imWidth, int _di = 10, int _dj = 10, int _nbri = 10, int _nbrj = 10, int _pas = 1, vpImage<unsigned char> *Imask = NULL, double _rho = 1., int _derivativeMaskSize = 7) ;

  void init(unsigned int _nbr, unsigned int _nbc, double _Z);
	vpMatrix interaction(unsigned int select = FEATURE_ALL);
  void interaction(vpMatrix &L);

	CFeatureDefocusedLuminance &operator=(const CFeatureDefocusedLuminance &f);



  void setCameraParameters(CCameraThinLensParameters &_cam) ;
  //void resetCameraParameters(CCameraThinLensParameters &_cam) ;

  //void set_DOF(bool un = true, bool deux = true, bool trois = true, bool quatre = true, bool cinq = true, bool six = true);

//private:
  //void cartImagePlaneInteraction(vpMatrix &L);

 public:
  CCameraThinLensParameters cam ;

} ;

#endif //CFeatureDefocusedLuminance_h
