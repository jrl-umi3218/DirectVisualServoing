/*!
  \class CFeatureDefocusedLuminance.cpp
  \brief Class that defines the defocused image brightness visual feature

  for more details see
  G. Caron, RAL/ICRA 2021 submission
*/

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpImageConvert.h>

#include <visp3/core/vpImageFilter.h>
#include "CImageFilter.h"
#include "CFeatureDefocusedLuminance.h"

void CFeatureDefocusedLuminance::init(unsigned int _nbr, unsigned int _nbc, double _Z)
{
  vpFeatureLuminance::init();

  nbr = _nbr;
  nbc = _nbc;

  if ((nbr < 2 * bord) || (nbc < 2 * bord)) {
    throw vpException(vpException::dimensionError, "border is too important compared to number of row or column.");
  }

  // number of feature = nb column x nb lines in the images
  dim_s = (nbr - 2 * bord) * (nbc - 2 * bord);

  s.resize(dim_s);

  if (pixInfo != NULL)
    delete[] pixInfo;

  pixInfo = new CDefocusedLuminance[dim_s];

  Z = _Z;
}

/*! 
  Default constructor that build a visual feature.
*/
CFeatureDefocusedLuminance::CFeatureDefocusedLuminance() : vpFeatureLuminance(), pixInfo(NULL),  cam(8e-3,5e-6,1,0.25,320,240)
{
  nbParameters = 1;
  dim_s = 0;
  flags = NULL;

  vpFeatureLuminance::init() ;
}

/*!
 Copy constructor.
 */
CFeatureDefocusedLuminance::CFeatureDefocusedLuminance(const CFeatureDefocusedLuminance &f)
  : vpFeatureLuminance(f), pixInfo(NULL), cam()
{
  *this = f;
}

/*!
 Copy operator.
 */
CFeatureDefocusedLuminance &CFeatureDefocusedLuminance::operator=(const CFeatureDefocusedLuminance &f)
{
  Z = f.Z;
  nbr = f.nbr;
  nbc = f.nbc;
  bord = f.bord;
  firstTimeIn = f.firstTimeIn;
  cam = f.cam;
  if (pixInfo)
    delete[] pixInfo;

  pixInfo = new CDefocusedLuminance[dim_s];

  for (unsigned int i = 0; i < dim_s; i++)
    pixInfo[i] = f.pixInfo[i];

  return (*this);
}


/*! 
  Default destructor.
*/
CFeatureDefocusedLuminance::~CFeatureDefocusedLuminance() 
{
  if (pixInfo != NULL)
  {
    delete [] pixInfo ; 
    pixInfo == NULL;
  }
 /* if (flags != NULL)
  {
    delete [] flags;
    flags = NULL;
  }
 */
}

void
CFeatureDefocusedLuminance::setCameraParameters(CCameraThinLensParameters &_cam) 
{
  cam = _cam ;
}

/*!

  Build a luminance feature directly from the image
*/
void
CFeatureDefocusedLuminance::buildFrom(vpImage<unsigned char> &I)
{
	unsigned int l = 0;
  double Ix, Iy;
	double Ixx;

  double px = cam.get_px();
  double py = cam.get_py();

  if (firstTimeIn == 0) {
    firstTimeIn = 1;
    l = 0;
    for (unsigned int i = bord; i < nbr - bord; i++) {
      //   cout << i << endl ;
      for (unsigned int j = bord; j < nbc - bord; j++) {
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, j, i, x, y);

        pixInfo[l].x = x;
        pixInfo[l].y = y;

        pixInfo[l].Z = Z;

        l++;
      }
    }
  }

  l = 0;
  for (unsigned int i = bord; i < nbr - bord; i++) {
      // std::cout << i << std::endl ;
    for (unsigned int j = bord; j < nbc - bord; j++) {
       //std::cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<std::endl ;
      Ix = px * vpImageFilter::derivativeFilterX(I, i, j);
      Iy = py * vpImageFilter::derivativeFilterY(I, i, j);

			Ixx = CImageFilter::laplacianFilterX(I, i, j) + CImageFilter::laplacianFilterY(I, i, j);

      // Calcul de Z

      pixInfo[l].I = I[i][j];
      s[l] = I[i][j];
      pixInfo[l].Ix = Ix;
      pixInfo[l].Iy = Iy;

			pixInfo[l].Ixx  = Ixx;

      l++;
    }
  }

}


/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
CFeatureDefocusedLuminance::interaction(vpMatrix &L)
{
	L.resize(dim_s, 6);
	
	double f = cam.get_f();
	double cste = -cam.get_D()*f/(6.0*cam.get_ku()*(cam.get_Zf()-f));

  for (unsigned int m = 0; m < L.getRows(); m++) {
    double Ix = pixInfo[m].Ix;
    double Iy = pixInfo[m].Iy;

    double x = pixInfo[m].x;
    double y = pixInfo[m].y;
    double Zinv = 1 / pixInfo[m].Z;

		double Z = pixInfo[m].Z;
		double Ixx = pixInfo[m].Ixx;
		double IxxcsteilZ = Ixx*cste/(Z-f);

    {
      L[m][0] = Ix * Zinv;
      L[m][1] = Iy * Zinv;
      L[m][2] = -((x * Ix + y * Iy) * Zinv - IxxcsteilZ);
      L[m][3] = -Ix * x * y - (1 + y * y) * Iy - IxxcsteilZ*(-y*Z);
      L[m][4] = (1 + x * x) * Ix + Iy * x * y - IxxcsteilZ*x*Z;
      L[m][5] = Iy * x - Ix * y;
    }
  }

}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is
  made thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix CFeatureDefocusedLuminance::interaction(const unsigned int /* select */)
{
  /* static */ vpMatrix L; // warning C4640: 'L' : construction of local
                           // static object is not thread-safe
  interaction(L);
  return L;
}


/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
CFeatureDefocusedLuminance*
CFeatureDefocusedLuminance::duplicate() const
{
  CFeatureDefocusedLuminance *feature = new CFeatureDefocusedLuminance ;
  return feature ;
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
