/****************************************************************************
 *
 * July 2011
 *
 * Author:
 * Guillaume Caron
 * inspired from visp/vpImageFilter (Eric Marchand)
 *
 *****************************************************************************/

#ifndef CImageFilter_H
#define CImageFilter_H

/*!
  \file CImageFilter.h
  \brief  Various directional image filter, convolution, etc...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpConfig.h>
#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>
#include <visp/vpMath.h>

/*!
  \class CImageFilter

  \ingroup ImageFiltering

  \brief  Various image filter, convolution, etc...

*/
class CImageFilter
{

public:
  static double  laplacianFilterX(vpImage<unsigned char> &I,
				   const int r, int c) ;

  static double  laplacianFilterY(vpImage<unsigned char> &I,
				   const int r, int c) ;

  static double  derivativeFilterX(vpImage<unsigned char> &I,
				   const int r, int c) ;

  static double  derivativeFilterY(vpImage<unsigned char> &I,
				   const int r, int c) ;

  static double  derivativeFilter(vpImage<unsigned char> &I,
				   const int r, int c, int**** N) ;

  static double  derivativeFilter_InterpBilinear(vpImage<unsigned char> &I,
				   const int r, int c, float**** N) ;

  //same as aboves but for vpImage<double> images
  static double  derivativeFilterX(vpImage<double> &I,
				   const int r, int c) ;

  static double  derivativeFilterY(vpImage<double> &I,
				   const int r, int c) ;

  static double  derivativeFilter(vpImage<double> &I,
				   const int r, int c, int**** N) ;

  static double  derivativeFilter_InterpBilinear(vpImage<double> &I,
				   const int r, int c, float**** N) ;

} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
