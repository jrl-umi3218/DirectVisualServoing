
/*!
  \file CCameraThinLensParameters.cpp
  \brief Definition of the CCameraThinLensParameters class member functions.
  Class CCameraThinLensParameters define the camera intrinsic parameters
  October 2020, G. Caron
*/

#include "CCameraThinLensParameters.h"
//#include <visp/vpDebug.h>
#include <visp/vpException.h>

CCameraThinLensParameters::CCameraThinLensParameters(const double f, const double ku,
		     const double FNumber, const double Zf, const double u0, const double v0) : D(1)
{
  initParamsCamThinLens(f,ku,FNumber,Zf,u0,v0);
}

void CCameraThinLensParameters::initParamsCamThinLens(const double _f, const double _ku,
		     const double _FNumber, const double _Zf, const double u0, const double v0)
{
  f = _f;
  ku = _ku;
  initPersProjWithoutDistortion(f/ku,f/ku,u0,v0);
  Zf = _Zf;
  FNumber = _FNumber;
  
  D = f/FNumber;
}

