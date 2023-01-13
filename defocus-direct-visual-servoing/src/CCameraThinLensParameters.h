/*!
  \file CCameraThinLensParameters.h
  \brief Declaration of the CCameraThinLensParameters class.
  Class CCameraThinLensParameters define the camera intrinsic parameters
  October 2020, G. Caron
*/

#ifndef CCAMERATHINLENS_H
#define CCAMERATHINLENS_H

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpColVector.h>

class CCameraThinLensParameters : public vpCameraParameters
{
  //generic functions
public:
  CCameraThinLensParameters() : D(1), FNumber(1), ku(1) {}
  CCameraThinLensParameters(const double f, const double ku,
		     const double FNumber, const double Zf, const double u0, const double v0) ;

  CCameraThinLensParameters& operator =(const CCameraThinLensParameters &c)
	{
		vpCameraParameters::operator=(c);
		this->f = c.f;
    this->ku = c.ku;
    this->FNumber = c.FNumber;
    this->Zf = c.Zf;
		
		return *this;
	}

  virtual ~CCameraThinLensParameters() { }

  void initParamsCamThinLens(const double f, const double ku,
		     const double FNumber, const double Zf, const double u0, const double v0) ;

public:
  inline double get_f() const { return f; }
  inline double get_ku() const { return ku; }
  inline double get_FNumber() const { return FNumber; }
  inline double get_Zf() const { return Zf; }
  inline double get_D() const { return D; }

  void set_f(const double _f) {f = _f; vpCameraParameters::initPersProjWithoutDistortion(f/ku, f/ku, this->get_u0(), this->get_v0()); D = f/FNumber;}
  void set_ku(const double _ku) {ku = _ku; vpCameraParameters::initPersProjWithoutDistortion(f/ku, f/ku, this->get_u0(), this->get_v0());}
  void set_FNumber(const double _FNumber) {f = _FNumber; D = f/FNumber;}
  void set_Zf(const double _Zf) {Zf = _Zf;}

private:
  double f; //!< focal length
  double ku; //!< pixel pitch
  double FNumber; //! camera relative aperture
  double Zf; //! focus depth

  double D; //! camera metric aperture
} ;

#endif //CCAMERATHINLENS_H

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
