/****************************************************************************
 *
 * July 2011, Jan 2020, October 2020
 *
 * Author:
 * Guillaume Caron
 * inspired from visp/vpImageFilter (Eric Marchand)
 *
 *****************************************************************************/

#include "CImageFilter.h"

/*
  optimizations : 
  - fr access (pointer like)
  - rn and cn access (pointer like)
*/

/*!
 Apply a 1x3 Laplacian Filter to an image pixel.
 
 \param fr : Image to filter
 \param r: coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
CImageFilter::laplacianFilterX(vpImage<unsigned char> & fr, const int r, const int c)
{
	//return fr[r][c+2] + fr[r][c-2] - 2*fr[r][c];

	return (-1421 *fr[r][c]
          -556 *(fr[r][c+1] + fr[r][c-1])
					+527 *(fr[r][c+2] + fr[r][c-2])
					+527 *(fr[r][c+3] + fr[r][c-3])
          +182 *(fr[r][c+4] + fr[r][c-4])
          +29 *(fr[r][c+5] + fr[r][c-5]))/10000;

    /*return (-1065 *fr[r][c]
          -103 *(fr[r][c+1] + fr[r][c-1])
					+526 *(fr[r][c+2] + fr[r][c-2])
					+102 *(fr[r][c+3] + fr[r][c-3])
          +7 *(fr[r][c+4] + fr[r][c-4]))/10000;*/

}

/*!
 Apply a 3x1 Laplacian Filter to an image pixel.
 
 \param fr : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
CImageFilter::laplacianFilterY(vpImage<unsigned char> & fr, const int r, const int c)
{
	//return fr[r+2][c] + fr[r-2][c] -2*fr[r][c];

	return (-1421 *fr[r][c]
          -556 *(fr[r+1][c] + fr[r-1][c])
					+527 *(fr[r+2][c] + fr[r-2][c])
					+527 *(fr[r+3][c] + fr[r-3][c])
          +182 *(fr[r+4][c] + fr[r-4][c])
          +29 *(fr[r+5][c] + fr[r-5][c]))/10000;
  
  /*return (-1065 *fr[r][c]
          -103 *(fr[r+1][c] + fr[r-1][c])
					+526 *(fr[r+2][c] + fr[r-2][c])
					+102 *(fr[r+3][c] + fr[r-3][c])
          +7 *(fr[r+4][c] + fr[r-4][c]))/10000;*/
}


/*!
 Apply a 1x3 Derivative Filter to an image pixel.
 
 \param fr : Image to filter
 \param r: coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
CImageFilter::derivativeFilterX(vpImage<unsigned char> & fr, const int r, const int c)
{
//	return (fr[r][c+1] - fr[r][c-1]);

  return (2047 *(fr[r][c+1] - fr[r][c-1])
					+913 *(fr[r][c+2] - fr[r][c-2])
					+112 *(fr[r][c+3] - fr[r][c-3]))/8418;

  /*return (2297 *(fr[r][c+1] - fr[r][c-1])
					+223 *(fr[r][c+2] - fr[r][c-2])
					+4 *(fr[r][c+3] - fr[r][c-3]))/10000;*/
}

/*!
 Apply a 3x1 Derivative Filter to an image pixel.
 
 \param fr : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
CImageFilter::derivativeFilterY(vpImage<unsigned char> & fr, const int r, const int c)
{
//	return (fr[r+1][c] - fr[r-1][c]);

	return (2047 *(fr[r+1][c] - fr[r-1][c])
					+913 *(fr[r+2][c] - fr[r-2][c])
					+112 *(fr[r+3][c] - fr[r-3][c]))/8418;

	/*return (2297 *(fr[r+1][c] - fr[r-1][c])
					+223 *(fr[r+2][c] - fr[r-2][c])
					+4 *(fr[r+3][c] - fr[r-3][c]))/10000;*/

}

/*!
 Apply a 3 Derivative Filter to an image pixel along any dimension (any list of exact pixelic neighbors)
 
 \param fr : Image to filter
 \param r: coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 \param N: pointer to neighborhood coordinates
 */
double
CImageFilter::derivativeFilter(vpImage<unsigned char> & fr, int r, int c, int**** N)
{
  int *rN = N[r][c][0], *cN = N[r][c][1];

  return (2047.0 *(fr[rN[3]][cN[3]] - fr[rN[2]][cN[2]])
          +913.0 *(fr[rN[4]][cN[4]] - fr[rN[1]][cN[1]])
          +112.0 *(fr[rN[5]][cN[5]] - fr[rN[0]][cN[0]]))*0.00011879306;///8418.0;
}

double
CImageFilter::derivativeFilter_InterpBilinear(vpImage<unsigned char> & fr, int r, int c, float**** N)
{
  float *rN = N[r][c][0], *cN = N[r][c][1];
  float rowI[6];
  int u, v;
  float du, dv, unmdu, unmdv;
  for(int i = 0 ; i < 6 ; i++)
  {
    v = (int)rN[i]; dv = rN[i]-v; unmdv = 1.0f-dv;
    u = (int)cN[i]; du = cN[i]-u; unmdu = 1.0f-du;

    rowI[i] = fr[v][u]*unmdv*unmdu + fr[v+1][u]*dv*unmdu + fr[v][u+1]*unmdv*du + fr[v+1][u+1]*dv*du;
  }
    
  return (2047.0 *(rowI[3] - rowI[2])
          +913.0 *(rowI[4] - rowI[1])
          +112.0 *(rowI[5] - rowI[0]))*0.00011879306;///8418.0;
}

/*!
 Jan. 2020
 Apply a 1x3 Derivative Filter to an image pixel of non integer intensities.
 
 \param fr : Image to filter
 \param r: coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
CImageFilter::derivativeFilterX(vpImage<double> & fr, const int r, const int c)
{
	return (2047.0 *(fr[r][c+1] - fr[r][c-1])
					+913.0 *(fr[r][c+2] - fr[r][c-2])
					+112.0 *(fr[r][c+3] - fr[r][c-3]))*0.000118793062;// /8418.0;
}

/*!
  Jan. 2020
 Apply a 3x1 Derivative Filter to an image pixel of non integer intensities.
 
 \param fr : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
CImageFilter::derivativeFilterY(vpImage<double> & fr, const int r, const int c)
{
	return (2047.0 *(fr[r+1][c] - fr[r-1][c])
					+913.0 *(fr[r+2][c] - fr[r-2][c])
					+112.0 *(fr[r+3][c] - fr[r-3][c]))*0.000118793062;// /8418.0;
}

/*!
  Jan. 2020
 Apply a 3 Derivative Filter to an image pixel of non integer intensities 
 along any dimension (any list of exact pixelic neighbors)
 
 \param fr : Image to filter
 \param r: coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 \param N: pointer to neighborhood coordinates
 */
double
CImageFilter::derivativeFilter(vpImage<double> & fr, int r, int c, int**** N)
{
  int *rN = N[r][c][0], *cN = N[r][c][1];

  return (2047.0 *(fr[rN[3]][cN[3]] - fr[rN[2]][cN[2]])
          +913.0 *(fr[rN[4]][cN[4]] - fr[rN[1]][cN[1]])
          +112.0 *(fr[rN[5]][cN[5]] - fr[rN[0]][cN[0]]))*0.00011879306;///8418.0;
}

/*!
  Jan. 2020
  
*/
double
CImageFilter::derivativeFilter_InterpBilinear(vpImage<double> & fr, int r, int c, float**** N)
{
  float *rN = N[r][c][0], *cN = N[r][c][1];
  float rowI[6];
  int u, v;
  float du, dv, unmdu, unmdv;
  for(int i = 0 ; i < 6 ; i++)
  {
    v = (int)rN[i]; dv = rN[i]-v; unmdv = 1.0f-dv;
    u = (int)cN[i]; du = cN[i]-u; unmdu = 1.0f-du;

    rowI[i] = fr[v][u]*unmdv*unmdu + fr[v+1][u]*dv*unmdu + fr[v][u+1]*unmdv*du + fr[v+1][u+1]*dv*du;
  }
    
  return (2047.0 *(rowI[3] - rowI[2])
          +913.0 *(rowI[4] - rowI[1])
          +112.0 *(rowI[5] - rowI[0]))*0.00011879306;///8418.0;
}
