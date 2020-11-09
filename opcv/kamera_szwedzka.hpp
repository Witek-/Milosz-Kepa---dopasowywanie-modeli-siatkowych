#pragma once

#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

#define cp_Ncx 1920					/* [sel]     Number of sensor elements in camera's x direction */
#define cp_Nfx 1080					/* [pix]     Number of pixels in frame grabber's x direction   */
#define cp_dx 1						/* [mm/sel]  X dimension of camera's sensor element (in mm)    */
#define cp_dy 1						/* [mm/sel]  Y dimension of camera's sensor element (in mm)    */
#define cp_dpx 1					/* [mm/pix]  Effective X dimension of pixel in frame grabber   */
#define cp_dpy 1					/* [mm/pix]  Effective Y dimension of pixel in frame grabber   */
#define cp_Cx 978.600918124883		/* [pix]     Z axis intercept of camera coordinate system      */
#define cp_Cy 597.394388425262		/* [pix]     Z axis intercept of camera coordinate system      */
#define cp_sx 1.0					/* []        Scale factor to compensate for any error in dpx   */



#define cc_f 1053.20082646788		/* [mm]          */
#define cc_kappa1 7.1373582508209E-07		/* [1/mm^2]      */
#define cc_p1 0 		/* [1/mm]        */
#define cc_p2 0		/* [1/mm]        */
#define cc_Tx -0.509928181303698		/* [mm]          */
#define cc_Ty -2.03089277266409		/* [mm]          */
#define cc_Tz 15.3755839967626		/* [mm]          */
#define cc_Rx 0		/* [rad]	 */
#define cc_Ry 0		/* [rad]	 */
#define cc_Rz 0		/* [rad]	 */
#define cc_r1 -0.478890620701837		/* []            */
#define cc_r2 0.877825461931066		/* []            */
#define cc_r3 0.00928610732871105		/* []            */
#define cc_r4 -0.632745109288807		/* []            */
#define cc_r5 -0.352482746394124		/* []            */
#define cc_r6 0.689484981827415		/* []            */
#define cc_r7 0.608520665281718		/* []            */
#define cc_r8 0.324312151915353		/* []            */
#define cc_r9 0.72424044905344		/* []            */



#define SQR(a)          ((a) * (a))
#define CUB(a)          ((a)*(a)*(a))
#define SINCOS(x,s,c)   s=sin(x);c=cos(x)
#define SQRT(x) sqrt(fabs(x))



double CBRT(double x);

void undistorted_to_distorted_sensor_coord(double Xu, double Yu, double* Xd, double* Yd);

void world_coord_to_image_coord(double xw, double yw, double zw, double* Xf, double* Yf);

void projectPoints2(std::vector<cv::Point3d>& punkty_swiat, std::vector<cv::Point2d>& punkty_kamera);

void projectPoints2(std::vector<cv::Point3f>& punkty_swiat, std::vector<cv::Point2f>& punkty_kamera);


