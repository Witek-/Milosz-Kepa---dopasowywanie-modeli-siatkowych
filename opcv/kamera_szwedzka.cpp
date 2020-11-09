#include "pch.h"
#include "kamera_szwedzka.hpp"

double    CBRT(double x)

{

	if (x == 0)
		return (0);
	else if (x > 0)
		return (pow(x, (double) 1.0 / 3.0));
	else
		return (-pow(-x, (double) 1.0 / 3.0));
}

void      undistorted_to_distorted_sensor_coord(double Xu, double Yu, double *Xd, double *Yd)
{
#define SQRT3   1.732050807568877293527446341505872366943

	double    Ru,
		Rd,
		lambda,
		c,
		d,
		Q,
		R,
		D,
		S,
		T,
		sinT,
		cosT;

	if (((Xu == 0) && (Yu == 0)) || (cc_kappa1 == 0)) {
		*Xd = Xu;
		*Yd = Yu;
		return;
	}

	Ru = hypot(Xu, Yu);	/* SQRT(Xu*Xu+Yu*Yu) */

	c = 1 / cc_kappa1;
	d = -c * Ru;

	Q = c / 3;
	R = -d / 2;
	D = CUB(Q) + SQR(R);

	if (D >= 0) {		/* one real root */
		D = SQRT(D);
		S = CBRT(R + D);
		T = CBRT(R - D);
		Rd = S + T;

		if (Rd < 0) {
			Rd = SQRT(-1 / (3 * cc_kappa1));
			fprintf(stderr, "\nWarning: undistorted image point to distorted image point mapping limited by\n");
			fprintf(stderr, "         maximum barrel distortion radius of %lf\n", Rd);
			fprintf(stderr, "         (Xu = %lf, Yu = %lf) -> (Xd = %lf, Yd = %lf)\n\n",
				Xu, Yu, Xu * Rd / Ru, Yu * Rd / Ru);
		}
	}
	else {			/* three real roots */
		D = SQRT(-D);
		S = CBRT(hypot(R, D));
		T = atan2(D, R) / 3;
		SINCOS(T, sinT, cosT);

		/* the larger positive root is    2*S*cos(T)                   */
		/* the smaller positive root is   -S*cos(T) + SQRT(3)*S*sin(T) */
		/* the negative root is           -S*cos(T) - SQRT(3)*S*sin(T) */

		Rd = -S * cosT + SQRT3 * S * sinT;	/* use the smaller positive root */
	}

	lambda = Rd / Ru;

	*Xd = Xu * lambda;
	*Yd = Yu * lambda;
}

void      world_coord_to_image_coord(double xw, double yw, double zw, double *Xf, double *Yf)
{
	double    xc,
		yc,
		zc,
		Xu,
		Yu,
		Xd,
		Yd;

	/* convert from world coordinates to camera coordinates */
	xc = cc_r1 * xw + cc_r2 * yw + cc_r3 * zw + cc_Tx;
	yc = cc_r4 * xw + cc_r5 * yw + cc_r6 * zw + cc_Ty;
	zc = cc_r7 * xw + cc_r8 * yw + cc_r9 * zw + cc_Tz;

	/* convert from camera coordinates to undistorted sensor plane coordinates */
	Xu = cc_f * xc / zc;
	Yu = cc_f * yc / zc;

	/* convert from undistorted to distorted sensor plane coordinates */
	undistorted_to_distorted_sensor_coord(Xu, Yu, &Xd, &Yd);

	/* convert from distorted sensor plane coordinates to image coordinates */
	*Xf = Xd * cp_sx / cp_dpx + cp_Cx;
	*Yf = Yd / cp_dpy + cp_Cy;
}

void projectPoints2(std::vector<cv::Point3d> &punkty_swiat, std::vector<cv::Point2d> &punkty_kamera)
{
	punkty_kamera.clear();
	for (cv::Point3d ps : punkty_swiat)
	{
		double X, Y;
		world_coord_to_image_coord(ps.x, ps.y, ps.z, &X, &Y);
		punkty_kamera.push_back(cv::Point2d(X, Y));
	}

}

void projectPoints2(std::vector<cv::Point3f>& punkty_swiat, std::vector<cv::Point2f>& punkty_kamera)
{
	punkty_kamera.clear();
	for (cv::Point3f ps : punkty_swiat)
	{	
		double X, Y;
		world_coord_to_image_coord(ps.x, ps.y, ps.z, &X, &Y);
		punkty_kamera.push_back(cv::Point2f(X, Y));
	}

}

