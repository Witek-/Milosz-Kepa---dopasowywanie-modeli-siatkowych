#pragma once

#include <opencv2/opencv.hpp>
#include <omp.h>

#define ITERATIONS 3
#define ANGLE_STEP 5
#define ANGLE_NO_STEPS 72
#define X_STEP 0.1
#define X_NO_STEPS 11
#define Y_STEP 0.1
#define Y_NO_STEPS 11
#define S_START 1.5
#define S_STEP 0.1
#define S_NO_STEPS 6
#define D_START 4.0
#define D_STEP 0.2
#define D_NO_STEPS 12
#define W_START 1.2
#define W_STEP 0.1
#define W_NO_STEPS 11
#define A_START 0.5
#define A_STEP 0.1
#define A_NO_STEPS 18
#define B_START 0.3
#define B_STEP 0.1
#define B_NO_STEPS 13
#define AT_START 0.2
#define AT_STEP 0.1
#define AT_NO_STEPS 15
#define BT_START 0.0
#define BT_STEP 0.1
#define BT_NO_STEPS 10

using namespace std;
using namespace cv;

class CarModel
{
private:
	vector<Point3f> model3D;
	vector<Point2f> model2D;
	Rect roi;
	Mat mask;
	Mat MacierzKamery, WspolczynnikiZnieksztalcen, MacierzRotacji, WektorTranslacji;
	//parametry w 3D podlegajace optymalizacji:
	int angle = 0, optimal_cost = 50000;
	float x = 0.0, y = 0.0, z = 0.0, s = 0.0, d = 0.0, w = 0.0, a = 0.0, b = 0.0, at = 0.0, bt = 0.0;
	//parametry w 3D poczatkowe
	int angle_start = 0;
	float x_start = 0.0, y_start = 0.0;

	Point3f rotateOverZ(Point3f, Point3f, int);
	void setDefaultParams();
	void setRandomParams();
	void setRandomParamsReduced(CarModel);
	void calculateStartXY();
	void buildModel();
	
	int calcCost();
	int optimizeParam(string param, float step, int no_steps, float init_value);
	int optimizeDoubleParam(string param, float step, float step2, int no_steps, int no_steps2, float init_value, float init_value2);
	int optimizeParamInt(string param, int step, int no_steps, int init_value);
	int tryReversed(int angle);
	void drawArea(Mat& obraz, Point offset);

public:
	CarModel();
	CarModel(Rect roi_, Mat mask_, Mat MacierzKamery_, Mat WspolczynnikiZnieksztalcen_, Mat MacierzRotacji_, Mat WektorTranslacji_);

	~CarModel();

	//getters
	int getAngle() { return angle; }
	float getX() { return x; }
	float getY() { return y; }
	float getZ() { return z; }
	float getS() { return s; }
	float getD() { return d; }
	float getW() { return w; }
	float getA() { return a; }
	float getB() { return b; }
	float getAt() { return at; }
	float getBt() { return bt; }
	Rect getRoi() { return roi; }
	Mat getMask() { return mask; }
	vector<Point3f> getModel3D() { return model3D; }
	vector<Point2f> getModel2D() { return model2D; }
	int getAngleStart() { return angle_start; }
	float getXStart() { return x_start; }
	float getYStart() { return y_start; }
	int getOptimalCost() { return optimal_cost; }

	//setters
	void setAngle(int angle_) { angle = angle_; }
	void setX(float x_) { x = x_; }
	void setY(float y_) { y = y_; }
	void setZ(float z_) { z = z_; }
	void setS(float s_) { s = s_; }
	void setD(float d_) { d = d_; }
	void setW(float w_) { w = w_; }
	void setA(float a_) { a = a_; }
	void setB(float b_) { b = b_; }
	void setAt(float a_) { at = a_; }
	void setBt(float b_) { bt = b_; }
	void setRoi(Rect roi_) { roi = roi_; }
	void setMask(Mat mask_) { mask = mask_; }
	void setModel3D(vector<Point3f> model_) { model3D = model_; }
	void setModel2D(vector<Point2f> model_) { model2D = model_; }
	void setAngleStart(int angle_) { angle_start = angle_; }
	void setXStart(float x_) { x_start = x_; }
	void setYStart(float y_) { y_start = y_; }
	void setMK(Mat MK_) { MacierzKamery = MK_; }
	void setMR(Mat MR_) { MacierzRotacji = MR_; }
	void setWZ(Mat WZ_) { WspolczynnikiZnieksztalcen = WZ_; }
	void setWT(Mat WT_) { WektorTranslacji = WT_; }
	void setAllParameters(int, float, float, float, float, float, float, float, float, float, float);
	void setOptimalCost(int cost_) { optimal_cost = cost_; }

	vector<Point3f> returnModel3D();
	vector<Point2f> returnModel2D();
	void draw(Mat& obraz2);
	void draw2(Mat& obraz2);
	void optimize(bool verbose=0);
	int get2DArea();
	void drawMask(Mat& obraz, Scalar c);
	void optimizeReduced(CarModel optimal, int last_angle);
	void buildModel2();
	int getMaskArea();
};

