#include "pch.h"
#include "CarModel.hpp"
//#include <opencv2/opencv.hpp>

/** USTAWIENIE SPOWODUJE WLACZENIE TRYBU OBSERWACJI OPTYMALIZACJI
*/
#define DEBUGGING 0

CarModel::CarModel() 
{
	CarModel::roi = Rect(0, 0, 0, 0);
}

CarModel::~CarModel() {}

CarModel::CarModel(Rect roi_, Mat mask_, Mat MacierzKamery_, Mat WspolczynnikiZnieksztalcen_, Mat MacierzRotacji_, Mat WektorTranslacji_)
{
	CarModel::setRoi(roi_);
	CarModel::setMask(mask_);
	CarModel::setMK(MacierzKamery_);
	CarModel::setWZ(WspolczynnikiZnieksztalcen_);
	CarModel::setMR(MacierzRotacji_);
	CarModel::setWT(WektorTranslacji_);
	// parametry poczatkowe
	CarModel::setAngleStart(rand() % 360);
	CarModel::calculateStartXY();
	CarModel::setDefaultParams();
}

void CarModel::setDefaultParams() 
{
	CarModel::setX(CarModel::getXStart());
	CarModel::setY(CarModel::getYStart());
	CarModel::setZ(0.0);
	CarModel::setS(S_START);
	CarModel::setD(D_START);
	CarModel::setW(W_START);
	CarModel::setA(A_START);
	CarModel::setB(B_START);
	CarModel::setAt(AT_START);
	CarModel::setBt(BT_START);
	CarModel::setAngle(CarModel::getAngleStart());
}

void CarModel::setRandomParams()
{
	CarModel::setX(CarModel::getXStart());
	CarModel::setY(CarModel::getYStart());
	CarModel::setZ(0.0);
	int r = rand() % 5;
	CarModel::setS(S_START + 0.1 * r);
	r = rand() % 15;
	CarModel::setD(D_START + 0.2 * r);
	r = rand() % 10;
	CarModel::setW(W_START + 0.1 * r);
	r = rand() % 10;
	CarModel::setB(B_START + 0.1 * r);
	r = rand() % 5;
	CarModel::setA(CarModel::getB() + 0.1 * r);
	r = rand() % 10;
	CarModel::setBt(BT_START + 0.1 * r);
	r = rand() % 5;
	CarModel::setAt(CarModel::getBt() + 0.1 * r);
	
	CarModel::setAngle(CarModel::getAngleStart());
}

void CarModel::setRandomParamsReduced(CarModel optimal)
{
	CarModel::setX(CarModel::getXStart());
	CarModel::setY(CarModel::getYStart());
	CarModel::setZ(0.0);
	CarModel::setS(optimal.getS());
	CarModel::setD(optimal.getD());
	CarModel::setW(optimal.getW());
	CarModel::setA(optimal.getA());
	CarModel::setB(optimal.getB());
	CarModel::setAt(optimal.getAt());
	CarModel::setBt(optimal.getBt());
	//CarModel::setAngle(CarModel::getAngleStart());
}

void CarModel::calculateStartXY()
{
	int xroi = CarModel::roi.x + CarModel::roi.width/2;
	int yroi = CarModel::roi.y - CarModel::roi.height/2;
	int wys = CarModel::roi.height;

	//punkt poczatkowy modelu
	vector<Point3f> p1{ { 0, 0, 0 } };
	vector<Point2f> p2, p2_last;

	//Mat obraz(1080, 1920, CV_8UC3, Scalar(0,0,0));

	//rzutowanie punktu na plaszczyzne obrazu
	projectPoints(p1, CarModel::MacierzRotacji, CarModel::WektorTranslacji, CarModel::MacierzKamery, CarModel::WspolczynnikiZnieksztalcen, p2);
	p2_last = p2;
	//ustawienie modelu w okolicy ROI - punkt poczatkowy modelu
	//przesuwanie punktow w 3d, i sprawdzanie ich wspolrzednych w 2d, czy zgadzaja sie z poczatkiem roi
	while (int(p2[0].x) < xroi) {
		p1[0].x += .1;
		p1[0].y -= .5;
		projectPoints(p1, CarModel::MacierzRotacji, CarModel::WektorTranslacji, CarModel::MacierzKamery, CarModel::WspolczynnikiZnieksztalcen, p2);
		while (int(p2[0].y) > yroi + wys) {
			p1[0].y += .1;
			projectPoints(p1, CarModel::MacierzRotacji, CarModel::WektorTranslacji, CarModel::MacierzKamery, CarModel::WspolczynnikiZnieksztalcen, p2);
		}
		//line(obraz, p2[0], p2_last[0], Scalar(255, 0, 0));
		//p2_last = p2;
	}
	//imshow("xy",obraz);
	//waitKey();
	// zwiekszanie o 0.5 zeby od razu przesunac blizej srodka roi
	CarModel::setXStart(p1[0].x);
	CarModel::setYStart(p1[0].y);
}

Point3f CarModel::rotateOverZ(Point3f point, Point3f center, int degrees) {
	float x = point.x;
	float y = point.y;
	float z = point.z;
	float rad = degrees * 3.14 / 180;
	Mat R = (Mat_<float>(3, 3) << cos(rad), -sin(rad), 0,
		sin(rad), cos(rad), 0,
		0, 0, 1);
	Point3f result{ x, y, z };
	Mat v = (Mat_<float>(3, 1) << result.x - center.x, result.y - center.y, result.z);
	v = R * v;
	result.x = v.at<float>(0, 0) + center.x;
	result.y = v.at<float>(1, 0) + center.y;
	result.z = v.at<float>(2, 0);
	return result;
}

void CarModel::buildModel() {
	vector<Point2f> p2_mod;
	vector<Point3f> p1_mod = { { x, y, z }, { x + d, y, z }, { x + d, y + s, z }, { x, y + s, z },
									 { x, y, z + w }, { x + d, y, z + w - a }, { x + d, y + s, z + w - a }, { x, y + s, z + w },
									 { x + d - b, y, z + w }, { x + d - b, y + s, z + w } };
	Point3f c = { x + d / 2, y + s / 2, 0 };
	for (int i = 0; i < p1_mod.size(); i++) {
		p1_mod[i] = CarModel::rotateOverZ(p1_mod[i], c, angle);
	}
	CarModel::setModel3D(p1_mod);
	projectPoints(p1_mod, CarModel::MacierzRotacji, CarModel::WektorTranslacji, CarModel::MacierzKamery, CarModel::WspolczynnikiZnieksztalcen, p2_mod);
	CarModel::setModel2D(p2_mod);
}

void CarModel::buildModel2() {
	vector<Point2f> p2_mod;
	float wys = float(2.0 / 3.0);
	float dwa = float(2.0);
	vector<Point3f> p1_mod = {	{ x - d/dwa, y - s/dwa, z }, { x + d/dwa, y - s/dwa, z }, { x + d/dwa, y + s/dwa, z }, { x - d/dwa, y + s/dwa, z },
								{ x - d/dwa, y - s/dwa, z + wys * w }, { x - d/dwa + bt, y - s/dwa, z + wys * w }, {x - d/dwa + bt, y + s/dwa, z + wys * w}, {x - d/dwa, y + s/dwa, z + wys * w},
								{x - d/dwa + at, y - s/dwa, z + w}, {x + d/dwa - a, y - s/dwa, z + w}, {x + d/dwa - a, y + s/dwa, z + w}, {x - d/dwa + at, y + s/dwa, z + w},
								{x + d/dwa - b, y - s/dwa, z + wys * w}, {x + d/dwa, y - s/dwa, z + wys * w}, {x + d/dwa, y + s/dwa, z + wys * w}, {x + d/dwa - b, y + s/dwa, z + wys * w}
							};
	Point3f c = { x, y, 0 };
	for (int i = 0; i < p1_mod.size(); i++) {
		p1_mod[i] = CarModel::rotateOverZ(p1_mod[i], c, angle);
	}
	CarModel::setModel3D(p1_mod);
	projectPoints(p1_mod, CarModel::MacierzRotacji, CarModel::WektorTranslacji, CarModel::MacierzKamery, CarModel::WspolczynnikiZnieksztalcen, p2_mod);
	CarModel::setModel2D(p2_mod);
}

int CarModel::get2DArea() 
{
	vector<Point2f> p2_mod = CarModel::getModel2D();
	vector<Point2f> hull;
	vector<Point> polygon = {};
	convexHull(p2_mod, hull, 1, 1);
	for (int i = 0; i < hull.size(); i++) {
		polygon.push_back(Point(int(hull[i].x), int(hull[i].y)));
	}
	Mat czarny_prostokat(1080/4, 1920/4, CV_8U, Scalar(0));
	//Mat czarny_prostokat(int(1.5*CarModel::roi.height), int(1.5*CarModel::roi.width), CV_8U, Scalar(0));
	Mat xo;
	drawContours(czarny_prostokat, vector<vector<Point>>{polygon}, -1, Scalar(255), FILLED, 8, noArray(), 2147483647, Point(-CarModel::roi.x, -CarModel::roi.y));
	return countNonZero(czarny_prostokat);
}

void CarModel::drawArea(Mat& obraz, Point offset)
{
	// dolny prostopadloscian: 0 1 2 3 4 7 13 14
	// gorny ksztalt: 5 6 8 9 10 11 12 15
	CarModel::buildModel2();
	vector<Point2f> m = CarModel::getModel2D();
	vector<Point2f> dol = { m[0],m[1],m[2],m[3],m[4],m[7],m[13],m[14] };
	vector<Point2f> hull;
	vector<Point> polygon;
	convexHull(dol, hull, 1, 1);
	for (int i = 0; i < hull.size(); i++) {
		polygon.push_back(Point(int(hull[i].x), int(hull[i].y)));
	}
	drawContours(obraz, vector<vector<Point>>{polygon}, -1, Scalar(255), FILLED, 8, noArray(), 2147483647, offset);
	vector<Point2f> gora = { m[5],m[6],m[8],m[9],m[10],m[11],m[12],m[15] };
	vector<Point2f> hull2;
	vector<Point> polygon2;
	convexHull(gora, hull2, 1, 1);
	for (int i = 0; i < hull.size(); i++) {
		polygon2.push_back(Point(int(hull2[i].x), int(hull2[i].y)));
	}
	drawContours(obraz, vector<vector<Point>>{polygon2}, -1, Scalar(255), FILLED, 8, noArray(), 2147483647, offset);
}

int CarModel::getMaskArea()
{
	Mat maska = 255 * CarModel::mask;
	return countNonZero(maska);
}

int CarModel::calcCost()
{
	int cost;
	//vector<Point2f> p2_mod, hull;
	//operacje na punktach
	//CarModel::buildModel2();
	//p2_mod = CarModel::getModel2D();
	//vector<Point> polygon = {};
	//convexHull(p2_mod, hull, 1, 1);
	//for (int i = 0; i < hull.size(); i++) {
	//	polygon.push_back(Point(int(hull[i].x), int(hull[i].y)));
	//}
	//operacje na obrazach
	int xmargin = int(1.1 * CarModel::roi.width);
	int ymargin = int(1.1 * CarModel::roi.height);
	Mat czarny_prostokat(CarModel::roi.height + 2 * ymargin, CarModel::roi.width + 2 * xmargin, CV_8U, Scalar(0));
	Mat maska(CarModel::roi.height + 2 * ymargin, CarModel::roi.width + 2 * xmargin, CV_8U, Scalar(0));
	Mat xo;
	CarModel::drawArea(czarny_prostokat, Point(-CarModel::roi.x + xmargin, -CarModel::roi.y + ymargin));
	//drawContours(czarny_prostokat, vector<vector<Point>>{polygon}, -1, Scalar(255), FILLED, 8, noArray(), 2147483647, Point(-CarModel::roi.x + xmargin, -CarModel::roi.y + ymargin));
	Rect r(xmargin, ymargin, CarModel::mask.cols, CarModel::mask.rows);
	maska(r) = 255*CarModel::mask;
	cv::bitwise_xor(maska, czarny_prostokat, xo);
	
	//SEKCJA DEBUGOWANIA DOPASOWANIA
	//U¿ywaæ w celu obserwacji krok po kroku dopasowania modelu - odpowiednia klatke mozna ustawic w glownym programie (zmienna klatka)
	//Warto zakomentowac pragme omp parallel, zeby uniknac wielu okien

	if (DEBUGGING) {
		Mat test(1080, 1920, CV_8UC3, Scalar(0, 0, 0));
		Mat m = CarModel::mask.clone();
		cvtColor(m, m, COLOR_GRAY2BGR);
		test(CarModel::roi) = m * 255;
		CarModel::draw2(test);
		imshow("1", test);
		imshow("2", xo);
		imshow("3", czarny_prostokat);
		waitKey();
	}

	cost = countNonZero(xo);
	return cost;
}

/*
int CarModel::calcCost()
{
	int cost;
	CarModel::buildModel();
	int area = CarModel::get2DArea();
	return abs(area - countNonZero(CarModel::getMask()));
}
*/
//dla optymalizacji parametru int
int CarModel::optimizeParamInt(string param, int step, int no_steps, int init_value)
{
	int cost, cost_min = 50000, i_min = 0;
	for (int i = 0; i < no_steps; i++)
	{
		if (param.compare("angle")==0) CarModel::setAngle(init_value + i * step);
		cost = CarModel::calcCost();
		if (cost <= cost_min) {
			cost_min = cost;
			i_min = i;
		}
	}
	if (param.compare("angle")==0) CarModel::setAngle((init_value + i_min * step)%360);
	return cost_min;
}

//dla optymalizacji parametru float
int CarModel::optimizeParam(string param, float step, int no_steps, float init_value)
{
	int cost, cost_min = 50000, i_min = 0;
	for (int i = 0; i < no_steps; i++)
	{
		if (param.compare("x")==0) CarModel::setX(init_value + i * step);
		if (param.compare("y")==0) CarModel::setY(init_value + i * step);
		if (param.compare("z")==0) CarModel::setZ(init_value + i * step);
		if (param.compare("s")==0) CarModel::setS(init_value + i * step);
		if (param.compare("d")==0) CarModel::setD(init_value + i * step);
		if (param.compare("w")==0) CarModel::setW(init_value + i * step);
		if (param.compare("a")==0) CarModel::setA(init_value + i * step);
		if (param.compare("b")==0) CarModel::setB(init_value + i * step);
		if (param.compare("at")==0) CarModel::setAt(init_value + i * step);
		if (param.compare("bt")==0) CarModel::setBt(init_value + i * step);

		cost = CarModel::calcCost();
		if (cost < cost_min) {
			cost_min = cost;
			i_min = i;
		}
	}

	if (param.compare("x")==0) CarModel::setX(init_value + i_min * step);
	if (param.compare("y")==0) CarModel::setY(init_value + i_min * step);
	if (param.compare("z")==0) CarModel::setZ(init_value + i_min * step);
	if (param.compare("s")==0) CarModel::setS(init_value + i_min * step);
	if (param.compare("d")==0) CarModel::setD(init_value + i_min * step);
	if (param.compare("w")==0) CarModel::setW(init_value + i_min * step);
	if (param.compare("a")==0) CarModel::setA(init_value + i_min * step);
	if (param.compare("b")==0) CarModel::setB(init_value + i_min * step);
	if (param.compare("at")==0) CarModel::setAt(init_value + i_min * step);
	if (param.compare("bt")==0) CarModel::setBt(init_value + i_min * step);
	return cost_min;
}

int CarModel::optimizeDoubleParam(string param, float step, float step2, int no_steps, int no_steps2, float init_value, float init_value2)
{
	int cost, cost_min = 50000, i_min = 0, j_min = 0;
	for (int i = 0; i < no_steps; i++)
	{
		for (int j = 0; j < no_steps2; j++)
		{
			if (param.compare("front") == 0) { CarModel::setA(init_value + i * step); CarModel::setB(init_value2 + j * step2); }
			if (param.compare("back") == 0) { CarModel::setAt(init_value + i * step); CarModel::setBt(init_value2 + j * step2); }

			cost = CarModel::calcCost();
			if (cost <= cost_min) {
				cost_min = cost;
				i_min = i;
				j_min = j;
			}
		}
	}

	if (param.compare("front") == 0) { CarModel::setA(init_value + i_min * step); CarModel::setB(init_value2 + j_min * step2); }
	if (param.compare("back") == 0) { CarModel::setAt(init_value + i_min * step); CarModel::setBt(init_value2 + j_min * step2); }
	return cost_min;
}

void CarModel::setAllParameters(int angle_, float x_, float y_, float z_, float s_, float d_, float w_, float a_, float b_, float at_, float bt_)
{
	CarModel::setAngle(angle_);
	CarModel::setX(x_);
	CarModel::setY(y_);
	CarModel::setZ(z_);
	CarModel::setS(s_);
	CarModel::setD(d_);
	CarModel::setW(w_);
	CarModel::setA(a_);
	CarModel::setB(b_);
	CarModel::setAt(at_);
	CarModel::setBt(bt_);
}

//public
vector<Point3f> CarModel::returnModel3D()
{
	CarModel::buildModel();
	return CarModel::getModel3D();
}

vector<Point2f> CarModel::returnModel2D()
{
	CarModel::buildModel();
	return CarModel::getModel2D();
}

void CarModel::draw(Mat& obraz2) {
	vector<Point2f> p2_mod = CarModel::getModel2D();
	line(obraz2, p2_mod[0], p2_mod[1], CV_RGB(255, 0, 0), 2);
	line(obraz2, p2_mod[1], p2_mod[2], CV_RGB(255, 0, 0), 2);
	line(obraz2, p2_mod[2], p2_mod[3], CV_RGB(255, 0, 0), 2);
	line(obraz2, p2_mod[3], p2_mod[0], CV_RGB(255, 0, 0), 2);

	line(obraz2, p2_mod[4], p2_mod[8], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[8], p2_mod[9], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[9], p2_mod[7], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[7], p2_mod[4], CV_RGB(255, 255, 0), 2);

	line(obraz2, p2_mod[0], p2_mod[4], CV_RGB(255, 0, 255), 2);
	line(obraz2, p2_mod[3], p2_mod[7], CV_RGB(255, 0, 255), 2);

	line(obraz2, p2_mod[1], p2_mod[5], CV_RGB(0, 255, 0), 2);
	line(obraz2, p2_mod[2], p2_mod[6], CV_RGB(0, 255, 0), 2);

	line(obraz2, p2_mod[5], p2_mod[8], CV_RGB(0, 0, 255), 2);
	line(obraz2, p2_mod[6], p2_mod[9], CV_RGB(0, 0, 255), 2);
	line(obraz2, p2_mod[5], p2_mod[6], CV_RGB(0, 0, 255), 2);
}

void CarModel::draw2(Mat& obraz2) {
	vector<Point2f> p2_mod = CarModel::getModel2D();

	//podwozie
	line(obraz2, p2_mod[0], p2_mod[1], CV_RGB(255, 0, 0), 2);
	line(obraz2, p2_mod[1], p2_mod[2], CV_RGB(255, 0, 0), 2);
	line(obraz2, p2_mod[2], p2_mod[3], CV_RGB(255, 0, 0), 2);
	line(obraz2, p2_mod[3], p2_mod[0], CV_RGB(255, 0, 0), 2);

	//gora_tyl
	line(obraz2, p2_mod[4], p2_mod[5], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[5], p2_mod[6], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[6], p2_mod[7], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[7], p2_mod[4], CV_RGB(255, 255, 0), 2);

	//gora
	line(obraz2, p2_mod[8], p2_mod[9], CV_RGB(0, 255, 0), 2);
	line(obraz2, p2_mod[9], p2_mod[10], CV_RGB(0, 255, 0), 2);
	line(obraz2, p2_mod[10], p2_mod[11], CV_RGB(0, 255, 0), 2);
	line(obraz2, p2_mod[11], p2_mod[8], CV_RGB(0, 255, 0), 2);

	//gora_przod
	line(obraz2, p2_mod[12], p2_mod[13], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[13], p2_mod[14], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[14], p2_mod[15], CV_RGB(255, 255, 0), 2);
	line(obraz2, p2_mod[15], p2_mod[12], CV_RGB(255, 255, 0), 2);

	//tyl
	line(obraz2, p2_mod[0], p2_mod[4], CV_RGB(255, 0, 255), 2);
	line(obraz2, p2_mod[3], p2_mod[7], CV_RGB(255, 0, 255), 2);

	//skos_tyl
	line(obraz2, p2_mod[5], p2_mod[8], CV_RGB(0, 255, 0), 2);
	line(obraz2, p2_mod[6], p2_mod[11], CV_RGB(0, 255, 0), 2);

	//skos_przod
	line(obraz2, p2_mod[9], p2_mod[12], CV_RGB(0, 255, 0), 2);
	line(obraz2, p2_mod[10], p2_mod[15], CV_RGB(0, 255, 0), 2);

	//przod
	line(obraz2, p2_mod[1], p2_mod[13], CV_RGB(255, 0, 255), 2);
	line(obraz2, p2_mod[2], p2_mod[14], CV_RGB(255, 0, 255), 2);
}

int CarModel::tryReversed(int angle_)
{
	int cost_old = CarModel::getOptimalCost();
	CarModel::setAngle(angle_ + 180);
	int cost_new = CarModel::calcCost();
	if (cost_new > cost_old) return 1;
	else return 0;
}

void CarModel::optimize(bool verbose)
{
	//CarModel::calculateStartXY();
	int angle_opt = CarModel::angle_start;
	float x_opt = CarModel::x_start, y_opt = CarModel::y_start, z_opt = 0.0, s_opt = S_START, d_opt = D_START, w_opt = W_START, a_opt = A_START, b_opt = B_START, at_opt = AT_START, bt_opt = BT_START;
	int cost = 50000, cost_min = 50000, cost_p = 50000, iteracje = 0;
	if(verbose) cout << "Optymalizuje..." << endl;

	for (int iter = 0; iter < ITERATIONS; iter++)
	{
		CarModel::setRandomParams();
		CarModel::setAngle(CarModel::getAngleStart() + int(iter * 360 / ITERATIONS));
		do {
			CarModel::optimizeParamInt("angle", ANGLE_STEP, ANGLE_NO_STEPS, CarModel::getAngle());
			CarModel::setX(CarModel::getX() - (X_NO_STEPS - 1.0) / 20.0);
			CarModel::optimizeParam("x", X_STEP, X_NO_STEPS, CarModel::getX());
			CarModel::setY(CarModel::getY() - (Y_NO_STEPS - 1.0) / 20.0);
			CarModel::optimizeParam("y", Y_STEP, Y_NO_STEPS, CarModel::getY());
			//CarModel::setD(D_START);
			CarModel::optimizeParam("d", D_STEP, D_NO_STEPS, D_START);
			//CarModel::setS(S_START);
			CarModel::optimizeParam("s", S_STEP, S_NO_STEPS, S_START);
			//CarModel::setW(W_START);
			CarModel::optimizeParam("w", W_STEP, W_NO_STEPS, W_START);
			/*
			CarModel::setB(B_START);
			CarModel::optimizeParam("b", B_STEP, B_NO_STEPS, CarModel::getB());
			CarModel::setA(CarModel::getB());
			CarModel::optimizeParam("a", A_STEP, A_NO_STEPS, CarModel::getA());
			CarModel::setBt(BT_START);
			CarModel::optimizeParam("bt", BT_STEP, BT_NO_STEPS, CarModel::getBt());
			CarModel::setAt(CarModel::getBt());
			CarModel::optimizeParam("at", AT_STEP, AT_NO_STEPS, CarModel::getAt());
			*/
			//CarModel::optimizeParam("angle", ANGLE_STEP, ANGLE_NO_STEPS, CarModel::getAngle());
			//CarModel::setX(CarModel::getX() - (X_NO_STEPS - 1.0) / 20.0);
			
			
			/*
			//CarModel::setA(A_START);
			CarModel::optimizeParam("a", A_STEP, A_NO_STEPS, A_START);
			//CarModel::setB(B_START);
			int steps = int(CarModel::getA() / A_STEP);
           	CarModel::optimizeParam("b", B_STEP, steps - B_START - 2, B_START);
			//CarModel::setAt(AT_START);
			CarModel::optimizeParam("at", AT_STEP, AT_NO_STEPS, AT_START);
			//CarModel::setBt(BT_START);
			steps = int(CarModel::getAt() / AT_STEP);
			CarModel::optimizeParam("bt", BT_STEP, steps - BT_START, BT_START);
			*/
			float front_start = 0.2 * d;
			float front_step = (0.3 * d - front_start) / 13.0;
			float rear_step = (0.15 * d) / 10.0;
			//CarModel::optimizeDoubleParam("front", A_STEP, B_STEP, A_NO_STEPS, B_NO_STEPS, A_START, B_START);
			//CarModel::optimizeDoubleParam("back", AT_STEP, BT_STEP, AT_NO_STEPS, BT_NO_STEPS, AT_START, BT_START);
			CarModel::optimizeDoubleParam("front", front_step, front_step, A_NO_STEPS, B_NO_STEPS, front_start + 0.2, front_start);
			CarModel::optimizeDoubleParam("back", rear_step, rear_step, AT_NO_STEPS, BT_NO_STEPS, AT_START, BT_START);
			
			cost_p = cost;
			cost = CarModel::calcCost();
			if(verbose) cout << cost << endl;
			iteracje++;
		} while (abs(cost_p - cost) > 10 && iteracje <= 10);
		if(verbose) cout << "Iteracje: " << iteracje << endl << endl;
		iteracje = 0;
		if (cost < cost_min) {
			angle_opt = CarModel::getAngle();
			x_opt = CarModel::getX();
			y_opt = CarModel::getY();
			d_opt = CarModel::getD();
			s_opt = CarModel::getS();
			w_opt = CarModel::getW();
			a_opt = CarModel::getA();
			b_opt = CarModel::getB();
			at_opt = CarModel::getAt();
			bt_opt = CarModel::getBt();
			CarModel::setOptimalCost(cost);
			cost_min = cost;
		}
	}

	//if(CarModel::tryReversed(angle_opt) == 0) CarModel::setAllParameters(angle_opt+180, x_opt, y_opt, z_opt, s_opt, d_opt, w_opt, a_opt, b_opt, at_opt, bt_opt);
	//else
	CarModel::setAllParameters(angle_opt, x_opt, y_opt, z_opt, s_opt, d_opt, w_opt, a_opt, b_opt, at_opt, bt_opt);
	CarModel::buildModel2();
	if(verbose) cout << "Zakonczono" << endl;
}

void CarModel::drawMask(Mat& obraz, Scalar c)
{
	Mat region;
	Mat color(this->roi.height, this->roi.width, CV_8UC3);
	Mat maska = this->mask.clone();
	region = obraz(this->roi);
	color.setTo(c);
	cvtColor(maska, maska, cv::COLOR_GRAY2BGR);
	bitwise_not(maska, maska);
	add(maska, color, color);
	addWeighted(region, 0.6, color, 0.4, 0.0, region);
	obraz(this->roi) = region;
}

void CarModel::optimizeReduced(CarModel optimal, int last_angle)
{
	//CarModel::calculateStartXY();
	int angle_opt = last_angle;
	float x_opt = CarModel::x_start, y_opt = CarModel::y_start, z_opt = 0.0, s_opt = optimal.getS(), d_opt = optimal.getD(), w_opt = optimal.getW();
	float a_opt = optimal.getA(), b_opt = optimal.getB(), at_opt = optimal.getAt(), bt_opt = optimal.getBt();
	int cost = 50000, cost_min = 50000, cost_p = 50000, iteracje = 0;

	for (int iter = 0; iter < 1; iter++)
	{
		CarModel::setRandomParamsReduced(optimal);
		//CarModel::setAngle(int(last_angle - ((ANGLE_STEP/5)*ANGLE_NO_STEPS)/2.0));
		CarModel::setAngle(last_angle - 5);
		//CarModel::setAngle(rand()%360);
		do {
			//CarModel::optimizeParamInt("angle", ANGLE_STEP, ANGLE_NO_STEPS, CarModel::getAngle());
			CarModel::setAngle(last_angle - 5);
			CarModel::optimizeParamInt("angle", 1, 10, CarModel::getAngle());
			CarModel::setX(CarModel::getX() - (X_NO_STEPS - 1.0) / 20.0);
			CarModel::optimizeParam("x", X_STEP, X_NO_STEPS, CarModel::getX());
			CarModel::setY(CarModel::getY() - (Y_NO_STEPS - 1.0) / 20.0);
			CarModel::optimizeParam("y", Y_STEP, Y_NO_STEPS, CarModel::getY());

			cost_p = cost;
			cost = CarModel::calcCost();
			iteracje++;
		} while (abs(cost_p - cost) > 60 && iteracje <= 10);
		/*iteracje = 0;
		if (cost < cost_min) {
			angle_opt = CarModel::getAngle();
			x_opt = CarModel::getX();
			y_opt = CarModel::getY();
			CarModel::setOptimalCost(cost);
			cost_min = cost;
		}
	}
	CarModel::setAllParameters(angle_opt, x_opt, y_opt, z_opt, s_opt, d_opt, w_opt, a_opt, b_opt, at_opt, bt_opt);
	*/
	}
	CarModel::setOptimalCost(cost);
	CarModel::buildModel2();
}
