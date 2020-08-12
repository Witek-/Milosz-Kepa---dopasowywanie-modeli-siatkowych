#include "pch.h"
#include "MultiframeModel.hpp"

MultiframeModel::MultiframeModel()
{
	this->models = {};
	this->status = 1;
}

MultiframeModel::MultiframeModel(CarModel model, int frame, int id)
{
	this->models = {};
	this->status = 1;
	this->models[frame] = model;
	this->track_id = id;
}

int MultiframeModel::close()
{
	this->status = 0;
	return this->status;
}

int MultiframeModel::insert(CarModel model, int frame)
{
	if (this->status == 1) {
		this->models[frame] = model;
		return 0;
	}
	else {
		return -1;
	}
}

void MultiframeModel::optimizeModels()
{
	//klatki najblizej srodka
	vector<int> centralFrames = this->find5CentralFrames();
	//pelna optymalizacja dla trzech klatek najblizej srodka i usrednienie parametrow - kontener: this->optimal
	//this->calcAverageParams(this->models[centralFrames[0]], this->models[centralFrames[1]], this->models[centralFrames[2]]);
	this->models[centralFrames[0]].optimize();
	this->models[centralFrames[1]].optimize();
	this->models[centralFrames[2]].optimize();
	this->models[centralFrames[3]].optimize();
	this->models[centralFrames[4]].optimize();
	this->calc5BestParams(this->models[centralFrames[0]], this->models[centralFrames[1]], this->models[centralFrames[2]], this->models[centralFrames[3]], this->models[centralFrames[4]]);
	//teraz juz tylko angle, x, y
	int minframe = this->findMinFrame();
		//Inicjalizacja Filtru Kalmana
	//this->models[minframe].optimize();
	//this->models[minframe].optimize();
/*	int last_angle = this->optimal.getAngle();
	KalmanFilter filtr = initFilter(minframe);
	Mat_<float> pomiary(3, 1);
	Mat state;

	//optymalizacja do przodu - od poczatku
	for (int k = this->findMinFrame(); k <= this->findMaxFrame(); k++) {
		if (this->models.count(k) == 0) continue;
		filtr.predict();
		this->models[k].optimizeReduced(this->optimal, last_angle);
		pomiary.at<float>(0) = this->models[k].getX();
		pomiary.at<float>(1) = this->models[k].getY();
		pomiary.at<float>(2) = float(this->models[k].getAngle());
		Mat estimated = filtr.correct(pomiary);
		this->models[k].setX(estimated.at<float>(0));
		this->models[k].setY(estimated.at<float>(1));
		this->models[k].setAngle(int(estimated.at<float>(2)));
		this->models[k].buildModel2();
		last_angle = int(estimated.at<float>(2));
	}
*/

	//Inicjalizacja Filtru Kalmana
	KalmanFilter filtr = initFilter(centralFrames[0]);
	Mat_<float> pomiary(3, 1);
	int last_angle = this->optimal.getAngle();
	Mat state;

	//optymalizacja do przodu
	for (int k = centralFrames[0]; k <= this->findMaxFrame(); k++) {
		if (this->models.count(k) == 0) continue;
		filtr.predict();
		this->models[k].optimizeReduced(this->optimal, last_angle);
		pomiary.at<float>(0) = this->models[k].getX();
		pomiary.at<float>(1) = this->models[k].getY();
		pomiary.at<float>(2) = float(this->models[k].getAngle());
		Mat estimated = filtr.correct(pomiary);
		this->models[k].setX(estimated.at<float>(0));
		this->models[k].setY(estimated.at<float>(1));
		this->models[k].setAngle(int(estimated.at<float>(2)));
		this->models[k].buildModel2();
		last_angle = int(estimated.at<float>(2));
		if (k == centralFrames[0] + 2) state = filtr.statePost;
	}

	KalmanFilter filtr2 = initFilter(centralFrames[0]);
	filtr2.statePre = state;
	Mat_<float> pomiary2(3, 1);
	last_angle = this->optimal.getAngle();

	//optymalizacja do tylu
	for (int k = centralFrames[0] + 2; k >= this->findMinFrame(); k--) {
		if (this->models.count(k) == 0) continue;
		filtr2.predict();
		this->models[k].optimizeReduced(this->optimal, last_angle);
		pomiary2.at<float>(0) = this->models[k].getX();
		pomiary2.at<float>(1) = this->models[k].getY();
		pomiary2.at<float>(2) = float(this->models[k].getAngle());
		Mat estimated = filtr2.correct(pomiary2);
		this->models[k].setX(estimated.at<float>(0));
		this->models[k].setY(estimated.at<float>(1));
		this->models[k].setAngle(int(estimated.at<float>(2)));
		this->models[k].buildModel2();
		last_angle = int(estimated.at<float>(2));
		if (k == this->findMinFrame()) state = filtr2.statePost;
	}
	
	filtr = initFilter(this->findMinFrame());
	filtr.statePre = state;

	for (int k = this->findMinFrame(); k <= this->findMaxFrame(); k++) {
		if (this->models.count(k) == 0) continue;
		filtr.predict();
		this->models[k].optimizeReduced(this->optimal, last_angle);
		pomiary.at<float>(0) = this->models[k].getX();
		pomiary.at<float>(1) = this->models[k].getY();
		pomiary.at<float>(2) = float(this->models[k].getAngle());
		Mat estimated = filtr.correct(pomiary);
		this->models[k].setX(estimated.at<float>(0));
		this->models[k].setY(estimated.at<float>(1));
		this->models[k].setAngle(int(estimated.at<float>(2)));
		this->models[k].buildModel2();
		last_angle = int(estimated.at<float>(2));
	}

	//for (auto& model : this->models) {
	//	model.second.optimizeReduced(this->optimal);
	//}
}

KalmanFilter MultiframeModel::initFilter(int startframe)
{
	KalmanFilter filtr(6, 3, 0);
	filtr.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
												   0, 1, 0, 0, 1, 0,
												   0, 0, 1, 0, 0, 1,
												   0, 0, 0, 1, 0, 0,
												   0, 0, 0, 0, 1, 0,
												   0, 0, 0, 0, 0, 1);
	setIdentity(filtr.measurementMatrix);
	//setIdentity(filtr.processNoiseCov, Scalar::all(1e-2));        //Kowariancja Q 
	filtr.processNoiseCov = (Mat_<float>(6, 6) << 1e-3, 0, 0, 0, 0, 0,
												  0, 1e-3, 0, 0, 0, 0,
												  0, 0, 1e-1, 0, 0, 0,
												  0, 0, 0, 1e-3, 0, 0,
												  0, 0, 0, 0, 1e-3, 0,
												  0, 0, 0, 0, 0, 1e-3);
	//setIdentity(filtr.measurementNoiseCov, Scalar::all(0.1));    //Kowariancja R
	filtr.measurementNoiseCov = (Mat_<float>(3, 3) << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1);
	setIdentity(filtr.errorCovPost, Scalar::all(1));

	filtr.statePre.at<float>(0) = this->optimal.getX();        //Po³o¿enie x 
	filtr.statePre.at<float>(1) = this->optimal.getY();        //Po³o¿enie y 
	filtr.statePre.at<float>(2) = float(this->optimal.getAngle());        //Po³o¿enie theta
	filtr.statePre.at<float>(3) = float(1.0);        //Prêdkosc x 
	filtr.statePre.at<float>(4) = float(1.0);        //Prêdkosc y 
	filtr.statePre.at<float>(5) = float(0.0);        //Prêdkosc theta

	return filtr;
}

//filtr.statePre.at<float>(3) = abs(this->models[centralFrames[0]].getX() - this->models[centralFrames[1]].getX());        //Prêdkosc x 
//filtr.statePre.at<float>(4) = abs(this->models[centralFrames[0]].getY() - this->models[centralFrames[1]].getY()) / 2.0;        //Prêdkosc y 
//filtr.statePre.at<float>(5) = float(abs(this->models[centralFrames[0]].getAngle() - this->models[centralFrames[1]].getAngle()));        //Prêdkosc theta
vector<int> MultiframeModel::findCentralFrames()
{
	int firstFrame = -1, secondFrame = -1, thirdFrame = -1;
	double min_dist1 = 10000.0, min_dist2 = 10000.0, min_dist3 = 10000.0;
	for (auto model : this->models) {
		Point p = Point((model.second.getRoi().tl() + model.second.getRoi().br()) * 0.5);
		double dist = this->calcDistanceFromCenter(p);
		if (dist < min_dist1) {
			min_dist3 = min_dist2;
			min_dist2 = min_dist1;
			min_dist1 = dist;
			thirdFrame = secondFrame;
			secondFrame = firstFrame;
			firstFrame = model.first;
		}
		else if (dist < min_dist2) {
			min_dist3 = min_dist2;
			min_dist2 = dist;
			thirdFrame = secondFrame;
			secondFrame = model.first;
		}
		else if (dist < min_dist3) {
			min_dist3 = dist;
			thirdFrame = model.first;
		}
	}
	vector<int> ret = { firstFrame, secondFrame, thirdFrame };
	return ret;
}

vector<int> MultiframeModel::find5CentralFrames()
{
	int firstFrame = -1, secondFrame = -1, thirdFrame = -1, fourthFrame = -1, fifthFrame = -1;
	double min_dist1 = 100000.0, min_dist2 = 100000.0, min_dist3 = 100000.0, min_dist4 = 100000.0, min_dist5 = 100000.0;
	for (auto model : this->models) {
		Point p = Point((model.second.getRoi().tl() + model.second.getRoi().br()) * 0.5);
		double dist = this->calcDistanceFromCenter(p);
		if (dist < min_dist1) {
			min_dist5 = min_dist4;
			min_dist4 = min_dist3;
			min_dist3 = min_dist2;
			min_dist2 = min_dist1;
			min_dist1 = dist;
			fifthFrame = fourthFrame;
			fourthFrame = thirdFrame;
			thirdFrame = secondFrame;
			secondFrame = firstFrame;
			firstFrame = model.first;
		}
		else if (dist < min_dist2) {
			min_dist5 = min_dist4;
			min_dist4 = min_dist3;
			min_dist3 = min_dist2;
			min_dist2 = dist;
			fifthFrame = fourthFrame;
			fourthFrame = thirdFrame;
			thirdFrame = secondFrame;
			secondFrame = model.first;
		}
		else if (dist < min_dist3) {
			min_dist5 = min_dist4;
			min_dist4 = min_dist3;
			min_dist3 = dist;
			fifthFrame = fourthFrame;
			fourthFrame = thirdFrame;
			thirdFrame = model.first;
		}
		else if (dist < min_dist4) {
			min_dist5 = min_dist4;
			min_dist4 = dist;
			fifthFrame = fourthFrame;
			fourthFrame = model.first;
		}
		else if (dist < min_dist5) {
			min_dist5 = dist;
			fifthFrame = model.first;
		}
	}
	vector<int> ret = { firstFrame, secondFrame, thirdFrame, fourthFrame, fifthFrame };
	return ret;
}

int MultiframeModel::findMinFrame()
{
	int minframe = 500000;
	for (auto& model : this->models) {
		if (model.first < minframe) minframe = model.first;
	}
	return minframe;
}

int MultiframeModel::findMaxFrame()
{
	int maxframe = 0;
	for (auto& model : this->models) {
		if (model.first > maxframe) maxframe = model.first;
	}
	return maxframe;
}

double MultiframeModel::calcDistanceFromCenter(Point p) {
	Point center(960, 540);
	return sqrt(double(pow(p.x - center.x, 2) + pow(p.y - center.y, 2)));
}

void MultiframeModel::calcAverageParams(CarModel a, CarModel b, CarModel c)
{
	a.optimize(0);
	b.optimize(0);
	c.optimize(0);
	this->optimal.setS(float((a.getS() + b.getS() + c.getS()) / float(3.0)));
	this->optimal.setD(float((a.getD() + b.getD() + c.getD()) / float(3.0)));
	this->optimal.setW(float((a.getW() + b.getW() + c.getW()) / float(3.0)));
	this->optimal.setA(float((a.getA() + b.getA() + c.getA()) / float(3.0)));
	this->optimal.setB(float((a.getB() + b.getB() + c.getB()) / float(3.0)));
	this->optimal.setAt(float((a.getAt() + b.getAt() + c.getAt()) / float(3.0)));
	this->optimal.setBt(float((a.getBt() + b.getBt() + c.getBt()) / float(3.0)));
}

void MultiframeModel::calcBestParams(CarModel a, CarModel b, CarModel c)
{
	double cost_a = double(a.getOptimalCost()) / double(a.getMaskArea());
	double cost_b = double(b.getOptimalCost()) / double(b.getMaskArea());
	double cost_c = double(c.getOptimalCost()) / double(c.getMaskArea());
	if (cost_a <= cost_b && cost_a <= cost_c) this->optimal = a;
	else if (cost_b < cost_a && cost_b < cost_c) this->optimal = b;
	else if (cost_c < cost_a && cost_c < cost_b) this->optimal = c;
}

void MultiframeModel::calc5BestParams(CarModel a, CarModel b, CarModel c, CarModel d, CarModel e)
{
	double cost_a = double(a.getOptimalCost()) / double(a.getMaskArea());
	double cost_b = double(b.getOptimalCost()) / double(b.getMaskArea());
	double cost_c = double(c.getOptimalCost()) / double(c.getMaskArea());
	double cost_d = double(d.getOptimalCost()) / double(d.getMaskArea());
	double cost_e = double(e.getOptimalCost()) / double(e.getMaskArea());

	vector<double> costs = { cost_a, cost_b, cost_c, cost_d, cost_e };
	vector<double>::iterator it = min_element(costs.begin(), costs.end());
	switch (distance(costs.begin(),it)) {
	case 0:
		this->optimal = a;
		break;
	case 1:
		this->optimal = b;
		break;
	case 2:
		this->optimal = c;
		break;
	case 3:
		this->optimal = d;
		break;
	case 4:
		this->optimal = e;
		break;
	default:
		break;
	}

}

/*
CarModel MultiframeModel::getModelByFrame(int frame)
{
	vector<int>::iterator it = std::find(this->frames.begin(), this->frames.end(), frame);
	int index = std::distance(this->frames.begin(), it);
	return this->models[index];
}

int MultiframeModel::getModelIndexByFrame(int frame)
{
	vector<int>::iterator it = std::find(this->frames.begin(), this->frames.end(), frame);
	int index = std::distance(this->frames.begin(), it);
	return index;
}
*/