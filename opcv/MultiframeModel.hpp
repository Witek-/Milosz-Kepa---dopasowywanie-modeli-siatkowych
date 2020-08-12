#pragma once

#include <algorithm>
#include "CarModel.hpp"

using namespace std;

class MultiframeModel
{
private:
	int status; //0 - closed, 1 - open
	CarModel optimal;
	void calcAverageParams(CarModel, CarModel, CarModel);
	void calcBestParams(CarModel, CarModel, CarModel);
	void calc5BestParams(CarModel, CarModel, CarModel, CarModel, CarModel);
	vector<int> findCentralFrames();
	vector<int> find5CentralFrames();
	int findMinFrame();
	int findMaxFrame();
	double calcDistanceFromCenter(Point);
	KalmanFilter initFilter(int);

public:

	// map<nr klatki, model>
	map<int, CarModel> models;
	int track_id = -1;
	
	MultiframeModel();
	MultiframeModel(CarModel model, int frame, int id = -1);

	int close();
	int insert(CarModel model, int frame);
	void optimizeModels();

	int getStatus() { return this->status; }
	int getSize() { return this->models.size(); }

	//CarModel getModelByFrame(int frame);
	//int getModelIndexByFrame(int frame);
};

