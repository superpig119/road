#ifndef trajectory_h
#define trajectory_h

#include "conf.h"
#include "trajectoryunit.h"
#include <sstream>

class Trajectory
{
public:
	int readRawTrajectory();
	void readRawTrajectoryFile(string filename);
	struct tm wrapTime(ifstream &ifile);
	void testTrajectory();

	Conf cf;
	vector<taxiTrajectory> vTrajectory;
};

#endif
