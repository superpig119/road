#include "conf.h"
#include <sstream>
#include <vector>
#include <time.h>

typedef struct TRAJECTORYUNIT
{
	time_t t;
	double x, y;
}trajectoryUnit;

typedef struct TAXITRAJECTORY
{
	time_t sTime;
	time_t eTime;
	double distance;
	vector<trajectoryUnit> vTU;
}taxiTrajectory;

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

