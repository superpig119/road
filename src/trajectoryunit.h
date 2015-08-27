#ifndef trajectoryuni_h
#define trajectoryuni_h

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

#endif
