#include "conf.h"
#include <sstream>

class Trajectory
{
public:
	int readRawTrajectory();
	void readRawTrajectoryFile(string filename);

	Conf cf;

};

