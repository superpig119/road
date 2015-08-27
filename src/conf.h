#include <fstream>
#include <string>
#include <iostream>

using namespace std;

#ifndef CONF_H_
#define CONF_H_

class Conf
{
public:
	string roadFilePath;
	string rawTrajectoryFolder;
    int QuadTreeLevel;
	int readConf();
};
#endif
