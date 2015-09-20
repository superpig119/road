#include <fstream>
#include <string>
#include <iostream>

using namespace std;

#ifndef CONF_H_
#define CONF_H_

class Conf
{
public:
	string city;
	string dataPath;
	string nodeFilePath;
	string edgeFilePath;
	string trajectoryFilePath;
	string speedFilePath;
	string avgSpeedFilePath;
	string isoRoadFilePath;
	string isoNodeFilePath;
	int readConf();
	int	h;
	int	m;
};
#endif
