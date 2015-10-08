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
	string nodeMapFilePath;
	string datapath;
	string splitType;//universal, slot, continous, adpative
	string MS;	//Missing value:temporal,spatial,ST, correlation
	int readConf();
	int	h;
	int	m;
	int testNum;
};
#endif
