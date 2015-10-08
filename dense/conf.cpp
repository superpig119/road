#include "conf.h"

int Conf::readConf()
{
	ifstream iconf("configure");
	if(!iconf)
	{
		cout << "Cannot open configure file" << endl;
		return 1;
	}
	iconf >> city;
	iconf.close();

	ifstream ifile(("configure_" + city).c_str());
	if(!ifile)
	{
		cout << "Cannot open " + city + " configure file" << endl;
		return 1;
	}
	string s;
	while(ifile >> s)
	{
		if(s == "nodeFile")
			ifile >> nodeFilePath;
		else if(s == "edgeFile")
			ifile >> edgeFilePath;
		else if(s == "trajectoryFile")
			ifile >> trajectoryFilePath;
		else if(s == "speedFile")
			ifile >> speedFilePath;
		else if(s == "interval")
			ifile >> h >> m;
		else if(s == "dataPath")
			ifile >> dataPath;
		else if(s == "avgSpeedFile")
			ifile >> avgSpeedFilePath;
		else if(s == "isoRoadFile")
			ifile >> isoRoadFilePath;
		else if(s == "isoNodeFile")
			ifile >> isoNodeFilePath;
		else if(s == "testNum")
			ifile >> testNum;
		else if(s == "nodeMapFile")
			ifile >> nodeMapFilePath;
		else if(s == "datapath")
			ifile >> datapath;
		else if(s == "splitType")
			ifile >> splitType;
		else if(s == "MS")
			ifile >> MS;

	}

	return 0;
}
	
