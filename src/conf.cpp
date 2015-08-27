#include "conf.h"

int Conf::readConf()
{
	fstream ifile("configure");
	if(!ifile)
	{
		cout << "Cannot open configure file" << endl;
		return 1;
	}
	string s;
	while(ifile >> s)
	{
		if(s == "datapath")
			ifile >> roadFilePath;
		else if(s == "rawTrajectory")
			ifile >> rawTrajectoryFolder;
        else if(s == "QuadTreeLevel")
            ifile >> QuadTreeLevel;
	}

	return 0;
}
