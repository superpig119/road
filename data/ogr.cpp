#include <ogrsf_frmts.h>
#include <gdal.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <fstream>
#include <map>
#include <string>

using namespace std;

vector<string> split(const string &s, const string &seperator);

int main()
{
    GDALAllRegister();  //register all the format drivers
    cout << "GDAL All Registed!" << endl;

    GDALDataset *poDS;  //Data source
    poDS = (GDALDataset*) GDALOpenEx("./beijing/road/Nbeijing_point.shp", GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(poDS == NULL)
    {
        cout << "Open shp file failed!" << endl;
        exit(1);
    }
    cout << "Data source open success!" << endl;

    int layerNum = poDS->GetLayerCount();   //a dataset may have many layers
    cout << "Layer number:" << layerNum << endl;

    OGRLayer *poLayer;
    poLayer = poDS->GetLayerByName("Nbeijing_point");
    
    //feature is a geometry and a set of attributes
    OGRFeature *poFeature;
    poLayer->ResetReading();    //Start at the beginning of the layer
    cout << "Feature number:" << poLayer->GetFeatureCount() << endl;

    stringstream ss;
    map<string, pair<double, double> > mip;
    map<string, pair<double, double> >::iterator imip;
    
	ofstream ofile("beijingNode");

    string id;
	int	crossFlag;
	string stmp, stmp2;
	vector<string> vs;
	vector<string>::iterator ivs;
    while((poFeature = poLayer->GetNextFeature()) != NULL)
    {
        //Defn contains the definition of all the fields
        OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
//        for(iField = 0; iField < poFDefn->GetFieldCount(); iField++)
 //       {
//        OGRFieldDefn * poFieldDefn = poFDefn->GetFieldDefn(iField);
        id = poFeature->GetFieldAsString(1);

		ofile << id;

		crossFlag = poFeature->GetFieldAsInteger(4);
		if(crossFlag == 0)
		{
			ofile << "\t" << crossFlag << "\t" << 0 << "\t" << 0 << "\t" << 0;
		}
		else if(crossFlag == 1)
		{	
			ofile << "\t" << crossFlag << "\t" << 0 << "\t" << poFeature->GetFieldAsInteger(7) << "\t" << 0;
		}
		else if(crossFlag == 2)
		{
			stmp = poFeature->GetFieldAsString(6);
			vs = split(stmp, "|");
			ofile << "\t" << crossFlag << "\t" << vs.size();
			for(ivs = vs.begin(); ivs != vs.end(); ivs++)
				ofile << "\t" << *ivs;
			vs.clear();
			ofile << "\t" << poFeature->GetFieldAsInteger(7) << "\t" << 0;
		}
		else if(crossFlag == 3)
		{
			stmp = poFeature->GetFieldAsString(6);
			vs = split(stmp, "|");
			ofile << "\t" << crossFlag << "\t" << vs.size();
			for(ivs = vs.begin(); ivs != vs.end(); ivs++)
				ofile << "\t" << *ivs;
			vs.clear();
			
			ofile << "\t" << poFeature->GetFieldAsInteger(7);
			stmp = poFeature->GetFieldAsString(8);
			stmp2 = poFeature->GetFieldAsString(9);
			if(stmp2 != "")
				stmp += "|" + stmp2;
			vs = split(stmp, "|");
			ofile << "\t" << vs.size();
			for(ivs = vs.begin(); ivs != vs.end(); ivs++)
				ofile << "\t" << *ivs;
			vs.clear();
		}
	
		ofile << "\t" << poFeature->GetFieldAsString(11);

		stmp = poFeature->GetFieldAsString(12);
		vs = split(stmp, "|");
		ofile << "\t" << vs.size();
		for(ivs = vs.begin(); ivs != vs.end(); ivs++)
			ofile << "\t" << *ivs;
		vs.clear();
/*            if(poFieldDefn->GetType() == OFTInteger)
                cout << poFeature->GetFieldAsInteger(iField) << ", ";
            else if(poFieldDefn->GetType() == OFTInteger64)
                cout << poFeature->GetFieldAsInteger64(iField) << ", ";
            else if(poFieldDefn->GetType() == OFTReal)
                cout << setprecision(15) << poFeature->GetFieldAsDouble(iField) << ", ";
            else if(poFieldDefn->GetType() == OFTString)
                cout << poFeature->GetFieldAsString(iField) << ", ";
            else
                cout << poFeature->GetFieldAsString(iField) << ", ";*/
//        }

        OGRGeometry *poGeometry;
        poGeometry = poFeature->GetGeometryRef();
//        cout << "Geometry Type:" << poGeometry->getGeometryType() << endl;
        if(poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbMultiPoint)
        {
            OGRMultiPoint *poMultiPoint = (OGRMultiPoint*)poGeometry;
  //          cout << "Number in the MultiPoint:" << poMultiPoint->getNumGeometries() << endl;

            OGRGeometry *pG;
            pG = poMultiPoint->getGeometryRef(0);
            OGRPoint *pP = (OGRPoint*)pG;
			ofile << "\t" <<  pP->getY() << "\t" << pP->getX();
//            pP->flattenTo2D();
 //           cout <<  setprecision(15) << pP->getY() << ", " << pP->getX() << endl;
 //           mip[id] = make_pair(pP->getY(), pP->getX());
            
         }
        else
            cout << "No Point Geometry" << endl;
		ofile << endl;
        OGRFeature::DestroyFeature(poFeature);
    }

/*	cout << "Writing nodes" << endl;
    for(imip = mip.begin(); imip != mip.end(); imip++)
        ofile << setprecision(15) << (*imip).first << "\t" << (*imip).second.first << "\t" << (*imip).second.second << endl;*/

    GDALClose(poFeature);
    ofile.close();

    return 0;
}

vector<string> split(const string &s, const string &seperator)
{
	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;
		    
	while(i != s.size())
	{
		int flag = 0;
		while(i != s.size() && flag == 0)
		{
			flag = 1;
			for(string_size x = 0; x < seperator.size(); ++x)
			if(s[i] == seperator[x])
			{
				++i;
				flag = 0;
				break;
			}
		}
						    
		flag = 0;
		string_size j = i;
		while(j != s.size() && flag == 0)
		{
			for(string_size x = 0; x < seperator.size(); ++x)
				if(s[j] == seperator[x])
				{
					flag = 1;
					break;
				}
				if(flag == 0) 
					++j;
		}
									    
		if(i != j)
		{
			result.push_back(s.substr(i, j-i));
			i = j;														    
		}
										  
	}
			  
	return result;
}
