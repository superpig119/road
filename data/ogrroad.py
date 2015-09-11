#!/usr/bin/python

from osgeo import ogr
from osgeo import gdal
import os

shapefile = "Rbeijing_polyline.shp"
ds = gdal.OpenEx(shapefile, gdal.OF_VECTOR);

if ds is None:
	print "Open Failed.\n"
	sys.exit(1)

layer = ds.GetLayer("Rbeijing_polyline");
layer.ResetReading()

featureCount = layer.GetFeatureCount()
print "Number of features is %d."  % (featureCount)

outfile = open("road", 'w')

for feature in layer:
	feature_def = layer.GetLayerDefn()
	outline = ""
	outline += feature.GetFieldAsString(1);			#ID

	outline += "\t" + feature.GetFieldAsString(5);	#Direction
#	outline += "\t" + feature.GetFieldAsString(12);	
	
	length = feature.GetFieldAsDouble(12) * 1000	#Length
	outline += "\t" + str(int(length))

	outline += "\t" + feature.GetFieldAsString(9);
	outline += "\t" + feature.GetFieldAsString(10);
	

	geom = feature.GetGeometryRef()
		
	if geom is not None and geom.GetGeometryType() == ogr.wkbLineString:
		for i in range(0, geom.GetPointCount()):
			pt = geom.GetPoint(i)
#			print "%i). POINT(%f %f)" % (i, pt[1], pt[0])
			outline += "\t" + str(pt[1]) + "," + str(pt[0])
	outline += "\n"
	outfile.write(outline)

outfile.close()
"""	for i in range(feature_def.GetFieldCount()):
		field_defn = feature_def.GetFieldDefn(i)
	    # Tests below can be simplified with just :
	    # print feat.GetField(i)
		if field_defn.GetType() == ogr.OFTInteger or field_defn.GetType() == ogr.OFTInteger64:
			print "%d" % feature.GetFieldAsInteger64(i)
		elif field_defn.GetType() == ogr.OFTReal:
			print "%.6f" % feature.GetFieldAsDouble(i)
		elif field_defn.GetType() == ogr.OFTString:
			print "%s" % feature.GetFieldAsString(i)
		else:
			print "%s" % feature.GetFieldAsString(i)
"""
