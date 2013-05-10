/********************************************************************************************************************** 
 * ENDURANCE DeltaT Processing Tool
 *---------------------------------------------------------------------------------------------------------------------
 * Author: 
 *	Alessandro Febretti							Electronic Visualization Laboratory, University of Illinois at Chicago
 * Contact & Web:
 *  febret@gmail.com							http://febretpository.net
 *---------------------------------------------------------------------------------------------------------------------
 * This tool has been built as part of the ENDURANCE Project (http://www.evl.uic.edu/endurance/).
 * ENDURANCE is supported by the NASA ASTEP program under Grant NNX07AM88G and by the NSF USAP.
 *---------------------------------------------------------------------------------------------------------------------
 * Copyright (c) 2010, Electronic Visualization Laboratory, University of Illinois at Chicago
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
 * following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
 * disclaimer. Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
 * and the following disclaimer in the documentation and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************************************************/ 
#define _CRT_SECURE_NO_WARNINGS

#include <limits>
#include <sstream>
#include <fstream>
#include <float.h>
#include <math.h>
#ifndef WIN32
#include <cmath>
using std::isnan;
#define _isnan isnan
#define _finite finite
#endif


// VCG lib stuff (used for normal computation)
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/trimesh/base.h>
#include <vcg/complex/trimesh/update/normal.h>
#include <vcg/space/normal_extrapolation.h>

#include "mb_fbt.h"

#include "PointCloud.h"
#include "Utils.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloud::PointCloud():
	myHasNormals(false), myHasBeamInfo(false), myHasPoseInfo(false), myHasQuality(false),
	myNormalNeighbors(10),
	myFile(NULL),
	myReadLength(0),
	myReadDone(false)
{
	resetBounds();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointCloud::~PointCloud()
{
	myData.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::read(const std::string& filename, FileFormat::Enum format)
{
	openInputFile(filename, format);
	while(!endOfFile())
	{
		int bufferSize = 128000000;
		readNext(bufferSize);
	}
	closeFile();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PointCloud::endOfFile()
{
	return feof(myFile) || (myReadLength != 0 && (myFileOffset - myReadOffset) >= myReadLength) || myReadDone;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::setReadOffset(long int offset)
{
	char lineData[1024];
	// Adjust the offset to read from the beginning of a line.
	if(offset > 512)
	{
		fseek(myFile, offset - 512, SEEK_SET);
		int len = fread(lineData, 1, 512, myFile);
		lineData[len] = '\0';
		char* lastnl = strrchr(lineData, '\n');
		if(lastnl != NULL)
		{
			int offdiff = lastnl - lineData;
			offset = offset - len + offdiff;
		}
	}
	
	myFileOffset = offset;
	myReadOffset = offset;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::setReadLength(long int length)
{
	myReadLength = length;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long int PointCloud::getSizeInBytes()
{
	fseek(myFile, 0, SEEK_END);
	return ftell(myFile);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::readNext(int bufferSize)
{
	DTT_ASSERT(!myIsFileOutput);

	char* data = new char[bufferSize + 1];
	fseek(myFile, myFileOffset, SEEK_SET);
	
	// Adjust the read buffer size so we don't go past the read length. Only do this if read length is different than 0.
	if(myFileOffset + bufferSize - myReadOffset >= myReadLength) 
	{
		bufferSize = myReadLength - (myFileOffset - myReadOffset);
		myReadDone = true;
	}
	
	int len = fread(data, 1, bufferSize, myFile);
	data[len] = '\0';

	if(len != 0)
	{
		double values[64];

		char* line = data;
		char* nl = strchr(line, '\n');

		int i = 0;
		while(nl != NULL)
		{
			*nl = '\0';

			int size = Utils::extractValues(line, values);

			SonarPoint pt;

			if(myFormat == FileFormat::XYZPointsWithNormals)
			{
				pt.position = vmml::vec3d(values[0], values[1], values[2]);
				pt.normal = vmml::vec3d(values[3], values[4], values[5]);

				myHasNormals = true;
			}
			else if(myFormat == FileFormat::CSVPoints)
			{
				if(size >= 3)
				{
					pt.position = vmml::vec3d(values[0], values[1], values[2]);
				}
				else
				{
					pt.position = vmml::vec3d(values[0], values[1], 0);
				}
			}
			else if(myFormat == FileFormat::CSVPoints2)
			{
				pt.position = vmml::vec3d(values[1], values[2], values[3]);
			}
			else if(myFormat == FileFormat::CSVPointsPose && size == 9)
			{
				pt.position = vmml::vec3d(values[0], values[1], values[2]);
				pt.sourcePosition = vmml::vec3d(values[3], values[4], values[5]);
				pt.sourceOrientation = vmml::vec3d(values[6], values[7], values[8]);
			}
			else
			{
				fprintf(stderr, "Wrong file format specified! Stopped reading file at line %d\n", i);
				delete data;
				return;	
			}

			myData.push_back(pt);
			updateBounds(pt);

			// fetch next line.
			line = nl + 1;
			nl = strchr(line, '\n');

			i++;
		}

		// Set the file offset to be the start if the next unparsed line.
		// ON WINDOWS the + i at the end is needed because each /n character in the text stream is actually two
		// bytes (/n and /r), so we need to add the number of lines to correctly compute the file
		// offset for the next buffered read.
#ifdef WIN32
		myFileOffset += (line - data) + i;
#else
		myFileOffset += (line - data);
#endif
	}

	delete data;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::writeItem(const SonarPoint& pt)
{
	if(myFormat == FileFormat::CSVPoints)
	{
		fprintf(myFile, "%f, %f, %f\n", 
			pt.position[0], pt.position[1], pt.position[2]);
	}
	else if(myFormat == FileFormat::CSVPointsPose)
	{
		const vmml::vec3d& pos = pt.position;
		const vmml::vec3d& ppos = pt.sourcePosition;
		const vmml::vec3d& por = pt.sourceOrientation;

		fprintf(myFile, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
			pos[0], pos[1], pos[2],
			ppos[0], ppos[1], ppos[2],
			por[0], por[1], por[2]);
	}
	else if(myFormat == FileFormat::CSVPointsWithBeamInfo)
	{
		const vmml::vec3d& pos = pt.position;

		fprintf(myFile, "%f, %f, %f, %f, %f, %d\n", 
			pos[0], pos[1], pos[2], pt.angle, pt.range, pt.diveTag);
	}
	else if(myFormat == FileFormat::CSVPointsWithBeamPoseInfo)
	{
		const vmml::vec3d& pos = pt.position;
		const vmml::vec3d& ppos = pt.sourcePosition;
		const vmml::vec3d& por = pt.sourceOrientation;

		fprintf(myFile, "%f, %f, %f, %f, %f, %d, %f, %f, %f, %f, %f, %f, %d\n", 
			pos[0], pos[1], pos[2], pt.angle, pt.range, 
			(int)pt.t,
			ppos[0], ppos[1], ppos[2],
			por[0], por[1], por[2],
			pt.diveTag);
	}
	else if(myFormat == FileFormat::XYZPointsWithNormals)
	{
		//DTT_ASSERT(myHasNormals);
		// If the dataset has normal data otput it. Otherwise, just output a default up (positive Y) normal for each point.
		if(myHasNormals && pt.normal != vmml::vec3f::ZERO)
		{
			const vmml::vec3f& pos = pt.position;
			const vmml::vec3f& normal = pt.normal;
			fprintf(myFile, "%f %f %f %f %f %f\n", 
				pos[0], pos[1], pos[2],
				normal[0], normal[1], normal[2]);
		}
		else
		{
			const vmml::vec3f& pos = pt.position;
			fprintf(myFile, "%f %f %f 0 1 0\n", pos[0], pos[1], pos[2]);
		}
	}
	//else if(myFormat == FileFormat::XYZBinaryPointsWithNormals)
	//{
	//	const vmml::vec3f& pos = pt.position;
	//	vmml::vec3f normal;
	//	if(myHasNormals)
	//	{
	//		normal = pt.normal;
	//	}
	//	else
	//	{
	//		normal = pt.position - pt.sourcePosition;
	//		normal.normalize();
	//	}
	//	fwrite(pos.begin(), sizeof(float), 3, myFile);
	//	fwrite(normal.begin(), sizeof(float), 3, myFile);
	//}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::openOutputFile(const std::string& filename, FileFormat::Enum format, bool append)
{
	DTT_ASSERT((
		format == FileFormat::CSVPoints || 
		format == FileFormat::CSVPointsPose || 
		format == FileFormat::CSVPointsWithBeamInfo || 
		format == FileFormat::CSVPointsWithBeamPoseInfo || 
		//format == FileFormat::XYZBinaryPointsWithNormals || 
		format == FileFormat::XYZPointsWithNormals));
	
	myIsFileOutput = true;
	myFileOffset = 0;
	myFileEof = false;
	
	myFile = fopen(filename.c_str(), append ? "a" : "w");
	myFormat = format;
}

void PointCloud::deleteOutputFile(const std::string& filename)
{
	remove(filename.c_str());
}

void PointCloud::mergeOutputFiles(const std::string& finalOutputFile, const std::string& singleOutputFile)
{
	char temp[1024];
	FILE* finalOutput = fopen(finalOutputFile.c_str(), "a");
	FILE* singleOutput = fopen(singleOutputFile.c_str(), "r");
	
	while(fgets(temp, sizeof temp, singleOutput)!=NULL)
	{
		fprintf(finalOutput, temp);
	}

	fclose(singleOutput);
	fclose(finalOutput);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::openInputFile(const std::string& filename, FileFormat::Enum format)
{
	DTT_ASSERT(
		format == FileFormat::XYZPointsWithNormals || 
		//format == FileFormat::XYZBinaryPointsWithNormals || 
		format == FileFormat::CSVPoints || 
		format == FileFormat::CSVPoints2 || 
		format == FileFormat::CSVPointsPose);

	myIsFileOutput = false;
	myFileOffset = 0;
	myFileEof = false;

	myFile = fopen(filename.c_str(), "rb");
	myFormat = format;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::closeFile()
{
	if(myFile != NULL)
	{
		fflush(myFile);
		fclose(myFile);
	}
	
	myFile = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::flush()
{
	for(unsigned int i = 0; i < myData.size(); i++)
	{
		SonarPoint& pt = myData[i];
		writeItem(pt);
	}
	myData.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::write(const std::string& filename, FileFormat::Enum format, bool append)
{
	openOutputFile(filename, format, append);
	flush();
	closeFile();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::updateBounds()
{
	resetBounds();
	for(unsigned int i = 0; i < myData.size(); i++) updateBounds(myData[i]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::resetBounds()
{
	float dmax = std::numeric_limits<float>::max();
	float dmin = -std::numeric_limits<float>::max();
	myMaxBounds = vmml::vec3d(dmin, dmin, dmin);
	myMinBounds = vmml::vec3d(dmax, dmax, dmax);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::updateBounds(const SonarPoint& pt)
{
	for(int j = 0; j < 3; j++)
	{
		if(pt.position[j] < myMinBounds[j]) myMinBounds[j] = pt.position[j];
		if(pt.position[j] > myMaxBounds[j]) myMaxBounds[j] = pt.position[j];

		//if(pt.sourcePosition[j] < myMinBounds[j]) myMinBounds[j] = pt.sourcePosition[j];
		//if(pt.sourcePosition[j] > myMaxBounds[j]) myMaxBounds[j] = pt.sourcePosition[j];
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::addPing(const RangeDataPing& ping, const RaytracerConfiguration& cfg, PingStats& stats)
{
	vmml::vec4f beamPoints[MAX_RANGES];

    // Modify the point data source
	double thetai = cfg.beamAngleStart;
	double thetaf = cfg.beamAngleEnd;
	double dtheta = (cfg.beamAngleEnd - cfg.beamAngleStart) / cfg.beamCount;
	double theta = thetai;

	int numBeams = 0;

	stats.totalPoints += cfg.beamCount;

	// Build the beam point cloud in sensor space.
    for (unsigned int i = 0; i < cfg.beamCount; i++)
    {
        double r = ping.ranges[i];

		// Filter out invalid beams.
		if(theta > cfg.beamFilterAngleMin &&
			theta < cfg.beamFilterAngleMax &&
			(i % cfg.beamDecimation == 0) &&
			(r > cfg.rangeThreshold))
		{

			stats.passedPoints++;

			double xx;
			double zz;
			bool ok = true;

			double takeoffAngle = theta + cfg.beamAngleOffset;

			if(cfg.svp.model != NULL)
			{
				int verbose = 5;
				int error = 0;
				double d = ping.position[2];
				double ttime;
				int ray_stat;
				double atakeoffAngle = takeoffAngle >= 0 ? takeoffAngle : -takeoffAngle;
				int status = mb_rt(verbose, cfg.svp.model, d, 
					atakeoffAngle, r / cfg.sensorSoundVelocity,
					MB_SSV_CORRECT, cfg.sensorSoundVelocity, 0, 
					0, NULL, NULL, NULL, 
					&xx, &zz, 
					&ttime, &ray_stat, &error);

				if(status == MB_FAILURE || _isnan(xx) || _isnan(zz) || !_finite(xx) || !_finite(zz)) 
				{
					ok = false;
					stats.raytraceFailures++;
				}

				if(takeoffAngle < 0) xx = -xx;
				if(takeoffAngle > 90) zz = - zz;
			}
			else
			{
				// If no sound velocity profile is specified, perform a simple point projection along a straight
				// line instead of raytracing				
				xx = r * sin(DEG_TO_RAD(takeoffAngle));
				zz = r * cos(DEG_TO_RAD(takeoffAngle));
			}

			if(ok)
			{
				beamPoints[numBeams] = vmml::vec4f((float)takeoffAngle, (float)xx, (float)zz, (float)r);
				numBeams++;
			}

		}
		theta += dtheta;
    }

	stats.addedPoints += numBeams;

    //if (numBeams) printf("%12.3f %12.3f %6.3f %5d\n", ping.position[0], ping.position[1], ping.position[2], numBeams);

    // Update the body to world matrix
	vmml::mat4d bodyToWorld = vmml::mat4d::IDENTITY;

	bodyToWorld.rotate_x(ping.orientation[0]);
	bodyToWorld.rotate_y(ping.orientation[1]);
	bodyToWorld.rotate_z(ping.orientation[2]);
	bodyToWorld.set_translation(ping.position[0], ping.position[1], ping.position[2]);

	// The number of elements before we start inserting this ping points.
	int prevDataSize = myData.size();

	for (int i = 0; i < numBeams; i++)
	{
		float theta = beamPoints[i][0];
		float range = beamPoints[i][3];
		beamPoints[i][0] = 0.0; // Component 0 was used to store beam angle. restore it to its original value.
		beamPoints[i][3] = 1.0; // Component 3 was used to store beam range. restore it to its original value (1.0).

		// Transform the point from sensor to world coordinates
		vmml::vec4f pointB = cfg.sensorTransform * beamPoints[i];
		vmml::vec4d pointW = cfg.worldTransform * bodyToWorld * pointB;

		SonarPoint pt;
		// Write point position converting from homogeneous coordinates.
		pt.position = pointW / pointW[3];

		pt.t = ping.t;
		//pt.diveTag = ping.diveTag;
		pt.range = range;
		pt.angle = theta;
		pt.sourcePosition = ping.position;
		pt.sourceOrientation = ping.orientation;

		if(!_finite(pt.position[0]) || !_finite(pt.position[1]) || !_finite(pt.position[2]))
		{
			abort();
		}

		myData.push_back(pt);
		updateBounds(pt);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::appendPoints(PointCloud& points)
{
	myData.insert(myData.end(), points.getPoints().begin(), points.getPoints().end());
	vmml::vec3d minb = points.getMinBounds();
	vmml::vec3d maxb = points.getMaxBounds();

	// Update bounds
	for(int j = 0; j < 3; j++)
	{
		if(minb[j] < myMinBounds[j]) myMinBounds[j] = minb[j];
		if(minb[j] > myMaxBounds[j]) myMaxBounds[j] = minb[j];
		if(maxb[j] < myMinBounds[j]) myMinBounds[j] = maxb[j];
		if(maxb[j] > myMaxBounds[j]) myMaxBounds[j] = maxb[j];
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloud::addPoint(const SonarPoint& point)
{
	myData.push_back(point);
	updateBounds(point);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Normal estimation using VCGlib.
class MyVertex;
class MyUsedTypes: public vcg::UsedTypes<vcg::Use<MyVertex>::AsVertexType> {};
class MyVertex: public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f> {};
void PointCloud::computeNormals()
{
	// Quite horrible... we have to copy all the data into another structure, doubling the memory requirements...
	std::vector<MyVertex> vertices;
	vertices.reserve(myData.size());

	for(int i = 0; i < myData.size(); i++)
	{
		MyVertex v;
		v.P().X() = myData[i].position[0];
		v.P().Y() = myData[i].position[1];
		v.P().Z() = myData[i].position[2];

		vertices.push_back(v);
	}

	fprintf(stderr, "Computing normals (neighbors: %d)...\n", myNormalNeighbors);
	vcg::NormalExtrapolation<std::vector<MyVertex> >::ExtrapolateNormals(vertices.begin(), vertices.end(), myNormalNeighbors);

	// Copy back the normals.
	for(int i = 0; i < myData.size(); i++)
	{
		myData[i].normal[0] = vertices[i].N().X();
		myData[i].normal[1] = vertices[i].N().Y();
		myData[i].normal[2] = vertices[i].N().Z();
	}

	myHasNormals = true;
}

