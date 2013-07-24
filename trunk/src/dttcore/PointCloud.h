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
#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "dttypes.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct RaytracerConfiguration
{
	RaytracerConfiguration():
		beamCount(480),
		beamAngleStart(-60.0),
		beamAngleEnd(60.0),
		//beamAngleOffset(0),
		rangeThreshold(10),
		pingDecimation(1),
		beamDecimation(1),
		sensorSoundVelocity(1500),
		sensorTransform(vmml::mat4d::IDENTITY),
		worldTransform(vmml::mat4d::IDENTITY)
		{}

	void setSensorTransform(const vmml::vec3d& sensorPosition, const vmml::vec3d& sensorRotation)
	{
		sensorTransform = vmml::mat4d::IDENTITY;
		sensorTransform.rotate_z(DEG_TO_RAD(sensorRotation[2]));
		sensorTransform.rotate_y(DEG_TO_RAD(sensorRotation[1]));
		sensorTransform.rotate_x(DEG_TO_RAD(sensorRotation[0]));
		sensorTransform.set_translation(sensorPosition);
		
		//beamAngleOffset = sensorRotation[1];

		//sensorTransform.rotate_y(0);
	}

	void setWorldTransform(const vmml::mat4d& value)
	{
		worldTransform = value;
	}

	// Sonar information
	unsigned int beamCount;
	double beamAngleStart;
	double beamAngleEnd;
	double rangeThreshold;
	//double beamAngleOffset;
	double beamFilterAngleMin;
	double beamFilterAngleMax;

	// Data processing config
	int pingDecimation;
	int beamDecimation;

	// Raytracing data.
	SoundVelocityProfile svp;
	float sensorSoundVelocity;

	vmml::mat4d sensorTransform;
	vmml::mat4d worldTransform;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct PingStats
{
	PingStats(): totalPoints(0), passedPoints(0), addedPoints(0), raytraceFailures(0) {}

	int totalPoints;
	int passedPoints;
	int addedPoints;
	int raytraceFailures;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PointCloud
{
public:

public:
	PointCloud();
	~PointCloud();

	bool hasNormals() { return myHasNormals; }
	void setHasNormals(bool value) { myHasNormals = value; }

	void write(const std::string& filename, FileFormat::Enum format, bool append = false);
	void read(const std::string& filename, FileFormat::Enum format);

	//! Stream IO
	void openOutputFile(const std::string& filename, FileFormat::Enum format, bool append = false);
	void openInputFile(const std::string& filename, FileFormat::Enum format);
	void deleteOutputFile(const std::string& filename);
	void mergeOutputFiles(const std::string& finalOutputFile, const std::string& singleOutputFile);
	void closeFile();
	bool endOfFile();
	//! Set read offset. Used by multiprocessor implementation to distribute reads.
	void setReadOffset(long int offset);
	//! If file offset gets past read length, endOfFil returns true (even if we are not really at end of file)
	//! Used by multiprocessor implementation to distribute reads.
	void setReadLength(long int offset);
	long int getSizeInBytes();
	void readNext(int bufferSize);
	void flush();

    void addPing(const RangeDataPing& ping, const RaytracerConfiguration& cfg, PingStats& stats);

	inline bool beamHitNaive(double theta, double r, const vmml::mat4d& sensorToWorld,  vmml::vec4d& beamHitW);


	bool beamHitRaytrace( double theta, double r, const vmml::mat4d& sensorToWorld, const SoundVelocityProfile& svp, float sensorSoundVelocity, vmml::vec4d& beamHitW);

	void appendPoints(PointCloud& points);
	void addPoint(const SonarPoint& point);

	void computeNormals();

	std::vector<SonarPoint>& getPoints() { return myData; }
	vmml::vec3f& getMinBounds() { return myMinBounds; }
	vmml::vec3f& getMaxBounds() { return myMaxBounds; }

	int getNormalNeighbors() { return myNormalNeighbors; }
	void setNormalNeighbors(int value) { myNormalNeighbors = value; }

	void updateBounds();

private:
	void updateBounds(const SonarPoint& point);
	void resetBounds();
	void writeItem(const SonarPoint& item);

private:
	std::vector<SonarPoint> myData;
	vmml::vec3f myMinBounds; 
	vmml::vec3f myMaxBounds; 

	int myNormalNeighbors;

	bool myHasNormals;
	bool myHasPoseInfo;
	bool myHasBeamInfo;
	bool myHasQuality;

	// Stream IO
	FILE* myFile;
	void* myBuffer;
	bool myIsFileOutput;
	bool myFileEof;
	bool myReadDone;
	int myFileOffset;
	int myReadOffset;
	int myReadLength;
	FileFormat::Enum myFormat;
};

#endif
