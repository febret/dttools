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
#ifndef __DTTYPES_H__
#define __DTTYPES_H__

#include <queue>
#include <vector>
#include <stdio.h>

#define VMMLIB_DONT_FORCE_ALIGNMENT // Disable vmmlib data alignment since it doesn't work with std containers.
#include "vmmlib/vector.hpp"
#include "vmmlib/matrix.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif
#define DEG_TO_RAD(angle) ((angle)*M_PI/180.0)
#define RAD_TO_DEG(angle) ((angle)*180.0/M_PI)
#define MAX_RANGES 512

#define DTT_ASSERT(x) if(!(x)) { fprintf(stderr, "%s(%d) assertion failed: %s", __FILE__, __LINE__, #x); abort(); }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class FileFormat
{
public:
	enum Enum { 
		Invalid, 
		DeltaT83P,
		CSVENDURANCEDeltaT,
		CSVENDURANCEPose,
		CSVPoints, 
		CSVPoints2, 
		CSVPointsPose, 
		CSVPointsWithQuality, 
		CSVPointsWithBeamInfo,
		CSVPointsWithBeamPoseInfo,
		XYZPointsWithNormals, 
		BinaryPointsWithBeamInfo,
		//XYZBinaryPointsWithNormals, 
		VTKStructuredPointsASCII,
		PLY};

public:
	static const char* toString( FileFormat::Enum value)
	{
		switch(value)
		{
			case PLY: return "PLY";
			case DeltaT83P: return "DeltaT83P";
			case CSVENDURANCEDeltaT: return "CSVENDURANCEDeltaT";
			case CSVENDURANCEPose: return "CSVENDURANCEPose";
			case CSVPoints: return "CSVPoints";
			case CSVPoints2: return "CSVPoints2";
			case CSVPointsPose: return "CSVPointsPose";
			case XYZPointsWithNormals: return "XYZPointsWithNormals";
			//case XYZBinaryPointsWithNormals: return "XYZBinaryPointsWithNormals";
			case CSVPointsWithQuality: return "CSVPointsWithQuality";
			case VTKStructuredPointsASCII: return "VTKStructuredPointsASCII";
			case CSVPointsWithBeamInfo: return "CSVPointsWithBeamInfo";
			case CSVPointsWithBeamPoseInfo: return "CSVPointsWithBeamPoseInfo";
			case BinaryPointsWithBeamInfo: return "BinaryPointsWithBeamInfo";
		}
		fprintf(stderr, "Invalid file format code: %d\n", value);
		return "InvalidFormat";
	}

	static const char* toFileExtension( FileFormat::Enum value)
	{
		switch(value)
		{
			case PLY: return "ply";
			case DeltaT83P: return "d3p";
			case CSVENDURANCEDeltaT: return "csv";
			case CSVENDURANCEPose: return "csv";
			case CSVPoints: return "csv";
			case CSVPoints2: return "csv";
			case CSVPointsPose: return "csv";
			case XYZPointsWithNormals: return "xyz";
			//case XYZBinaryPointsWithNormals: return "XYZBinaryPointsWithNormals";
			case CSVPointsWithQuality: return "csv";
			case VTKStructuredPointsASCII: return "vtk";
			case CSVPointsWithBeamInfo: return "csv";
			case CSVPointsWithBeamPoseInfo: return "csv";
			case BinaryPointsWithBeamInfo: return "xyzb";
		}
		fprintf(stderr, "Invalid file format code: %d\n", value);
		return "InvalidFormat";
	}
    
	static Enum fromString(const char* value) 
	{
		if(!strcmp(value, "PLY")) return PLY;
		if(!strcmp(value, "DeltaT83P")) return DeltaT83P;
		if(!strcmp(value, "CSVENDURANCEDeltaT")) return CSVENDURANCEDeltaT;
		if(!strcmp(value, "CSVENDURANCEPose")) return CSVENDURANCEPose;
		if(!strcmp(value, "CSVPoints")) return CSVPoints;
		if(!strcmp(value, "CSVPoints2")) return CSVPoints2;
		if(!strcmp(value, "CSVPointsPose")) return CSVPointsPose;
		if(!strcmp(value, "XYZPointsWithNormals")) return XYZPointsWithNormals;
		//if(!strcmp(value, "XYZBinaryPointsWithNormals")) return XYZBinaryPointsWithNormals;
		if(!strcmp(value, "CSVPointsWithQuality")) return CSVPointsWithQuality;
		if(!strcmp(value, "VTKStructuredPointsASCII")) return VTKStructuredPointsASCII;
		if(!strcmp(value, "CSVPointsWithBeamInfo")) return CSVPointsWithBeamInfo;
		if(!strcmp(value, "CSVPointsWithBeamPoseInfo")) return CSVPointsWithBeamPoseInfo;
		if(!strcmp(value, "BinaryPointsWithBeamInfo")) return BinaryPointsWithBeamInfo;

		fprintf(stderr, "Invalid file format string: %s\n", value);
		return Invalid;
	}

private:
	FileFormat() {};
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SoundVelocityProfile
{
public:
	void load(const char* filename);

public:
	int size;
	float* depth;
	float* velocity;
	void* model;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Stores a single pose for the AUV
struct RangeDataPose
{
    RangeDataPose() {}

	int diveTag;
    double t;
	vmml::vec3d position;
	vmml::vec3f orientation;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Stores a single sonar ping.
struct RangeDataPing
{
    double t;
	double ranges[MAX_RANGES];
	RangeDataPose* pose;
	vmml::vec3d position;
	vmml::vec3f orientation;

	RangeDataPing(): t(0), pose(NULL) {}
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A point in the computed 3d sonar cloud.
struct SonarPoint
{
	SonarPoint() {}

	int diveTag;
	float range;
	float angle;
	double t;
	vmml::vec3d position;
	vmml::vec3f normal;
	vmml::vec3d sourcePosition;
	vmml::vec3f sourceOrientation;
};

#endif
