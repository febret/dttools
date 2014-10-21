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
#include <sstream>
#include <limits>
#include <time.h>

#include "quickcfg.h"
#include "mb_fbt.h"

#include "PointCloud.h"
#include "DeltaTData.h"
#include "PoseData.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tool configuration parameters.
static const int MAX_FILES = 128;
int inputFiles;
char* deltatFile[MAX_FILES];
char* poseFile[MAX_FILES];
int   diveTag[MAX_FILES];

char* outputFile = "output";
char outputFullFilename[MAX_FILES];
char* outputFormat = "CSVPoints";
RaytracerConfiguration raytracerCfg;

char* inputDeltaTFormat = "CSVENDURANCEDeltaT";
char* inputPoseFormat = "CSVENDURANCEPose";

// Beam Raytracing config
char* svpFile = NULL;
float sensorSoundVelocity = 1500;

// Sonar information
vmml::vec3d sensorPosition;
vmml::vec3d sensorRotation;
double rangeThreshold = 10; 

// Data processing config
int beamDecimation;
int pingDecimation;

// Beam angle filtering
float beamFilterAngleMin = -180;
float beamFilterAngleMax = 180;

// Vehicle pose filtering
vmml::vec3f vehiclePositionMin;
vmml::vec3f vehiclePositionMax;

// Pose transformations
vmml::vec3f poseTranslation = vmml::vec3f::ZERO;
float poseTransform[9];
vmml::mat4f poseWorldTransform;

// Buffering
int outputBufferSize = 8000; // Use an 8 mb buffer for writing. Size doesn't really get any speed advantage here.

void loadConfig(const char* cfg);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void loadConfigInclude( const char* cfg )
{
	char* includeConfig = NULL;

	CFG_START(cfg);
	CFG_MAP_STRING(includeConfig);
	CFG_END;
	if(includeConfig != NULL) loadConfig(includeConfig);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loadConfig(const char* cfg)
{
	// check for an include
	loadConfigInclude(cfg);

	printf("loading config %s...\n", cfg);
	CFG_START(cfg);

	CFG_MAP_STRING(inputDeltaTFormat);
	CFG_MAP_STRING(inputPoseFormat);

	CFG_MAP_INT(inputFiles);
	for(int i = 0; i < inputFiles; i++)
	{
		CFG_MAP_STRING_ARRAY(deltatFile, i);
		CFG_MAP_STRING_ARRAY(poseFile, i);
		CFG_MAP_INT_ARRAY(diveTag, i);
	}

	CFG_MAP_INT(pingDecimation);
	CFG_MAP_INT(beamDecimation);

	for(int i = 0; i < 3; i++)
	{
		CFG_MAP_FLOAT_ARRAY(sensorPosition, i);
		CFG_MAP_FLOAT_ARRAY(sensorRotation, i);
		CFG_MAP_FLOAT_ARRAY(poseTranslation, i);
	}

	for(int i = 0; i < 9; i++)
	{
		CFG_MAP_FLOAT_ARRAY(poseTransform, i);
	}

	CFG_MAP_STRING(svpFile);
	CFG_MAP_FLOAT(sensorSoundVelocity);

	CFG_MAP_FLOAT(rangeThreshold);
	CFG_MAP_FLOAT(beamFilterAngleMin);
	CFG_MAP_FLOAT(beamFilterAngleMax);

	for(int i = 0; i < 3; i++)
	{
		CFG_MAP_FLOAT_ARRAY(vehiclePositionMin, i);
		CFG_MAP_FLOAT_ARRAY(vehiclePositionMax, i);
	}

	CFG_MAP_STRING(outputFile);
	CFG_MAP_STRING(outputFormat);
	CFG_MAP_INT(outputBufferSize);

	CFG_END;
    
    sprintf(outputFullFilename, "%s.%s", outputFile, FileFormat::toFileExtension(FileFormat::fromString(outputFormat)));
}

// Records the configuration used for this processing set in a file with the output file name + ".cfg"
void recordConfig( const char* cfgFile, char* argv[] ) 
{
	char timestr[64];

	// some preliminaries
	time_t now = time(NULL);
	strftime(timestr, 64, "%F %T UTC", gmtime(&now));

	CFG_REC_START(outputFullFilename); // record config with same base name as output file
	CFG_REC_COMMENT_FMT("Configuration used to create %s", outputFullFilename);
	char cmdline[512] = ""; for (int i=0,c=0; argv[i] && c < 511; i++) c += sprintf(cmdline+c, " %s", argv[i]); // hope we don't overflow cmdline...
	CFG_REC_COMMENT_FMT("Command line: %s", cmdline);
	CFG_REC_COMMENT_FMT("Base configuration file: %s", cfgFile);
	CFG_REC_COMMENT_FMT("Processed at %s.", timestr);
	CFG_REC_COMMENT("using dtrt executable compiled on " __DATE__ " at " __TIME__ ".");

	// the following should match what's read in loadConfig()
	CFG_REC_SEC;
	CFG_REC_STRING(inputDeltaTFormat);
	CFG_REC_STRING(inputPoseFormat);

	CFG_REC_SEC;
	CFG_REC_INT(inputFiles);
	for(int i = 0; i < inputFiles; i++)
	{
		CFG_REC_STRING_ARRAY(deltatFile, i);
		CFG_REC_STRING_ARRAY(poseFile, i);
		CFG_REC_INT_ARRAY(diveTag, i);
	}

	CFG_REC_SEC;
	CFG_REC_INT(pingDecimation);
	CFG_REC_INT(beamDecimation);

	CFG_REC_SEC;
	for(int i = 0; i < 3; i++) CFG_REC_FLOAT_ARRAY(sensorPosition, i);
	CFG_REC_SEC;
	for(int i = 0; i < 3; i++) CFG_REC_FLOAT_ARRAY(sensorRotation, i);
	CFG_REC_SEC;
	for(int i = 0; i < 3; i++) CFG_REC_FLOAT_ARRAY(poseTranslation, i);

	CFG_REC_SEC;
	for(int i = 0; i < 9; i++)
	{
		CFG_REC_FLOAT_ARRAY(poseTransform, i);
	}

	CFG_REC_SEC;
	CFG_REC_STRING(svpFile);
	CFG_REC_FLOAT(sensorSoundVelocity);

	CFG_REC_SEC;
	CFG_REC_FLOAT(rangeThreshold);
	CFG_REC_FLOAT(beamFilterAngleMin);
	CFG_REC_FLOAT(beamFilterAngleMax);

	CFG_REC_SEC;
	for(int i = 0; i < 3; i++) CFG_REC_FLOAT_ARRAY(vehiclePositionMin, i);
	CFG_REC_SEC;
	for(int i = 0; i < 3; i++) CFG_REC_FLOAT_ARRAY(vehiclePositionMax, i);

	CFG_REC_SEC;
	CFG_REC_STRING(outputFile);
	CFG_REC_STRING(outputFormat);
	CFG_REC_INT(outputBufferSize);

	CFG_REC_END;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(const char* cfgFile)
{

	memset(poseTransform, 0, sizeof(float) * 9);
	poseTransform[0] = 1;
	poseTransform[4] = 1;
	poseTransform[8] = 1;

	vehiclePositionMin = -std::numeric_limits<float>::max();
	vehiclePositionMax =  std::numeric_limits<float>::max();

	loadConfig(cfgFile);


        beamDecimation = beamDecimation < 1 ? 1 : beamDecimation;
        pingDecimation = pingDecimation < 1 ? 1 : pingDecimation;

	// Load the sound velocity profiles
	if(svpFile != NULL) raytracerCfg.svp.load(svpFile);

	raytracerCfg.setSensorTransform(sensorPosition, sensorRotation);
	raytracerCfg.beamCount = DeltaTData::ENDURANCEDeltaTBeamCount;
	raytracerCfg.beamFilterAngleMax = beamFilterAngleMax;
	raytracerCfg.beamFilterAngleMin = beamFilterAngleMin;
	raytracerCfg.rangeThreshold = rangeThreshold;
	raytracerCfg.beamDecimation = beamDecimation;
	raytracerCfg.vehiclePositionMin = vehiclePositionMin;
	raytracerCfg.vehiclePositionMax = vehiclePositionMax;
	raytracerCfg.sensorSoundVelocity = sensorSoundVelocity;

	fprintf(stderr, "Sound velocity set in DeltaT sensor: %f\n", sensorSoundVelocity);

	// Setup the pose world transform.
	poseWorldTransform[0][0] = poseTransform[0];
	poseWorldTransform[0][1] = poseTransform[1];
	poseWorldTransform[0][2] = poseTransform[2];
	poseWorldTransform[0][3] = poseTranslation[0];

	poseWorldTransform[1][0] = poseTransform[3];
	poseWorldTransform[1][1] = poseTransform[4];
	poseWorldTransform[1][2] = poseTransform[5];
	poseWorldTransform[1][3] = poseTranslation[1];

	poseWorldTransform[2][0] = poseTransform[6];
	poseWorldTransform[2][1] = poseTransform[7];
	poseWorldTransform[2][2] = poseTransform[8];
	poseWorldTransform[2][3] = poseTranslation[2];

	poseWorldTransform[3][0] = 0;
	poseWorldTransform[3][1] = 0;
	poseWorldTransform[3][2] = 0;
	poseWorldTransform[3][3] = 1;
	raytracerCfg.setWorldTransform(poseWorldTransform);

	outputBufferSize *= 1024;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void processDive(const char* deltaTFilename, const char* poseFilename, int diveTag, PointCloud& pointCloud, RaytracerConfiguration& cfg)
{
	DeltaTData deltaTData;
	PoseData poseData;

	printf("reading %s...\n", deltaTFilename);
	deltaTData.read(deltaTFilename, FileFormat::fromString(inputDeltaTFormat), pingDecimation);

	printf("reading %s...\n", poseFilename);
	poseData.read(poseFilename, FileFormat::fromString(inputPoseFormat));

	printf("processing %d pings...\n", deltaTData.getLength());

	PingStats pingStats;

	int poseIdx = 0; // pose data index
	int pingIdx = 0; // deltaT data index
	
	// advance ping index to first pose data (assumes invalid pose data has been cropped)
	RangeDataPose& firstPose = poseData.getPose(poseIdx);
	while (pingIdx < deltaTData.getLength() && deltaTData.getPing(pingIdx).t < firstPose.t) pingIdx++;
	if (pingIdx > 0)
		printf("First %d sonar pings have no associated pose data, skipping...\n", pingIdx);

	printf("  0%%"); fflush(stdout);

	for(; pingIdx < deltaTData.getLength(); pingIdx++)
	{
	  if ( pingIdx % (deltaTData.getLength() / 100) == 0)
	    {
	      printf("\b\b\b\b%3d%%",pingIdx*100/deltaTData.getLength());
	      fflush(stdout);
	    }
		// Merge pose and range data.
		RangeDataPing& ping = deltaTData.getPing(pingIdx);
		while(poseIdx < poseData.getLength() && poseData.getPose(poseIdx).t < ping.t) poseIdx++;

		// Interpolate?
		ping.position = poseData.getPose(poseIdx).position;
		ping.orientation = poseData.getPose(poseIdx).orientation;

		// Generate a point cloud for this ping and store it in the point cloud manager by the generator.
		pointCloud.addPing(ping, cfg, pingStats);

		if(pointCloud.getPoints().size() >= outputBufferSize / sizeof(SonarPoint))
		{
			pointCloud.flush();
		}
	}
	printf("\b\b\b\b100%%\n");

	float passedPerc = pingStats.passedPoints * 100.0f / pingStats.totalPoints;
	float addedPerc = pingStats.passedPoints > 0 ? pingStats.addedPoints * 100.0f / pingStats.passedPoints : 100.0f;
	float failedPerc = pingStats.passedPoints > 0 ? pingStats.raytraceFailures * 100.0f / pingStats.passedPoints : 0.0f;

	printf("Points Total: %dk Passed: %dk(%.0f%% of total) Added: %dk(%.0f%% of passed) Raytrace Failures: %dk(%.0f%% of passed)\n",
		pingStats.totalPoints / 1000, 
		pingStats.passedPoints / 1000, passedPerc,
		pingStats.addedPoints / 1000, addedPerc,
		pingStats.raytraceFailures / 1000, failedPerc);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		fprintf(stderr, "%s: no configuration file specified.\n", argv[0]);
		return -1;
	}

	char* cfgFile = argv[1];

	// Load the tool configuration.
	setup(cfgFile);

	// record the tool configuration alongside the data
	recordConfig(cfgFile, argv);

	PointCloud pointCloud;
	pointCloud.getPoints().reserve(outputBufferSize / sizeof(SonarPoint));

	pointCloud.openOutputFile(outputFullFilename, FileFormat::fromString(outputFormat));

	int lastDiveTag = diveTag[0];
	for(int i = 0; i < inputFiles; i++)
	{
		processDive(deltatFile[i], poseFile[i], diveTag[i], pointCloud, raytracerCfg);
	}

	vmml::vec3d minb = pointCloud.getMinBounds();
	vmml::vec3d maxb = pointCloud.getMaxBounds();
	printf("min bounds: %f, %f, %f\n", minb[0], minb[1], minb[2]);
	printf("max bounds: %f, %f, %f\n", maxb[0], maxb[1], maxb[2]);

	pointCloud.flush();
	pointCloud.closeFile();

    return 0;
}
