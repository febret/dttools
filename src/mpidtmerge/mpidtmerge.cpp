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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "quickcfg.h"
#include "PointCloud.h"
#include "BinningGrid.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration parameters.
static const int MAX_FILES = 128;
int inputFiles;
char* inputFile[MAX_FILES];
char* inputFormat;
char* outputFile;
char* outputFormat;
float voxelSize;
int qualityThreshold;
int normalNeighbors = 0;
vmml::vec3f clipBoundsMin;
vmml::vec3f clipBoundsMax;

int inputBufferSize = 32000;
int preallocBins = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(const char* filename)
{
	voxelSize = 1;
	qualityThreshold = 0;

	printf("loading config %s...\n", filename);

	CFG_START(filename);

	CFG_MAP_STRING(outputFile);
	CFG_MAP_STRING(outputFormat);

	CFG_MAP_STRING(inputFormat);

	CFG_MAP_INT(inputFiles);
	for(int i = 0; i < inputFiles; i++)
	{
		CFG_MAP_STRING_ARRAY(inputFile, i);
	}

	CFG_MAP_FLOAT(voxelSize);
	CFG_MAP_INT(qualityThreshold);
	CFG_MAP_INT(normalNeighbors);

	for(int i = 0; i < 3; i++)
	{
		CFG_MAP_FLOAT_ARRAY(clipBoundsMin, i);
		CFG_MAP_FLOAT_ARRAY(clipBoundsMax, i);
	}

	CFG_MAP_INT(inputBufferSize);
	CFG_MAP_INT(preallocBins);

	CFG_END;

	inputBufferSize *= 1024;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	float dmax = std::numeric_limits<float>::max();
	float dmin = -std::numeric_limits<float>::max();

	clipBoundsMax = vmml::vec3f(dmax, dmax, dmax);
	clipBoundsMin = vmml::vec3f(dmin, dmin, dmin);

	vmml::vec3f minBounds = vmml::vec3f(dmax, dmax, dmax);
	vmml::vec3f maxBounds = vmml::vec3f(dmin, dmin, dmin);

	setup(argv[1]);

	// Computing bounds
	fprintf(stderr, "Computing data bounds...\n");
	for(int i = 0; i < inputFiles; i++)
	{
		PointCloud input;
		input.openInputFile(inputFile[i], FileFormat::fromString(inputFormat));
		int pointCount = 0;
		while(!input.endOfFile())
		{
			input.readNext(inputBufferSize);
			pointCount += input.getPoints().size();
			input.getPoints().clear();
		}
		input.closeFile();
		fprintf(stderr, "%s total points: %d\n", inputFile[i], pointCount);

		vmml::vec3f minb = input.getMinBounds();
		vmml::vec3f maxb = input.getMaxBounds();

		// Update bounds
		for(int j = 0; j < 3; j++)
		{
			if(minb[j] < minBounds[j]) minBounds[j] = minb[j];
			if(minb[j] > maxBounds[j]) maxBounds[j] = minb[j];
			if(maxb[j] < minBounds[j]) minBounds[j] = maxb[j];
			if(maxb[j] > maxBounds[j]) maxBounds[j] = maxb[j];
		}

	}

	fprintf(stderr, "Processing data...\n");

	// Update bounds using clipping bounds
	for(int i = 0; i < 3; i++)
	{
		if(minBounds[i] < clipBoundsMin[i]) minBounds[i] = clipBoundsMin[i];
		if(maxBounds[i] > clipBoundsMax[i]) maxBounds[i] = clipBoundsMax[i];
	}

	printf("min bounds: %f, %f, %f\n", minBounds[0], minBounds[1], minBounds[2]);
	printf("max bounds: %f, %f, %f\n", maxBounds[0], maxBounds[1], maxBounds[2]);

	BinningGrid grid;
	grid.initialize(minBounds, maxBounds, voxelSize, preallocBins);

	for(int i = 0; i < inputFiles; i++)
	{
		PointCloud input;
		input.openInputFile(inputFile[i], FileFormat::fromString(inputFormat));
		while(!input.endOfFile())
		{
			input.readNext(inputBufferSize);
			grid.addPoints(input);
			input.getPoints().clear();
		}
		input.closeFile();
	}

	fprintf(stderr, "Reading back point cloud...\n");
	PointCloud cloud;
	grid.writeToPointCloud(&cloud, qualityThreshold);

	cloud.setNormalNeighbors(normalNeighbors);

	if(!cloud.hasNormals() && normalNeighbors != 0)
	{
		cloud.computeNormals();
	}

	vmml::vec3f minb = cloud.getMinBounds();
	vmml::vec3f maxb = cloud.getMaxBounds();
	printf("min bounds: %f, %f, %f\n", minb[0], minb[1], minb[2]);
	printf("max bounds: %f, %f, %f\n", maxb[0], maxb[1], maxb[2]);

	printf("Writing output (%d items)...\n", cloud.getPoints().size());
	cloud.write(outputFile, FileFormat::fromString(outputFormat));

    return 0;
}

