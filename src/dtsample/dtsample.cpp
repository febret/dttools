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

char* samplesFile;
char* outputFile;

float sampleRadius = 1.0f;

int inputBufferSize = 32000;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(const char* filename)
{
	printf("loading config %s...\n", filename);

	CFG_START(filename);

	CFG_MAP_STRING(outputFile);
	CFG_MAP_STRING(samplesFile);

	CFG_MAP_STRING(inputFormat);

	CFG_MAP_INT(inputFiles);
	for(int i = 0; i < inputFiles; i++)
	{
		CFG_MAP_STRING_ARRAY(inputFile, i);
	}

	CFG_MAP_INT(inputBufferSize);
	CFG_MAP_FLOAT(sampleRadius);

	CFG_END;

	inputBufferSize *= 1024;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	setup(argv[1]);

	PointCloud samplesCloud;
	samplesCloud.read(samplesFile, FileFormat::CSVPoints);
	std::vector<SonarPoint>& samples = samplesCloud.getPoints();
	for(int i = 0; i < samples.size(); i++)
	{
		// Use range to store number of samples.
		samples[i].range = 0;
	}

	for(int i = 0; i < inputFiles; i++)
	{
		fprintf(stderr, "\nProcessing %s...\n", inputFile[i]);
		PointCloud input;
		input.openInputFile(inputFile[i], FileFormat::fromString(inputFormat));
		while(!input.endOfFile())
		{
			input.readNext(inputBufferSize);
			//fprintf(stderr, ".");
			// For each sample, if point is within sample radius, add to accumulator
			std::vector<SonarPoint>& pts = input.getPoints();
			for(int j = 0; j < pts.size(); j++)
			{
				std::vector<SonarPoint>& samples = samplesCloud.getPoints();
				for(int k = 0; k < samples.size(); k++)
				{
					vmml::vec3d dist = (pts[j].position - samples[k].position);
					// Compute 2D distance.
					dist.z() = 0;
					if(dist.length() < sampleRadius)
					{
						samples[k].range++;
						// Accumulate sample depth
						samples[k].position[2] += pts[j].position[2];
						fprintf(stderr, "%-10f %-10f %-10f %-10f\n",
							pts[j].position[0], pts[j].position[1], pts[j].position[2]);
					}
				}
			}
			input.getPoints().clear();
		}
		input.closeFile();
	}

	// Average and compute depths
	fprintf(stderr, "Sampling radius: %f\n", sampleRadius);
	fprintf(stderr, "Point(X, Y, Depth) - Number of samples:\n");
	for(int i = 0; i < samples.size(); i++)
	{
		samples[i].position[2] = samples[i].position[2] / samples[i].range;
		fprintf(stderr, "%-10f %-10f %-10f %-10f\n",
			samples[i].position[0], samples[i].position[1], samples[i].position[2],
			samples[i].range);
	}

	fprintf(stderr, "Writing point cloud...\n");
	samplesCloud.write(outputFile, FileFormat::CSVPoints);
    return 0;
}

