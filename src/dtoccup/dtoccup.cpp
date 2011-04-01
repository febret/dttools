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
#include "OccupancyGrid.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration parameters.
char* inputFile;
char* inputFormat;
char* outputFile;
char* outputFormat;
int occupiedCellValue;
int emptyCellValue;
int occupancyThreshold;
int gridResolution;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(const char* filename)
{
	occupiedCellValue = 8;
	emptyCellValue = -2;
	occupancyThreshold = OccupancyGrid::NoThreshold;
	gridResolution = 512;

	printf("loading config %s...\n", filename);

	CFG_START(filename);

	CFG_MAP_STRING(outputFile);
	CFG_MAP_STRING(outputFormat);

	CFG_MAP_STRING(inputFile);
	CFG_MAP_STRING(inputFormat);

	CFG_MAP_INT(occupancyThreshold);
	CFG_MAP_INT(occupiedCellValue);
	CFG_MAP_INT(emptyCellValue);

	CFG_MAP_INT(gridResolution);

	CFG_END;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	setup(argv[1]);

	PointCloud points;
	points.read(inputFile, FileFormat::fromString(inputFormat));

	vmml::vec3d minb = points.getMinBounds();
	vmml::vec3d maxb = points.getMaxBounds();
	printf("min bounds: %.4f, %.4f, %.4f\n", minb[0], minb[1], minb[2]);
	printf("max bounds: %.4f, %.4f, %.4f\n", maxb[0], maxb[1], maxb[2]);

	OccupancyGrid occGrid;
	occGrid.initialize(minb, maxb, gridResolution);

	occGrid.setOccupancyThreshold(occupancyThreshold);
	occGrid.setEmptyCellValue(emptyCellValue);
	occGrid.setOccupiedCellValue(occupiedCellValue);

	occGrid.tracePoints(points);
	occGrid.write(outputFile, FileFormat::fromString(outputFormat));
    return 0;
}

