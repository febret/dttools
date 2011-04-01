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

#include "OccupancyGrid.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid::OccupancyGrid(): 
	myResolution(0), 
	myOctree(NULL), 
	myWorldGrid(NULL),
	// Following values taken from Fairfield N. et Al., 
	// Real-time SLAM with Octree Evidence Grids for Exploration in Underwater Tunnels, 2006.
	myEmptyCellValue(-2),
	myOccupiedCellValue(8),
	myOccupancyThreshold(OccupancyGrid::NoThreshold)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid::~OccupancyGrid()
{
	if(myWorldGrid) delete myWorldGrid;
	if(myOctree) delete myOctree;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyGrid::initialize(const vmml::vec3f& minBounds, const vmml::vec3f& maxBounds, int resolution)
{
	vmml::vec3f size = maxBounds - minBounds;

	mySize = size;
	myMinBounds = minBounds;
	myMaxBounds = maxBounds;
	myResolution = resolution;

	// Compute the scaled 3d resolution of the occupancy grid.
	float maxSize = size[0];
	if(size[1] > maxSize) maxSize = size[1];
	if(size[2] > maxSize) maxSize = size[2];
	myScaledResolution = (size / maxSize) * (float)resolution;

	size[0] = (float)(resolution - 1) / size[0];
	size[1] = (float)(resolution - 1) / size[1];
	size[2] = (float)(resolution - 1) / size[2];

	vmml::mat4f translation = vmml::mat4d::IDENTITY;
	vmml::mat4f scale = vmml::mat4d::IDENTITY;
	translation.set_translation(-minBounds);
	scale.scale(size);

	vmml::mat4f transform = scale * translation;

	// Copy vmml matrix to ravec transform (ravec matrix need to be transposed after copy)
	for(int i = 0; i < 16; i++) myTransform.T_[i] = transform.begin()[i];
	myTransform.Transpose();

	myOctree = new ravec::OctreeGrid<int>(resolution, resolution, resolution, 0);
	myWorldGrid = new ravec::WorldGrid<int>(myTransform, myOctree);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyGrid::tracePoints(PointCloud& points)
{
	int numPoints = points.getPoints().size();
	fprintf(stderr, "Tracing points (%d)...\n", numPoints);

	float size = 1;

	for(int i = 0; i < numPoints; i++)
	{
		SonarPoint& pt = points.getPoints()[i];
		
		myWorldGrid->AddLineWorld(
			pt.sourcePosition[0], pt.sourcePosition[1], pt.sourcePosition[2],
			pt.position[0], pt.position[1], pt.position[2], 
			myEmptyCellValue);

		//myWorldGrid->AddWorld(
		//	pt.sourcePosition[0], pt.sourcePosition[1], pt.sourcePosition[2],
		//	myEmptyCellValue);

		myWorldGrid->AddWorld(
			pt.position[0], pt.position[1], pt.position[2], 
			myOccupiedCellValue);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyGrid::write(const std::string& filename, FileFormat::Enum format)
{
	DTT_ASSERT(format == FileFormat::VTKStructuredPointsASCII);

	FILE* fout = fopen(filename.c_str(), "w");
	if(fout != NULL)
	{
		if(format == FileFormat::VTKStructuredPointsASCII) writeVtkStructuredPoints(fout);

		fclose(fout);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyGrid::writeVtkStructuredPoints(FILE* fout)
{
	fprintf(stderr, "Writing resampled vtk grid (%dx%dx%d)...\n", myScaledResolution[0], myScaledResolution[1], myScaledResolution[2]);

	fprintf(fout, "# vtk DataFile Version 2.0\n");
	fprintf(fout, "Volume\n");
	fprintf(fout, "ASCII\n");
	fprintf(fout, "DATASET STRUCTURED_POINTS\n");
	fprintf(fout, "DIMENSIONS %d %d %d\n", myScaledResolution[0], myScaledResolution[1], myScaledResolution[2]);
	fprintf(fout, "ASPECT_RATIO 1 1 1\n");
	fprintf(fout, "ORIGIN 0 0 0\n");

	int numPoints = myScaledResolution[0] * myScaledResolution[1] * myScaledResolution[2];
	fprintf(fout, "POINT_DATA %d\n", numPoints);

	fprintf(fout, "SCALARS volume_scalars int 1\n");
	fprintf(fout, "LOOKUP_TABLE default\n");

	float dx = mySize[0] / myScaledResolution[0];
	float dy = mySize[1] / myScaledResolution[1];
	float dz = mySize[2] / myScaledResolution[2];
	for(float z = myMinBounds[2]; z < myMaxBounds[2]; z += dz)
	{
		for(float y = myMinBounds[1]; y < myMaxBounds[1]; y += dy)
		{
			for(float x = myMinBounds[0]; x < myMaxBounds[0]; x += dx)
			{
				int val = myWorldGrid->GetWorld(x, y, z);

				if(myOccupancyThreshold != OccupancyGrid::NoThreshold)
				{
					// Set value as occupied or empty depending on threshold.
					if(val > myOccupancyThreshold) val = 1;
					else val = 0;
				}

				fprintf(fout, "%d ", val);
			}
			fprintf(fout, "\n");
		}
	}

}


