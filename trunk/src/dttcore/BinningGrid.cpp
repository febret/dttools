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

#include "BinningGrid.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BinningGrid::BinningGrid(): 
	myResolution(0), 
	myOctree(NULL), 
	myWorldGrid(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BinningGrid::~BinningGrid()
{
	if(myWorldGrid) delete myWorldGrid;
	if(myOctree) delete myOctree;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BinningGrid::initialize(const vmml::vec3d& minBounds, const vmml::vec3d& maxBounds, float voxelSize, int preallocBins)
{
	vmml::vec3d size = maxBounds - minBounds;

	mySize = size;
	myMinBounds = minBounds;
	myMaxBounds = maxBounds;

	// Compute the scaled 3d resolution of the occupancy grid.
	double maxSize = size[0];
	if(size[1] > maxSize) maxSize = size[1];
	if(size[2] > maxSize) maxSize = size[2];

	int resolution = (int)(maxSize / voxelSize);

	fprintf(stderr, "BinningGrid: voxel size = %f, resolution = %d\n", voxelSize, resolution);

	size[0] = (double)(resolution - 1) / maxSize;
	size[1] = (double)(resolution - 1) / maxSize;
	size[2] = (double)(resolution - 1) / maxSize;

	myResolution = resolution;

	vmml::mat4d translation = vmml::mat4d::IDENTITY;
	vmml::mat4d scale = vmml::mat4d::IDENTITY;
	translation.set_translation(-minBounds);
	scale.scale(size);

	vmml::mat4d transform = scale * translation;

	// Copy vmml matrix to ravec transform (ravec matrix need to be transposed after copy)
	for(int i = 0; i < 16; i++) myTransform.T_[i] = transform.begin()[i];
	myTransform.Transpose();

	myOctree = new ravec::OctreeGrid<int>(resolution, resolution, resolution, -1);
	myWorldGrid = new ravec::WorldGrid<int>(myTransform, myOctree);

	if(preallocBins != 0)
	{
		myBinData.reserve(preallocBins);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BinningGrid::addPoints(PointCloud& points)
{
	int numPoints = points.getPoints().size();
	fprintf(stderr, "Tracing points (%d)...\n", numPoints);

	float size = 1;

	myHasNormals = points.hasNormals();

	for(int i = 0; i < numPoints; i++)
	{
		SonarPoint& pt = points.getPoints()[i];

		vmml::vec3d& pos = pt.position;

		int x, y, z;
		myWorldGrid->WorldToMap(pos[0], pos[1], pos[2], &x, &y, &z);

		if(x >= 0 && x < myResolution &&
			y >= 0 && y < myResolution &&
			z >= 0 && z < myResolution)
		{
			int dataIndex = myOctree->Get(x, y, z); 
			if(dataIndex == -1)
			{
				// Add data.
				BinData data;
				data.numPoints = 1;
				data.position = pt.position;

				if(points.hasNormals()) data.normal = pt.normal;

				myBinData.push_back(data);

				dataIndex = myBinData.size() - 1;
				myOctree->Set(x, y, z, dataIndex);
			}
			else
			{
				BinData& data = myBinData.at(dataIndex);
				data.numPoints++;
				data.position += pt.position;
				if(points.hasNormals()) data.normal += pt.normal;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BinningGrid::writeToPointCloud(PointCloud* cloud, int qualityThreshold)
{
	int filtered = 0;
	fprintf(stderr, "Writing %d of %d points to point cloud...\n", myBinData.size());
	// Reserve memory
	int items = 0;
	for(unsigned int i = 0; i < myBinData.size(); i++)
	{
		BinData& data = myBinData.at(i);
		if(data.numPoints > qualityThreshold)
		{
			items++;
		}
	}
	cloud->getPoints().clear();
	cloud->getPoints().reserve(items);

	for(unsigned int i = 0; i < myBinData.size(); i++)
	{
		BinData& data = myBinData.at(i);
		if(data.numPoints > qualityThreshold)
		{
			SonarPoint pt;
			pt.position = data.position / data.numPoints;
			if(pt.position[0] > myMinBounds[0] && pt.position[0] < myMaxBounds[0] &&
				pt.position[1] > myMinBounds[1] && pt.position[1] < myMaxBounds[1] &&
				pt.position[2] > myMinBounds[2] && pt.position[2] < myMaxBounds[2])
			{
				pt.normal = data.normal / data.numPoints;
				pt.normal.normalize();
				cloud->getPoints().push_back(pt);
				cloud->setHasNormals(myHasNormals);
			}
			else
			{
				filtered++;
			}
		}
		else 
		{
				filtered++;
		}
	}
	cloud->updateBounds();
	int filteredPercent = filtered * 100 / myBinData.size();
	fprintf(stderr, "point quality threshold: %d. Filtered out points: %d%%\n", qualityThreshold, filteredPercent);
}

