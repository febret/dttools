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
#ifndef BINNING_GRID
#define BINNING_GRID

#include "dttypes.h"
#include "ravec/world_grid.h"
#include "ravec/octree_grid.h"

#include "PointCloud.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct BinData
{
	BinData() {}

	vmml::vec3d position;
	vmml::vec3d normal;
	int numPoints;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BinningGrid
{
public:
	BinningGrid();
	~BinningGrid();

	void initialize(const vmml::vec3d& minBounds, const vmml::vec3d& maxBounds, float voxelSize, int preallocBins);
	void addPoints(PointCloud& points);

	void writeToPointCloud(PointCloud* cloud, int qualityThreshold = 0);

private:
	int myResolution;

	bool myHasNormals;

	vmml::vec3d mySize;
	vmml::vec3d myMinBounds;
	vmml::vec3d myMaxBounds;

	ravec::Transform myTransform;
	ravec::WorldGrid<int>* myWorldGrid;
	ravec::OctreeGrid<int>* myOctree;

	std::vector<BinData> myBinData;
};

#endif