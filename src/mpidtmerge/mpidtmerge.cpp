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
#include <mpi.h>
#include <sys/time.h>

#include <vector>

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

int NPROCS = 0; // Number of processors.
int PID = 0; // Rank of current processor.
	
///////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a high precision timer value in milliseconds
double mstime()
{
	struct timeval tp1;
	gettimeofday(&tp1, NULL);
    double time = (tp1.tv_sec * 1000000.0) + tp1.tv_usec;
	return time / 1000.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(const char* filename)
{
	voxelSize = 1;
	qualityThreshold = 0;

	//printf("loading config %s...\n", filename);

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
void computeBounds(vmml::vec3f& minBounds, vmml::vec3f& maxBounds)
{
	//fprintf(stderr, "Computing data bounds...\n");
	for(int i = 0; i < inputFiles; i++)
	{
		PointCloud input;
		input.openInputFile(inputFile[i], FileFormat::fromString(inputFormat));
		
		// Find the total size of the input file. Compute how much we need to read for each node.
		long int size = input.getSizeInBytes();
		long int sizePerProc = size / NPROCS;
		input.setReadLength(sizePerProc);
		
		// Set the read offset for this node
		input.setReadOffset(sizePerProc * PID);
		
		int pointCount = 0;
		while(!input.endOfFile())
		{
			input.readNext(inputBufferSize);
			pointCount += input.getPoints().size();
			input.getPoints().clear();
		}
		input.closeFile();
		//fprintf(stderr, "PID %d: %s total points: %d\n", PID, inputFile[i], pointCount);

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
	
	float gmax[3];
	float gmin[3];
	// Compute global minimums and maximums
	for(int i = 0; i < 3; i++)
	{
		MPI_Allreduce(&minBounds[i], &gmin[i], 1, MPI_FLOAT, MPI_MIN, MPI_COMM_WORLD);
		MPI_Allreduce(&maxBounds[i], &gmax[i], 1, MPI_FLOAT, MPI_MAX, MPI_COMM_WORLD);
	}
	
	minBounds = gmin;
	maxBounds = gmax;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	// Initialize MPI
	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &NPROCS);
	MPI_Comm_rank(MPI_COMM_WORLD, &PID);
	
	float dmax = std::numeric_limits<float>::max();
	float dmin = -std::numeric_limits<float>::max();

	clipBoundsMax = vmml::vec3f(dmax, dmax, dmax);
	clipBoundsMin = vmml::vec3f(dmin, dmin, dmin);

	vmml::vec3f minBounds = vmml::vec3f(dmax, dmax, dmax);
	vmml::vec3f maxBounds = vmml::vec3f(dmin, dmin, dmin);

	setup(argv[1]);
	
	double startTime = mstime();

	// Computing bounds
	computeBounds(minBounds, maxBounds);
	
	//fprintf(stderr, "Processing data...\n");

	// Update bounds using clipping bounds
	for(int i = 0; i < 3; i++)
	{
		if(minBounds[i] < clipBoundsMin[i]) minBounds[i] = clipBoundsMin[i];
		if(maxBounds[i] > clipBoundsMax[i]) maxBounds[i] = clipBoundsMax[i];
	}

	
	if(PID == 0)
	{
		printf("min bounds: %f, %f, %f\n", minBounds[0], minBounds[1], minBounds[2]);
		printf("max bounds: %f, %f, %f\n", maxBounds[0], maxBounds[1], maxBounds[2]);
	}

	BinningGrid grid;
	grid.initialize(minBounds, maxBounds, voxelSize, preallocBins);

	for(int i = 0; i < inputFiles; i++)
	{
		PointCloud input;
		input.openInputFile(inputFile[i], FileFormat::fromString(inputFormat));
		
		// Find the total size of the input file. Compute how much we need to read for each node.
		long int size = input.getSizeInBytes();
		long int sizePerProc = size / NPROCS;
		input.setReadLength(sizePerProc);
		
		// Set the read offset for this node
		input.setReadOffset(sizePerProc * PID);
		
		while(!input.endOfFile())
		{
			input.readNext(inputBufferSize);
			grid.addPoints(input);
			input.getPoints().clear();
		}
		input.closeFile();
	}
	
	int xprocs, yprocs;
	
	// Find a good factorization of NPROC to handle regions.
	for(xprocs = (NPROCS / 2) - 1; xprocs > 1; xprocs--) if(NPROCS % xprocs == 0) break;
	if(xprocs < 1) xprocs = 1;
	yprocs = NPROCS / xprocs;
	
	float bx = (maxBounds[0] - minBounds[0]) / xprocs;
	float by = (maxBounds[1] - minBounds[1]) / yprocs;
	
	if(PID == 0) printf("Region mesh size: %dx%d = %d   bx = %f    by = %f\n", xprocs, yprocs, NPROCS, bx, by);
	
	std::vector<BinData> localBins;
	
	// Loop through regions (one per process)
	for(int i = 0; i < NPROCS; i++)
	{
		int rx = i % xprocs;
		int ry = i / xprocs;
		// Compute region bounds
		vmml::vec3d rmin(
			(bx * rx) + minBounds[0], 
			(by * ry) + minBounds[1], 
			minBounds[2]);

		vmml::vec3d rmax
			((bx * rx) + bx + minBounds[0], 
			(by * ry) + by + minBounds[1], 
			maxBounds[2]);
			
		/*printf("Region %dx%d bounds: %f %f %f >>> %f %f %f\n", 
			 rx, ry, 
			 rmin[0], rmin[1], rmin[2],
			 rmax[0], rmax[1], rmax[2]);*/
		
		// If I'm not processor i, i'm sending
		if(PID != i)
		{
			std::vector<BinData> bindata;
			grid.getBinDataRegion(rmin, rmax, bindata);
			
			unsigned int size = bindata.size();
			//printf("PID %d: bin region elements: %d\n", PID, size);
			
			double* pointData = NULL;
			int* pointCount = NULL;
			
			if(size != 0)
			{
				// Stores point and normal
				pointData = new double[size * 6];
				// Stores point count
				pointCount = new int[size * 6];
				
				// Copy data to the arrays.
				for(int j = 0; j < size; j++)
				{
					BinData& elem = bindata.at(j);
					pointData[j * 6] = elem.position[0];
					pointData[j * 6 + 1] = elem.position[1];
					pointData[j * 6 + 2] = elem.position[2];
					pointData[j * 6 + 3] = elem.normal[0];
					pointData[j * 6 + 4] = elem.normal[1];
					pointData[j * 6 + 5] = elem.normal[2];
					
					pointCount[j] = elem.numPoints;
				}
			}
			
			//printf("PID %d sending %d elements to %d\n", PID, size, i);
			// Send the number of elements.
			MPI_Send(&size, 1, MPI_UNSIGNED, i, 0, MPI_COMM_WORLD);
			
			if(size != 0)
			{
				// Send the point data
				MPI_Send(pointData, size * 6, MPI_DOUBLE, i, 0, MPI_COMM_WORLD);
				// Send the point count
				MPI_Send(pointCount, size, MPI_INT, i, 0, MPI_COMM_WORLD);
				
				delete[] pointData;
				delete[] pointCount;
			}
		}
		else
		{
			grid.getBinDataRegion(rmin, rmax, localBins);
			
			for(int j = 0; j < NPROCS; j++)
			{
				if(j != PID)
				{
					MPI_Status status;
					// Otherwise, I'm receiving.
					unsigned int size = 0;
					
					// Receive the number of elements
					MPI_Recv(&size, 1, MPI_UNSIGNED, j, 0, MPI_COMM_WORLD, &status);
					
					if(size != 0)
					{
						//printf("PID %d receiving %d elements from %d\n", PID, size, j);
						
						// Stores point and normal
						double* pointData = new double[size * 6];
						// Stores point count
						int* pointCount = new int[size];
						
						// Receive the point data
						MPI_Recv(pointData, size * 6, MPI_DOUBLE, j, 0, MPI_COMM_WORLD, &status);
						// Receive the point count
						MPI_Recv(pointCount, size, MPI_DOUBLE, j, 0, MPI_COMM_WORLD, &status);
						
						// Copy the data into the local bins.
						for(int k = 0; k < size; k++)
						{
							BinData bd;
							bd.position[0] = pointData[k * 6];
							bd.position[1] = pointData[k * 6 + 1];
							bd.position[2] = pointData[k * 6 + 2];
							bd.normal[0] = pointData[k * 6 + 3];
							bd.normal[1] = pointData[k * 6 + 4];
							bd.normal[2] = pointData[k * 6 + 5];
							bd.numPoints = pointCount[k];
							localBins.push_back(bd);
						}
						
						delete[] pointData;
						delete[] pointCount;
					}
				}
			}
		}
	}
	
	// Merge all local bins
	//fprintf(stderr, "PID %d: Merging bins... (total bins %d)\n", PID, localBins.size());
	BinningGrid localGrid;
	localGrid.initialize(minBounds, maxBounds, voxelSize, preallocBins);
	localGrid.addBins(localBins);
	
	//fprintf(stderr, "PID: %d Reading back point cloud...\n", PID);
	PointCloud cloud;
	localGrid.writeToPointCloud(&cloud, qualityThreshold);

	cloud.setNormalNeighbors(normalNeighbors);
	if(!cloud.hasNormals() && normalNeighbors != 0)
	{
		cloud.computeNormals();
	}

	// Write output
	for(int i = 0; i < NPROCS; i++)
	{
		if(i == PID)
		{
			//fprintf(stderr, "PID: %d Writing output (%d items) to %s...\n", PID, cloud.getPoints().size(), outputFile);
			
			bool append = i != 0;
			cloud.write(outputFile, FileFormat::fromString(outputFormat), append);
		}
		MPI_Barrier(MPI_COMM_WORLD);
	}
	
	float totalTime = (mstime() - startTime) / 1000.0f;
	if(PID == 0) printf("Execution time: %.2f seconds\n", totalTime);

	// Finalize MPI
	MPI_Finalize();
	
    return 0;
}

