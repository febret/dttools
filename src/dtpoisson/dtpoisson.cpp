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
 * Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <stdarg.h>

#include "quickcfg.h"
#include "PointCloud.h"

// VCG lib stuff (used for mesh cleanup)
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/trimesh/base.h>
#include <vcg/complex/trimesh/clean.h>


#include "Time.h"
#include "MarchingCubes.h"
#include "Octree.h"
#include "SparseMatrix.h"
#include "CmdLineParser.h"
#include "FunctionData.h"
#include "PPolynomial.h"
#include "ply.h"
#include "MemoryUsage.h"
#include "MultiGridOctreeData.h"

#define SCALE 1.25
#define DEGREE 2

// Configuration parameters.
// Base configuration parameters
char* inputFile;
int inputFormat = 0;
char* outputFile;
char* outputFormat = "PLY";
int solverDepth;
float samplesPerNode;
float pointScale;
int cleanupPercentage = 10;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loadConfig(const char* filename)
{
	solverDepth = 8;
	samplesPerNode = 1;
	pointScale = 1.25;

	printf("loading config %s...\n", filename);

	CFG_START(filename);

	CFG_MAP_STRING(inputFile);
	//CFG_MAP_INT(inputFormat);

	CFG_MAP_STRING(outputFile);
	CFG_MAP_STRING(outputFormat);

	CFG_MAP_INT(solverDepth);
	CFG_MAP_FLOAT(samplesPerNode);
	CFG_MAP_FLOAT(pointScale);

	CFG_MAP_INT(cleanupPercentage);

	CFG_END;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void writePointCloud(CoredVectorMeshData* mesh, Point3D<float>& translate, const float& scale)
{
	fprintf(stderr, "Reading back point cloud...");

	PointCloud cloud;
	cloud.setHasNormals(true);

	// Allocate a temporary sonar point buffer.
	int nr_vertices=int(mesh->outOfCorePointCount()+mesh->inCorePoints.size());
	int nr_faces=mesh->triangleCount();
	SonarPoint* points = new SonarPoint[nr_vertices];

	int j = 0;

	// Read points from the mesh
	Point3D<float> p;
	for (unsigned int i = 0; i < mesh->inCorePoints.size(); i++)
	{
		p = mesh->inCorePoints[i];

		points[j].position[0] = p.coords[0] * scale + translate.coords[0];
		points[j].position[1] = p.coords[1] * scale + translate.coords[1];
		points[j].position[2] = p.coords[2] * scale + translate.coords[2];
		
		j++;
	}
	for (int i = 0; i < mesh->outOfCorePointCount(); i++)
	{
		mesh->nextOutOfCorePoint(p);

		points[j].position[0] = p.coords[0] * scale + translate.coords[0];
		points[j].position[1] = p.coords[1] * scale + translate.coords[1];
		points[j].position[2] = p.coords[2] * scale + translate.coords[2];
		
		j++;
	}  
	
	// Now loop through faces to compute point normals.
	TriangleIndex tIndex;
	int inCoreFlag;
	for (int i = 0; i < nr_faces; i++)
	{
		mesh->nextTriangle(tIndex,inCoreFlag);

		// Fix the point indices.
		if(!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) tIndex.idx[0] += mesh->inCorePoints.size();
		if(!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) tIndex.idx[1] += mesh->inCorePoints.size();
		if(!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) tIndex.idx[2] += mesh->inCorePoints.size();

		vmml::vec3f d1 = points[tIndex.idx[0]].position - points[tIndex.idx[1]].position;
		vmml::vec3f d2 = points[tIndex.idx[0]].position - points[tIndex.idx[2]].position;

		vmml::vec3f normal = d1.cross(d2);
		normal.normalize();

		for(int j = 0; j < 3; j++)
		{
			SonarPoint pt;
			pt.position = points[tIndex.idx[j]].position;
			pt.normal = normal;
			cloud.addPoint(pt);
		}
	}  

	delete points;

	vmml::vec3f minBounds = cloud.getMinBounds();
	vmml::vec3f maxBounds = cloud.getMaxBounds();
	printf("min bounds: %f, %f, %f\n", minBounds[0], minBounds[1], minBounds[2]);
	printf("max bounds: %f, %f, %f\n", maxBounds[0], maxBounds[1], maxBounds[2]);

	printf("Writing output (%d items)...\n", cloud.getPoints().size());
	cloud.write(outputFile, FileFormat::fromString(outputFormat));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyVertex;
class MyFace;
class MyMesh;
class MyUsedTypes: public vcg::UsedTypes<vcg::Use<MyVertex>::AsVertexType, vcg::Use<MyFace>::AsFaceType> {};
class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::VFAdj, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  >{};
class MyFace: public vcg::Face< MyUsedTypes, vcg::face::VFAdj, vcg::face::Normal3f, vcg::face::VertexRef, vcg::face::BitFlags> {};
class MyMesh: public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace> > {};
void cleanMesh(CoredVectorMeshData* mesh)
{
	MyMesh vcgMesh;

	/*std::vector<MyVertex> vertices;
	vertices.reserve(myData.size());

	for(int i = 0; i < myData.size(); i++)
	{
		MyVertex v;
		v.P().X() = myData[i].position[0];
		v.P().Y() = myData[i].position[1];
		v.P().Z() = myData[i].position[2];

		vertices.push_back(v);
	}*/

	fprintf(stderr, "Cleaning up surface (area percentage: %d)...\n", cleanupPercentage);
	float minCC = (float)cleanupPercentage;
	std::pair<int,int> delInfo= vcg::tri::Clean<MyMesh>::RemoveSmallConnectedComponentsDiameter(vcgMesh,minCC);
	fprintf(stderr, "Removed %2 connected components out of %1", delInfo.second, delInfo.first); 		

	// Copy back the normals.
	/*for(int i = 0; i < myData.size(); i++)
	{
		myData[i].normal[0] = vertices[i].N().X();
		myData[i].normal[1] = vertices[i].N().Y();
		myData[i].normal[2] = vertices[i].N().Z();
	}

	myHasNormals = true;*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc,char* argv[])
{
	loadConfig(argv[1]);

	double t;
	double tt=Time();
	Point3D<float> center;
	Real scale=1.0;
	Real isoValue=0;
	Octree<DEGREE> tree;
	PPolynomial<DEGREE> ReconstructionFunction=PPolynomial<DEGREE>::GaussianApproximation();

	center.coords[0]=center.coords[1]=center.coords[2]=0;
	
	TreeOctNode::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

	t=Time();
	int kernelDepth= solverDepth - 2;

	fprintf(stderr, "Setting function data...\n");
	tree.setFunctionData(ReconstructionFunction, solverDepth, 0, Real(1.0) / ( 1 << solverDepth));
	if(kernelDepth > solverDepth) return EXIT_FAILURE;


	bool binaryFile = inputFormat == 1 ? true : false;
	bool noResetSamples = false;
	bool confidence = false;
	bool noClipTree = false;
	int refine = 3;
	int solverDivide = 8;
	//int isoDivide = 0;
	int isoDivide = 8;

	tree.setTree(inputFile, solverDepth, binaryFile, kernelDepth, Real(samplesPerNode), pointScale, center ,scale,  noResetSamples, confidence);

	if(noClipTree)
	{
		fprintf(stderr, "Clipping tree...\n");
		tree.ClipTree();
	}

	fprintf(stderr, "Finalizing...\n");
	tree.finalize1(refine);
	tree.SetLaplacianWeights();
	tree.finalize2(refine);

	fprintf(stderr, "Solving linear system...\n");
	tree.LaplacianMatrixIteration(solverDivide);

	CoredVectorMeshData mesh;
	fprintf(stderr, "Computing iso value...\n");
	isoValue=tree.GetIsoValue();
	fprintf(stderr, "iso value: %e\n", isoValue);

	fprintf(stderr, "Running marching cubes...\n");
	if(isoDivide)
	{
		tree.GetMCIsoTriangles(isoValue, isoDivide, &mesh);
	}
	else
	{
		tree.GetMCIsoTriangles(isoValue, &mesh);
	}

	if(FileFormat::fromString(outputFormat) == FileFormat::PLY)
	{
		fprintf(stderr, "Saving mesh...\n");
		PlyWriteTriangles(outputFile, &mesh, PLY_BINARY_NATIVE, center, scale);
	}
	else
	{
		writePointCloud(&mesh, center, scale);
	}

	return 1;
}

