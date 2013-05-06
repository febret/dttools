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
#include <sstream>
#include <fstream>

#include "PoseData.h"
#include "Utils.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PoseData::~PoseData()
{
	if(myPoses != NULL) delete[] myPoses;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PoseData::read(const std::string& filename, FileFormat::Enum format)
{
	DTT_ASSERT(format == FileFormat::CSVENDURANCEPose);

	char* data = (char*)Utils::loadFile(filename.c_str());

	if(data != NULL)
	{
		int k = 0;
		int lines = Utils::countStringLines(data) - 1;
		myPoses = new RangeDataPose[lines + 2];

		double values[7];

		char* line = data;
		char* nl = strchr(line, '\n');
		*nl = '\0';
		
		int i = 0;
		while(nl != NULL)
		{
			*nl = '\0';

			// Skip comment lines.
			if(line[0] != '#')
			{
				int size = Utils::extractValues(line, values);

				// Skip invalid & comment lines.
				if(size == 7)
				{
					myPoses[i].t = values[0];

					myPoses[i].orientation = vmml::vec3d(values[1], values[2], values[3]);
					myPoses[i].position = vmml::vec3d(values[4], values[5], values[6]);
				}
			} 
			else
			  {
				myPoses[i].t = 0.0;
				myPoses[i].orientation = 0.0; //vmml::vec3d(values[1], values[2], values[3]);
				myPoses[i].position = 0.0; //vmml::vec3d(values[4], values[5], values[6]);
			  }
			i++;

			// fetch next line.
			line = nl + 1;
			nl = strchr(line, '\n');
		}
		
		free(data);
		
		myLength = lines;
		return true;
	}
	return false;
}
