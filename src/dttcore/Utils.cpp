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
#include <fstream>

#include "Utils.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Utils::countLines(const char* filename)
{
	// Count the number of items.
	FILE* file = fopen(filename, "rb");
	if(file == NULL)
	{
		return 0;
	}
	int lines = 0;
	while(!feof(file))
	{
		char buf[65535];
		fgets(buf, 65535, file);
		lines++;
	}
	fclose(file);
	return lines;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Utils::extractValues(const char* str, double* values)
{
	int size = 0;
	char* arg = (char*)str;
	char* tok = NULL;
	if (str[0] == '#') return size;
	while((tok = strtok(arg, ", ")) != NULL)
	{
		arg = NULL;
		values[size] = atof(tok);
		size++;
	}

	return size;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void* Utils::loadFile(const char* filename, int start, int size)
{
	// Count the number of items.
	FILE* file = fopen(filename, "rb");
	if(file == NULL)
	{
		fprintf(stderr, "Could not open file \"%s\".\n", filename);
		return NULL;
	}

	fseek(file, 0, SEEK_END);
	long length = ftell(file);

	// If size is specified and is a valid value, use that. Otherwise, read from <start> to end of file.
	if(size != -1 && start + size < length)
	{
		length = size;
	}
	else
	{
		if(size > 0)
		{
			length = length - size;
		}
	}

	void* data = malloc(length);
	fprintf(stderr, "%s: reading (%d Kb)...\n", filename, (length / 1024));
	fseek(file, start, SEEK_SET);
	fread(data, 1, length, file);
	
	fclose(file);

	return data;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Utils::countStringLines(const char* str)
{
	int lines = 0, i = 0;
	while(str[i] != '\0') if(str[i++] == '\n') lines++;
    return lines;
}

