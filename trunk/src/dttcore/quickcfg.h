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
#ifndef __QUICKCFG_H__
#define __QUICKCFG_H__

#include <stdlib.h>
#include <stdio.h>
#ifdef WIN32
#include <direct.h>
#else
#define _strdup strdup
#define _getcwd getcwd
#endif

#ifdef CFG_DEBUG
#define LOGMAP(name) printf("CONFIG: %s = %s\n", #name, &__str[strlen(#name) + 1]);
#define LOGMAP_ARRAY(name) printf("CONFIG: %s = %s\n", name, &__str[strlen(__name) + 1]);
#else
#define LOGMAP(name) 
#define LOGMAP_ARRAY(name) 
#endif

#define CFG_MAP_STRING(name) if(!strncmp(__str, #name, strlen(#name))) { LOGMAP(name); name = _strdup(&__str[strlen(#name) + 1]); }
#define CFG_MAP_STRING_ARRAY(name, i) sprintf(__name, "%s[%d]", #name, i); if(!strncmp(__str, __name, strlen(__name))) { LOGMAP_ARRAY(__name); name[i] = _strdup(&__str[strlen(__name) + 1]); }
#define CFG_MAP_INT(name)    if(!strncmp(__str, #name, strlen(#name))) { LOGMAP(name); name = atoi(  &__str[strlen(#name) + 1]); }
#define CFG_MAP_FLOAT(name)  if(!strncmp(__str, #name, strlen(#name))) { LOGMAP(name); name = (float)atof(  &__str[strlen(#name) + 1]); }
#define CFG_MAP_FLOAT_ARRAY(name, i) sprintf(__name, "%s[%d]", #name, i); if(!strncmp(__str, __name, strlen(__name))) { LOGMAP_ARRAY(__name); name[i] = (float)atof(&__str[strlen(__name) + 1]); }
#define CFG_MAP_INT_ARRAY(name, i) sprintf(__name, "%s[%d]", #name, i); if(!strncmp(__str, __name, strlen(__name))) { LOGMAP_ARRAY(__name); name[i] = atoi(&__str[strlen(__name) + 1]); }

#define CFG_START(file) \
	char __path[256]; \
	char __name[256]; \
	_getcwd(__path, 256); strcat(__path, "/"); strcat(__path, file); \
	FILE* __cfg = fopen(__path, "r"); \
	char __str[256]; \
	while(fgets(__str, 256, __cfg)) { char* __end =__str+strlen(__str)-1; if (*(__end-1) == '\r') __end--; *__end='\0';


#define CFG_END	} fclose(__cfg);

#define CFG_REC_SEC					  fprintf(__cfg, "\n");
#define CFG_REC_COMMENT(str)		  fprintf(__cfg, "# %s\n", str);
#define CFG_REC_COMMENT_FMT(fmt, ...) fprintf(__cfg, "# "); fprintf(__cfg, fmt, __VA_ARGS__); fprintf(__cfg, "\n");
#define CFG_REC_STRING(name)		  fprintf(__cfg, "%s %s\n", #name, name);
#define CFG_REC_STRING_ARRAY(name, i) fprintf(__cfg, "%s[%d] %s\n", #name, i, name[i]);
#define CFG_REC_INT(name)			  fprintf(__cfg, "%s %d\n", #name, name);
#define CFG_REC_INT_ARRAY(name, i) 	  fprintf(__cfg, "%s[%d] %d\n", #name, i, name[i]);
#define CFG_REC_FLOAT(name)			  fprintf(__cfg, "%s %g\n", #name, name);
#define CFG_REC_FLOAT_ARRAY(name, i)  fprintf(__cfg, "%s[%d] %g\n", #name, i, name[i]);

#define CFG_REC_START(file) \
	char __path[256]; \
	strncpy(__path, file, 252); strncat(__path, ".cfg", 256); \
	FILE* __cfg = fopen(__path, "w"); 

#define CFG_REC_END fclose(__cfg);

#endif
