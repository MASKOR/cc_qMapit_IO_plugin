//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                     Tobias Neumann                                     #
//#                                                                        #
//##########################################################################

#include "MapitIO.h"

//qCC_db
#include <ccImage.h>
#include <ccCameraSensor.h>
#include <ccLog.h>

//Qt
#include <QFileInfo>

//system
#include <stdio.h>
#include <assert.h>

//Max number of characters per line in an ASCII file
//TODO: use QFile instead!
const int MAX_ASCII_FILE_LINE_LENGTH = 4096;

bool MapitFilter::canLoadExtension(const QString& upperCaseExt) const
{
	return (upperCaseExt == "MAPIT");
}

bool MapitFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//export not supported
	return false;
}

CC_FILE_ERROR MapitFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
//	if (CheckForSpecialChars(filename))
//	{
//		ccLog::Warning(QString("[ICM] Input filename contains special characters. It might be rejected by the I/O filter..."));
//	}

//	//ouverture du fichier
//	FILE *fp = fopen(qPrintable(filename), "rt");
//	if (!fp)
//	{
//		return CC_FERR_READING;
//	}

//	//buffer
//	char line[MAX_ASCII_FILE_LINE_LENGTH];

//	//lecture du header
//	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
//	{
//		fclose(fp);
//		return CC_FERR_READING;
//	}

//	if (strncmp(line,"#CC_ICM_FILE",12)!=0)
//	{
//		fclose(fp);
//		return CC_FERR_WRONG_FILE_TYPE;
//	}

//	//on extrait le chemin relatif
//	QString path = QFileInfo(filename).absolutePath();

//	char cloudFileName[MAX_ASCII_FILE_LINE_LENGTH];
//	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
//	{
//		fclose(fp);
//		return CC_FERR_READING;
//	}
//	if (strncmp(line,"FILE_NAME=",10)!=0)
//	{
//		fclose(fp);
//		return CC_FERR_WRONG_FILE_TYPE;
//	}
//	sscanf(line,"FILE_NAME=%s",cloudFileName);

//	char subFileType[12];
//	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
//	{
//		fclose(fp);
//		return CC_FERR_READING;
//	}
//	if (strncmp(line,"FILE_TYPE=",10)!=0)
//	{
//		fclose(fp);
//		return CC_FERR_WRONG_FILE_TYPE;
//	}
//	sscanf(line,"FILE_TYPE=%s",subFileType);

//	FileIOFilter::Shared filter = FileIOFilter::FindBestFilterForExtension(subFileType);
//	if (!filter)
//	{
//		ccLog::Warning(QString("[ICM] No I/O filter found for loading file '%1' (type = '%2')").arg(cloudFileName,subFileType));
//		fclose(fp);
//		return CC_FERR_UNKNOWN_FILE;
//	}

//	//load the corresponding file (potentially containing several clouds)
//	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
//	ccHObject* entities = FileIOFilter::LoadFromFile(QString("%1/%2").arg(path,cloudFileName), parameters, filter, result);
//	if (!entities)
//	{
//		fclose(fp);
//		return CC_FERR_READING;
//	}

//	container.addChild(entities);

//	//chargement des images
//	if (!fgets(line, MAX_ASCII_FILE_LINE_LENGTH , fp))
//	{
//		ccLog::Error("[ICM] Read error (IMAGES_DESCRIPTOR)! No image loaded");
//		fclose(fp);
//		return CC_FERR_READING;
//	}
//	else
//	{
//		if (strncmp(line,"IMAGES_DESCRIPTOR=",18)!=0)
//		{
//			fclose(fp);
//			return CC_FERR_WRONG_FILE_TYPE;
//		}
//		char imagesDescriptorFileName[MAX_ASCII_FILE_LINE_LENGTH];
//		sscanf(line,"IMAGES_DESCRIPTOR=%s",imagesDescriptorFileName);
		
//		int n = LoadCalibratedImages(entities,path,imagesDescriptorFileName,entities->getBB_recursive());
//		ccLog::Print("[ICM] %i image(s) loaded ...",n);
//	}

//	fclose(fp);
	return CC_FERR_NO_ERROR;
}
