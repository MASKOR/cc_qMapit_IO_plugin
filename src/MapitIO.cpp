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
//#                     CloudCompare project                               #
//#                     Tobias Neumann                                     #
//#                                                                        #
//##########################################################################

#include "MapitIO.h"

//qCC_db
#include <ccImage.h>
#include <ccCameraSensor.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>

//qPCL
#include <sm2cc.h>
#include <pcl/filters/passthrough.h>

//Qt
#include <QFileInfo>

//Boost
#include <boost/make_shared.hpp>

//system
#include <stdio.h>
#include <assert.h>
#include <fstream>

//mapit
#include <mapit/versioning/repositoryfactorystandard.h>
#include <mapit/depthfirstsearch.h>
#include <mapit/layertypes/pointcloudlayer.h>

//typedef pcl::PCLPointCloud2 PCLCloud;

//Max number of characters per line in an ASCII file
//TODO: use QFile instead!
const int MAX_ASCII_FILE_LINE_LENGTH = 4096;

bool MapitFilter::canLoadExtension(const QString& upperCaseExt) const
{
	return (upperCaseExt == "CC-MAPIT");
}

bool MapitFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//export not supported
	return false;
}

CC_FILE_ERROR MapitFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	// open cc-mapit file to read what of the repo should be loaded
	std::ifstream def_file(filename.toStdString());
	std::vector<std::string> files_to_load;
	std::string workspace_to_load;
	std::string line;
	bool read_workspace = true;
	while (std::getline(def_file, line)) {
		if (0 == line.compare("--workspace")) {
			read_workspace = true;
			continue;
		} else if (0 == line.compare("--files")){
			read_workspace = false;
			continue;
		}

		if (read_workspace) {
			workspace_to_load = line;
		} else {
			files_to_load.push_back( line );
		}
	}

	// open the mapit repo within the cc-mapit file is located
	std::string filename_std = filename.toStdString();
	if ( filename_std.find("/") == std::string::npos ) {
		return CC_FERR_UNKNOWN_FILE;
	}
	std::string repo_path = filename_std.substr(0, filename_std.find_last_of("/"));
	std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepositorySimple( repo_path, false ) );

	ccHObject* container_current = &container;

	std::shared_ptr<mapit::Workspace> workspace = repo->getWorkspace(workspace_to_load);
	if (workspace == nullptr) {
		// error
		return CC_FERR_BAD_ARGUMENT;
	}
	ccHObject* container_new = new ccHObject();
	container_new->setName(QString::fromStdString( "workspace_"+workspace_to_load ));
	container_new->setEnabled(false);
	container_current->addChild(container_new);
	container_current = container_new;

	// go though whole workspace and load all selected files to CC
	workspace->depthFirstSearch(
				  [&](std::shared_ptr<mapit::msgs::Tree> obj, const mapit::msgs::ObjectReference& ref, const mapit::Path &path) {
					  if (0 == path.compare("")) {
						  return true;
					  }

					  QString name = QString::fromStdString( path.substr(path.find_last_of("/") + 1, path.length()) );

					  // create new container (as layer)
					  ccHObject* container_new = new ccHObject();
					  container_new->setName(name);
					  container_new->setEnabled(false);
					  container_current->addChild(container_new);

					  container_current = container_new;

					  return true;
				  }

				, [&](std::shared_ptr<mapit::msgs::Tree> obj, const mapit::msgs::ObjectReference& ref, const mapit::Path &path) {
					  container_current = container_current->getParent();

					  return true;
				  }
				, [&](std::shared_ptr<mapit::msgs::Entity> obj, const mapit::msgs::ObjectReference& ref, const mapit::Path &path) {
					  bool found = false;
					  for (std::string file : files_to_load) {
						  if (path.find(file) != std::string::npos) {
							  found = true;
							  break;
						  }
					  }
					  if (!found) {
						  // stop this function here
						  return true;
					  }

					  QString name = QString::fromStdString( path.substr(path.find_last_of("/") + 1, path.length()) );

					  ccHObject* container_new = new ccHObject();
					  container_new->setName(name);
					  container_new->setEnabled(false);

					  if ( 0 == obj->type().compare(PointcloudEntitydata::TYPENAME()) ) {
						  load_pointcloud(workspace, obj, path, container_new);

						  container_current->addChild(container_new);
					  } else {
						  std::cout << "Mapit entity " << path << " of type " << obj->type() << " is not supported" << std::endl;
						  delete container_new;
					  }

					  return true;
				  }
				, depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
				);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR
MapitFilter::load_pointcloud(std::shared_ptr<mapit::Workspace> workspace, std::shared_ptr<mapit::msgs::Entity> obj, const mapit::Path &path, ccHObject* container)
{
	std::shared_ptr<mapit::AbstractEntitydata> entity_data_abstract = workspace->getEntitydataReadOnly(path);
	std::shared_ptr<PointcloudEntitydata> entity_data = std::static_pointer_cast<PointcloudEntitydata>(entity_data_abstract);

	std::shared_ptr<pcl::PCLPointCloud2> cloud_ptr_in = entity_data->getData();
	boost::shared_ptr<pcl::PCLPointCloud2> cloud_ptr = boost::make_shared<pcl::PCLPointCloud2>(*cloud_ptr_in);

	Eigen::Vector4f origin;
	Eigen::Quaternionf orientation;

//	PCLCloud::Ptr cloud_ptr;
//	if (!cloud_ptr_in->is_dense) //data may contain NaNs --> remove them
//	{
//		//now we need to remove NaNs
//		pcl::PassThrough<PCLCloud> passFilter;
//		passFilter.setInputCloud(cloud_ptr_in);

//		cloud_ptr = PCLCloud::Ptr(new PCLCloud);
//		passFilter.filter(*cloud_ptr);
//	}
//	else
//	{
//		cloud_ptr = boost::shared_ptr<pcl::PCLPointCloud2>(*cloud_ptr_in);
//	}

	//convert to CC cloud
	ccPointCloud* ccCloud = sm2ccConverter(cloud_ptr).getCloud();
	if (!ccCloud)
	{
		ccLog::Warning("[PCL] An error occurred while converting PCD cloud to CloudCompare cloud!");
		return CC_FERR_CONSOLE_ERROR;
	}

	//now we construct a ccGBLSensor
	{
		// get orientation as rot matrix and copy it into a ccGLMatrix
		ccGLMatrix ccRot;
		{
			Eigen::Matrix3f eigrot = orientation.toRotationMatrix();
			float* X = ccRot.getColumn(0);
			float* Y = ccRot.getColumn(1);
			float* Z = ccRot.getColumn(2);
			//Warning: Y and Z are inverted
			X[0] =  eigrot(0,0); X[1] =  eigrot(1,0); X[2] =  eigrot(2,0);
			Y[0] = -eigrot(0,2); Y[1] = -eigrot(1,2); Y[2] = -eigrot(2,2);
			Z[0] =  eigrot(0,1); Z[1] =  eigrot(1,1); Z[2] =  eigrot(2,1);

			ccRot.getColumn(3)[3] = 1.0f;
			ccRot.setTranslation(origin.data());
		}

		ccGBLSensor* sensor = new ccGBLSensor;
		sensor->setRigidTransformation(ccRot);
		sensor->setYawStep(static_cast<PointCoordinateType>(0.05));
		sensor->setPitchStep(static_cast<PointCoordinateType>(0.05));
		sensor->setVisible(false);
		//uncertainty to some default
		sensor->setUncertainty(static_cast<PointCoordinateType>(0.01));
		//graphic scale
		sensor->setGraphicScale(ccCloud->getOwnBB().getDiagNorm() / 10);

		//Compute parameters
		ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(ccCloud);
		sensor->computeAutoParameters(pc);

		sensor->setEnabled(false);

		ccCloud->addChild(sensor);
	}

	container->addChild(ccCloud);
}
