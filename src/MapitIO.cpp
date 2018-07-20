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
	if (type == CC_TYPES::POINT_CLOUD || type == CC_TYPES::GBL_SENSOR)
	{
		multiple = true;
		exclusive = true;
		return true;
	}

	return false;
}

CC_FILE_ERROR MapitFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	// only one mapit repo can be loaded at once!!!
	if (cc_mapit_file_name_ != "" && cc_mapit_file_name_ != filename.toStdString()) {
		ccLog::Error(QString::fromStdString(
						 "Can only load one mapit repo with one CloudCompare instance\n"
						 "Currently loaded is: " + cc_mapit_file_name_
						 )
					);
		return CC_FERR_BAD_ARGUMENT;
	} else {
		cc_mapit_file_name_ = filename.toStdString();
	}

	// open cc-mapit file to read what of the repo should be loaded
	std::ifstream def_file(cc_mapit_file_name_);
	std::string line;
	enum FilePartToRead {
		WORKSPACE,
		FILES,
		FRAME_ID,
		UNKNOWN
	} file_part_to_be_read = FilePartToRead::UNKNOWN;
	while (std::getline(def_file, line)) {
		if (0 == line.compare("--workspace")) {
			file_part_to_be_read = FilePartToRead::WORKSPACE;
			continue;
		} else if (0 == line.compare("--files")){
			file_part_to_be_read = FilePartToRead::FILES;
			continue;
		} else if (0 == line.compare("--frame_id")) {
			file_part_to_be_read = FilePartToRead::FRAME_ID;
			continue;
		}

		if        (file_part_to_be_read == FilePartToRead::WORKSPACE) {
			name_workspace_ = line;
		} else if (file_part_to_be_read == FilePartToRead::FILES) {
			name_files_.push_back( line );
		} else if (file_part_to_be_read == FilePartToRead::FRAME_ID) {
			frame_id_ = line;
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

	workspace_ = repo->getWorkspace(name_workspace_);
	if (workspace_ == nullptr) {
		// error
		return CC_FERR_BAD_ARGUMENT;
	}
	ccHObject* container_new = new ccHObject();
	container_new->setName(QString::fromStdString( _PREFIX_WORKSPACE_ + name_workspace_ ));
	container_new->setEnabled(false);
	container_current->addChild(container_new);
	container_current = container_new;

	// go though whole workspace and load all selected files to CC
	workspace_->depthFirstSearch(
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
					  // check if this entity should be loaded to CC
					  bool found = false;
					  for (std::string file : name_files_) {
						  if (path.find(file) != std::string::npos) {
							  found = true;
							  break;
						  }
					  }
					  if (found) {
						  QString name = QString::fromStdString( path.substr(path.find_last_of("/") + 1, path.length()) );

						  ccHObject* container_new = new ccHObject();
						  container_new->setName(name);
						  container_new->setEnabled(false);

						  if ( 0 == obj->type().compare(PointcloudEntitydata::TYPENAME()) ) {
							  load_pointcloud(obj, path, container_new);

							  container_current->addChild(container_new);
						  } else {
							  ccLog::Print(QString::fromStdString(
													"Mapit entity " + path + " of type " + obj->type() + " is not supported"
													)
										  );
							  delete container_new;
						  }
					  }

					  return true;
				  }
				, depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
				);

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR
MapitFilter::load_pointcloud(std::shared_ptr<mapit::msgs::Entity> obj, const mapit::Path &path, ccHObject* container)
{
	std::shared_ptr<mapit::AbstractEntitydata> entity_data_abstract = workspace_->getEntitydataReadOnly(path);
	std::shared_ptr<PointcloudEntitydata> entity_data = std::static_pointer_cast<PointcloudEntitydata>(entity_data_abstract);

	std::shared_ptr<pcl::PCLPointCloud2> cloud_ptr_in = entity_data->getData();
	boost::shared_ptr<pcl::PCLPointCloud2> cloud_ptr = boost::make_shared<pcl::PCLPointCloud2>(*cloud_ptr_in);

	// TODO transform to frame_id_

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
			float* X = ccRot.getColumn(0);
			float* Y = ccRot.getColumn(1);
			float* Z = ccRot.getColumn(2);
			float* S = ccRot.getColumn(3);
			//Warning: Y and Z are inverted
			X[0] = 1.0f; X[1] = 0.0f; X[2] = 0.0f; X[3] = 0.0f;
			Y[0] = 0.0f; Y[1] = 1.0f; Y[2] = 0.0f; Y[3] = 0.0f;
			Z[0] = 0.0f; Z[1] = 0.0f; Z[2] = 1.0f; Z[3] = 0.0f;
			S[0] = 0.0f; S[1] = 0.0f; S[2] = 0.0f; S[2] = 1.0f;
		}

		ccGBLSensor* sensor = new ccGBLSensor;
		sensor->setName(QString::fromStdString(frame_id_));
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

CC_FILE_ERROR
MapitFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	// can only write to same mapit repo
	if (cc_mapit_file_name_ != filename.toStdString()) {
		ccLog::Error(QString::fromStdString(
						 "The data can only be stored in the mapit repo that was opend originaly\n"
						 "The opend mapit repo is: " + cc_mapit_file_name_
						 )
					 );
		return CC_FERR_BAD_ARGUMENT;
	}
	std::string type = "unknown";
	if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		type = "Pointcloud";
	} else if (entity->isA(CC_TYPES::GBL_SENSOR)) {
		type = "Transform";
	}
	// get complete mapit name by combining all parrents
	std::string mapit_entity_name = "";
	ccHObject* tmp = entity->getParent();
	while (tmp && 0 != tmp->getName().toStdString().compare(_PREFIX_WORKSPACE_ + name_workspace_)) {
		mapit_entity_name = "/" + tmp->getName().toStdString() + mapit_entity_name;
		tmp = tmp->getParent();
	}
	std::cout << "In workspace: " << name_workspace_ << std::endl
			  << "Store " << type << std::endl
			  << "Mapit name:       " << mapit_entity_name << std::endl
			  << " with name          " << entity->getName().toStdString() << std::endl
			  << "Parrent name:       " << entity->getParent()->getName().toStdString() << std::endl
			  << "Grandparrents name: " << entity->getParent()->getParent()->getName().toStdString() << std::endl;

	if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		// TODO
		return CC_FERR_NOT_IMPLEMENTED;
	} else if (entity->isA(CC_TYPES::GBL_SENSOR)) {
		return store_transform(entity, mapit_entity_name);
	} else {
		return CC_FERR_UNKNOWN_FILE;
	}
	return CC_FERR_NOT_IMPLEMENTED;
}

CC_FILE_ERROR
MapitFilter::store_transform(ccHObject* entity, std::string mapit_entity_name)
{
	return CC_FERR_NO_ERROR;
}
