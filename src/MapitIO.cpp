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

//mapit transform
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */
void
transformPointCloud(const Eigen::Matrix4f &transform, const pcl::PCLPointCloud2 &in,
					pcl::PCLPointCloud2 &out)
{
	// Get X-Y-Z indices
	int x_idx = pcl::getFieldIndex (in, "x");
	int y_idx = pcl::getFieldIndex (in, "y");
	int z_idx = pcl::getFieldIndex (in, "z");

	if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
		log_error("pointcloud2: Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
		throw MAPIT_STATUS_INVALID_DATA;
	}

	if (in.fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
		in.fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
		in.fields[z_idx].datatype != pcl::PCLPointField::FLOAT32) {
		log_error("pointcloud2: X-Y-Z coordinates not floats. Currently only floats are supported.");
		throw MAPIT_STATUS_INVALID_DATA;
	}

	// Check if distance is available
	int dist_idx = pcl::getFieldIndex(in, "distance");

	// Copy the other data
	if (&in != &out) {
		out.header = in.header;
		out.height = in.height;
		out.width  = in.width;
		out.fields = in.fields;
		out.is_bigendian = in.is_bigendian;
		out.point_step   = in.point_step;
		out.row_step     = in.row_step;
		out.is_dense     = in.is_dense;
		out.data.resize (in.data.size ());
		// Copy everything as it's faster than copying individual elements
		memcpy(&out.data[0], &in.data[0], in.data.size ());
	}

	Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

	for (size_t i = 0; i < in.width * in.height; ++i) {
		Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
		Eigen::Vector4f pt_out;

		bool max_range_point = false;
		int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
		float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
		if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2])) {
			if (distance_ptr==NULL || !std::isfinite(*distance_ptr)) { // Invalid point
				pt_out = pt;
			}
			else { // max range point
				pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
				pt_out = transform * pt;
				max_range_point = true;
				//std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
			}
		}
		else {
			pt_out = transform * pt;
		}

		if (max_range_point) {
			// Save x value in distance again
			*(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
			pt_out[0] = std::numeric_limits<float>::quiet_NaN();
		}

		memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
		memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
		memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));

		xyz_offset += in.point_step;
	}

	// Check if the viewpoint information is present
	int vp_idx = pcl::getFieldIndex (in, "vp_x");
	if (vp_idx != -1) {
		// Transform the viewpoint info too
		for (size_t i = 0; i < out.width * out.height; ++i)
		{
			float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
			// Assume vp_x, vp_y, vp_z are consecutive
			Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
			Eigen::Vector4f vp_out = transform * vp_in;

			pstep[0] = vp_out[0];
			pstep[1] = vp_out[1];
			pstep[2] = vp_out[2];
		}
	}
}

void
transformPointCloud(const mapit::tf::TransformStamped& transform, const pcl::PCLPointCloud2& in,
					pcl::PCLPointCloud2& out)
{
	Eigen::Affine3f mat;
	mat.matrix().block<3, 3>(0, 0) = transform.transform.rotation.toRotationMatrix();
	mat.matrix()(0, 3) = transform.transform.translation.x();
	mat.matrix()(1, 3) = transform.transform.translation.y();
	mat.matrix()(2, 3) = transform.transform.translation.z();
	transformPointCloud(mat.matrix(), in, out);
}

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
	mapit::tf2::BufferCore tf_buffer(workspace_.get(), "/tf/");
	mapit::tf::TransformStamped tf;
	try {
		tf = tf_buffer.lookupTransform(frame_id_, obj->frame_id(), mapit::time::from_msg(obj->stamp()));
	} catch (...) {
		ccLog::Error(QString::fromStdString(
						 "Can't lookup transform " + frame_id_ + " -> " + obj->frame_id()
						 + " at time " + mapit::time::to_string( mapit::time::from_msg(obj->stamp()) )
						 )
					 );
		return CC_FERR_BAD_ARGUMENT;
	}
	transformPointCloud(tf, *cloud_ptr_in, *cloud_ptr);


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
	std::cout << "Store " << type << " for: " << mapit_entity_name << std::endl;

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
	if (entity->getName().toStdString() == frame_id_) {
		ccLog::Error(QString::fromStdString(
						 "Entity name is same as frame_id from cc-mapit file\n"
						 "Transforms are created from <entity name> -> <frame_id from cc-mapit>"
						 )
					 );
		return CC_FERR_BAD_ARGUMENT;
	}

	std::string frame_id = entity->getName().toStdString();
	std::string child_frame_id = frame_id_;

	// get the header information of the entity that the transform belongs to
	std::shared_ptr<mapit::msgs::Entity> mapit_entity = workspace_->getEntity(mapit_entity_name);
	if ( ! mapit_entity) {
		// TODO error => mapit repo not as expected
		return CC_FERR_MALFORMED_FILE;
	}

	// get the rotation
	ccGBLSensor* cc_sensor = static_cast<ccGBLSensor*>(entity);
	ccGLMatrix cc_matrix = cc_sensor->getRigidTransformation();

	float* R1 = cc_matrix.getColumn(0);
	float* R2 = cc_matrix.getColumn(1);
	float* R3 = cc_matrix.getColumn(2);
	float* R4 = cc_matrix.getColumn(3);

	Eigen::Matrix3f rot_matrix;
	rot_matrix(0,0) = R1[0]; rot_matrix(0,1) = R2[0]; rot_matrix(0,2) = R3[0];
	rot_matrix(1,0) = R1[1]; rot_matrix(1,1) = R2[1]; rot_matrix(1,2) = R3[1];
	rot_matrix(2,0) = R1[2]; rot_matrix(2,1) = R2[2]; rot_matrix(2,2) = R3[2];

	Eigen::Quaternionf rot_quaternion(rot_matrix);

	// execute the operator to store the transform
	mapit::msgs::OperationDescription desc;
	desc.mutable_operator_()->set_operatorname("load_tfs");
	desc.set_params("{"
					"	  \"prefix\" : \"tf\""
					"	, \"transforms\" : ["
					"			{"
					"				  \"static\" : false"
					"				, \"header\" : {"
					"					  \"frame_id\" : \"" + frame_id + "\"" +
					"					, \"stamp\" : {"
					"						  \"sec\" : " + std::to_string( mapit_entity->stamp().sec() ) +
					"						, \"nsec\" : "  + std::to_string( mapit_entity->stamp().nsec() ) +
					"					  }"
					"				  }"
					"				, \"transform\" : {"
					"					  \"child_frame_id\" : \"" + child_frame_id + "\"" +
					"					, \"translation\" : {"
					"						  \"x\" : " + std::to_string( R4[0] ) +
					"						, \"y\" : " + std::to_string( R4[1] ) +
					"						, \"z\" : " + std::to_string( R4[2] ) +
					"					  }"
					"					, \"rotation\" : {"
					"						  \"w\" : " + std::to_string( rot_quaternion.w() ) +
					"						, \"x\" : " + std::to_string( rot_quaternion.x() ) +
					"						, \"y\" : " + std::to_string( rot_quaternion.y() ) +
					"						, \"z\" : " + std::to_string( rot_quaternion.z() ) +
					"					  }"
					"				  }"
					"			}"
					"		]"
					"}"
					);
	ccLog::Print(QString::fromStdString(
					 "Execute mapit operator " + desc.operator_().operatorname() + " with params:\n"
					 + desc.params()
					 )
				 );
	workspace_->doOperation(desc);

	return CC_FERR_NO_ERROR;
}
