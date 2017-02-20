/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Meißner Pascal, Reckling Reno, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
//Header include
#include <asr_ism_visualizations/ism_result_visualizer_rviz.hpp>

//Ilcas includes
#include <ISM/utility/GeometryHelper.hpp>
#include <ISM/utility/viz_helper.hpp>
#include <ISM/common_type/Pose.hpp>
#include <ISM/common_type/Object.hpp>
#include <ISM/common_type/RecognitionResult.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>
#include <asr_ism_visualizations/visualizer_rviz.hpp>

//ros includes
#include <ros/package.h>

//other includes
#include <algorithm>

namespace VIZ
{


void ISMResultVisualizerRVIZ::addVisualization(const ISM::RecognitionResultPtr recognition_result)
{

    visualization_msgs::MarkerArray current_marker_array = getMarkersFromResult(recognition_result);

    addMarker(current_marker_array);
    publishCollectedMarkers();
}

visualization_msgs::MarkerArray ISMResultVisualizerRVIZ::genTestMarker(){
    visualization_msgs::MarkerArray temp_marker;

    ISM::PosePtr id = ISM::PosePtr(new ISM::Pose(new ISM::Point(), new ISM::Quaternion()));
    std::vector<ISM::PosePtr> poses;
    poses.push_back(id);

    id->quat = (ISM::GeometryHelper::getQuatFromRPY(id->quat, 0, 0, 180));
    temp_marker = VizHelperRVIZ::createCoordinateMarkerWithAngle(id, "map", "test_marker", 1, 0.1, 0.01, 0);

    return temp_marker;
}

visualization_msgs::MarkerArray ISMResultVisualizerRVIZ::getMarkersFromResult(const ISM::RecognitionResultPtr result)
{
    visualization_msgs::MarkerArray ret_markers;
    ism_to_depth_map_.clear();
    ism_to_parent_ism_map_.clear();
    last_ref_pose_ = nullptr;
    z_offset_ = use_z_max_ ? -INFINITY : INFINITY;

    if (result->referencePose == nullptr)
    {
        return ret_markers;
    }

    std::vector<ISM::RecognitionResultPtr> ism_tree = traverseTree(result, 0);
    int max_depth = 0;
    for(std::pair<std::string, int> ism_to_depth_map_iter : ism_to_depth_map_){
        max_depth = std::max(max_depth, ism_to_depth_map_iter.second);
    }
    max_depth += 1;

    int i = 1;
    int j = 0;
    std::stringstream ns_stream;
    ns_stream << "ism_" << result->patternName << "_" << scene_iterator_ << "";
    std::string marker_namespace = ns_stream.str();
    Marker temp_marker;

    ref_pose_ = result->referencePose;
    for(ISM::RecognitionResultPtr leaf : ism_tree) {
        std::string leaf_pattern_name = leaf->patternName;
        int inverse_depth = max_depth - ism_to_depth_map_[leaf_pattern_name];
        float ref_scaling = (leaf_pattern_name == result->patternName) ? ism_marker_scale_ * 2 : ism_marker_scale_ / 2;
        ISM::PosePtr from_pose_ptr = getAdjustedPose(leaf->referencePose, inverse_depth, false);
        ISM::PointPtr from_point_ptr = from_pose_ptr->point;

        temp_marker = VizHelperRVIZ::createSphereMarker(from_point_ptr, base_frame_, marker_namespace, ++j, ref_scaling, VizHelperRVIZ::confidenceToColor(leaf->confidence), marker_lifetime_);
        ret_markers.markers.push_back(temp_marker);

        if(last_ref_pose_ != nullptr){
            //if ISM is not Root ISM
            from_pose_ptr->quat =(calculateOrientation(getAdjustedPose(ism_to_parent_ism_map_[leaf_pattern_name], inverse_depth +1, false)->point, from_point_ptr));
            temp_marker = VizHelperRVIZ::createConeMarker(from_pose_ptr, base_frame_, marker_namespace, ++j, ref_scaling * 0.9, VizHelperRVIZ::confidenceToColor(leaf->confidence), marker_lifetime_);
            ret_markers.markers.push_back(temp_marker);
        }

        ColorRGBA arrow_color = VizHelperRVIZ::createColorRGBA(0, 0, 0, 0);
        if(use_scene_coloring_){
            if(ism_to_color_map_.find(result->patternName) == ism_to_color_map_.end()){
                ism_to_color_map_.insert(std::make_pair(result->patternName, VizHelperRVIZ::hsvToRGBA(120 + (240.0 / (scene_count_ + 1)) * (ism_to_color_map_.size() + 1) , 1.0, 1.0)));
            }
            arrow_color = ism_to_color_map_[result->patternName];
        }else{
            arrow_color = VizHelperRVIZ::hsvToRGBA(120 + (240.0 / (ism_tree.size() + 1)) * i , 1.0, 1.0);
        }
        std::vector<ISM::PosePtr> to_poses;

        for(ISM::ObjectPtr object : leaf->recognizedSet->objects){

            to_poses.push_back(getAdjustedPose(object->pose, inverse_depth, true));

            if(ism_to_depth_map_.find(object->type) != ism_to_depth_map_.end()){
                continue;
            }

            if(ism_to_depth_map_.find(object->type + object->observedId) == ism_to_depth_map_.end()){
                ISM::PosePtr adjusted_pose_ptr = getAdjustedPose(object->pose, inverse_depth, true);


                //Meshmarker
                if(!object->ressourcePath.empty()){

                    Marker mesh_marker = VizHelperRVIZ::createMeshMarker(getAdjustedPose(object->pose, inverse_depth, true), base_frame_, marker_namespace, ++j, VizHelperRVIZ::createColorRGBA(ISM::getColorOfObject(object)), marker_lifetime_, object->ressourcePath.c_str());

                    mesh_marker.scale.x = 0.0001;
                    mesh_marker.scale.y = 0.0001;
                    mesh_marker.scale.z = 0.0001;
                    ret_markers.markers.push_back(mesh_marker);
                }

                //Cylinder
                double height = use_z_max_ ? std::abs(object->pose->point->eigen.z() - std::max(adjusted_pose_ptr->point->eigen.z(), z_offset_)) : std::abs(z_offset_ - std::min(object->pose->point->eigen.z(), adjusted_pose_ptr->point->eigen.z()));

                ISM::PosePtr cylinder_pose_ptr = calculateCylinderPose(object->pose, adjusted_pose_ptr->point, height);


                if(cylinder_map_.find(object->type + object->observedId) == cylinder_map_.end()){
                    cylinder_map_.insert(std::make_pair(object->type + object->observedId, std::make_pair(cylinder_pose_ptr, height)));
                }else{
                    if(cylinder_map_[object->type + object->observedId].second < height){
                        cylinder_map_[object->type + object->observedId] = std::make_pair(cylinder_pose_ptr, height);
                    }
                }

                //Cubearrow
                adjusted_pose_ptr->quat =(calculateOrientation(adjusted_pose_ptr->point, from_point_ptr, true));
                arrow_color.a = 0.5;
                MarkerArray cube_markers = VizHelperRVIZ::createCubeArrow(adjusted_pose_ptr, base_frame_, marker_namespace, ++j, ism_marker_scale_ / 5, arrow_color, VizHelperRVIZ::confidenceToColor(object->weight), marker_lifetime_);
                ret_markers.markers.insert(ret_markers.markers.end(), cube_markers.markers.begin(), cube_markers.markers.end());

            }
        }
        arrow_color.a = 1;
        temp_marker = VizHelperRVIZ::createLineArrow(from_point_ptr, to_poses, base_frame_, marker_namespace, ++j, line_scale_, arrow_color, marker_lifetime_);
        ret_markers.markers.push_back(temp_marker);
        last_ref_pose_ = from_point_ptr;
        i++;
    }
    scene_iterator_++;

    if(scene_count_ == scene_iterator_){

        int ii = 0;
        for(auto iter : cylinder_map_){
            Marker cylinder_marker = VizHelperRVIZ::createCylinderMarker(iter.second.first, base_frame_, "ism_cylinder", ii++, line_scale_ * 2, iter.second.second,
                                                                         VizHelperRVIZ::createColorRGBA(0.0, 0.0, 0.0, 0.25), marker_lifetime_);
            ret_markers.markers.push_back(cylinder_marker);
        }
    }
    return ret_markers;
}

ISM::PosePtr ISMResultVisualizerRVIZ::calculateCylinderPose(const ISM::PosePtr from_pose_ptr, const ISM::PointPtr to_point_ptr, double height){
    ISM::PosePtr ret_pose_ptr = ISM::PosePtr(new ISM::Pose());

    ret_pose_ptr->quat =(calculateOrientation(from_pose_ptr->point, to_point_ptr));
    ret_pose_ptr->point->eigen.z() =(std::min(z_offset_, from_pose_ptr->point->eigen.z()) + height / 2);
    ret_pose_ptr->point->eigen.y() = (from_pose_ptr->point->eigen.y());
    ret_pose_ptr->point->eigen.x() = (from_pose_ptr->point->eigen.x());

    return ret_pose_ptr;
}


ISM::QuaternionPtr ISMResultVisualizerRVIZ::calculateOrientation(ISM::PointPtr from_point_ptr, ISM::PointPtr to_point_ptr, bool pose_relative){

    //TODO use for cylinder
    pose_relative = false;

    double x = from_point_ptr->eigen.x() - to_point_ptr->eigen.x();
    double y = from_point_ptr->eigen.y() - to_point_ptr->eigen.y();
    double z = from_point_ptr->eigen.z() - to_point_ptr->eigen.z();

    double az = atan2(-x, -z);
    double alt = atan2(y, sqrt(x*x + z*z));

    if(z <= 0){
        alt = -alt;
        az = -az;
    }

    az = az < -M_PI / 2 || az > M_PI / 2 ? M_PI - az : az;
    alt = alt < -M_PI / 2 || alt > M_PI / 2 ? M_PI - alt : alt;


    Eigen::AngleAxisd rY(-az, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rX(-alt, Eigen::Vector3d::UnitX());

    return ISM::GeometryHelper::eigenQuatToQuat(rY * rX);
}


std::vector<ISM::RecognitionResultPtr> ISMResultVisualizerRVIZ::traverseTree(ISM::RecognitionResultPtr result, int depth){
    std::vector<ISM::RecognitionResultPtr> ism_tree;
    ism_tree.push_back(result);
    ism_to_depth_map_[result->patternName] = depth;

    for(ISM::ObjectPtr object : result->recognizedSet->objects){
        z_offset_ = use_z_max_? std::max(z_offset_, object->pose->point->eigen.z()) : std::min(z_offset_, object->pose->point->eigen.z());
    }

    depth++;
    for(ISM::RecognitionResultPtr subPattern : result->subPatterns) {
        ism_to_parent_ism_map_[subPattern->patternName] = result->referencePose;

        std::vector<ISM::RecognitionResultPtr> childTrees = traverseTree(subPattern, depth);
        ism_tree.insert(ism_tree.end(), childTrees.begin(), childTrees.end());
    }
    return ism_tree;
}

ISM::PosePtr ISMResultVisualizerRVIZ::getAdjustedPose(ISM::PosePtr pose, int depth, bool is_child)
{
    double delta_offset =  is_child ? -tree_depth_size_ / 2 : tree_depth_size_ / 2;
    ISM::PosePtr ret_pose = ISM::PosePtr(new ISM::Pose());
    ret_pose->quat =ISM::QuaternionPtr(pose->quat);

    double z_offset = ignore_z_offset_ ? z_offset_ : pose->point->eigen.z();

    if(is_pose_relative_){
        Eigen::Quaternion<double> rotation = ISM::GeometryHelper::quatToEigenQuat(ref_pose_->quat);
        rotation.normalize();
        Eigen::Vector3d translation(0.0, 0.0, (-depth * tree_depth_size_) - delta_offset);
        Eigen::Vector3d offset = rotation._transformVector(translation);

        const ISM::PointPtr ret_point_ptr = ISM::PointPtr(new ISM::Point(
                                                              pose->point->eigen.x() + offset[0],
                                                          pose->point->eigen.y() + offset[1] ,
                pose->point->eigen.z() + offset[2]));
        ret_pose->point = ret_point_ptr;
    }else{
        const ISM::PointPtr ret_point_ptr = ISM::PointPtr(new ISM::Point(
                                                              pose->point->eigen.x(),
                                                              pose->point->eigen.y(),
                                                              z_offset + depth * tree_depth_size_ + delta_offset));
        ret_pose->point = ret_point_ptr;
    }

    return ret_pose;
}
}


