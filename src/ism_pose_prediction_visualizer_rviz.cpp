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
#include <asr_ism_visualizations/ism_pose_prediction_visualizer_rviz.hpp>

//Ilcas includes
#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include <ISM/utility/viz_helper.hpp>
#include <ISM/utility/GeometryHelper.hpp>

//ros includes
#include <ros/package.h>

//foreigen includes
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

//other includes
#include <vector>
#include <string>
#include <math.h>





namespace VIZ
{

void ISMPosePredictionVisualizerRVIZ::addVisualization(ISM::RecognitionResultPtr result, asr_msgs::AsrAttributedPointCloud attributed_point_cloud, bool valid_position_viz)
{

    if(attributed_point_cloud.elements.size() == 0 || result->referencePose == nullptr)
    {
        ROS_ERROR("No matching scene for prediction. Skipping visualization");
        return;
    }

    int id = 0;
    object_type_to_poses_map_.clear();
    id_to_object_name_map_.clear();
    dist_color_.clear();

    //Fill maps with data from attributed point cloud
    for (asr_msgs::AsrAttributedPoint attributed_point: attributed_point_cloud.elements)
    {
        ISM::PointPtr ism_point_ptr(new ISM::Point(attributed_point.pose.position.x,
                                                   attributed_point.pose.position.y,
                                                   attributed_point.pose.position.z));
        ISM::QuaternionPtr ism_quat_ptr(new ISM::Quaternion(attributed_point.pose.orientation.w,
                                                            attributed_point.pose.orientation.x,
                                                            attributed_point.pose.orientation.y,
                                                            attributed_point.pose.orientation.z));
        ISM::PosePtr ism_pose_ptr(new ISM::Pose(ism_point_ptr, ism_quat_ptr));

        ISM::Object oPtr(attributed_point.type, ism_pose_ptr, attributed_point.identifier, mesh_resource_paths_[attributed_point.type].string());

        if (object_type_to_poses_map_.find(oPtr) == object_type_to_poses_map_.end()){
            std::vector<ISM::PosePtr> temp_vector;
            object_type_to_poses_map_[oPtr] = temp_vector;
            id_to_object_name_map_[id] = oPtr;
            id++;
        }
        object_type_to_poses_map_[oPtr].push_back(ism_pose_ptr);

    }

    if(valid_position_viz){
        //Visualization of valid positions
        MarkerArray ma = generateValidPositions(result);
        addMarker(ma);
        publishCollectedMarkers();


    }else{
        // normal Pose Prediction
        // generate and publish prediction marker
        MarkerArray ma = generatePredictionMarker(result);
        addMarker(ma);
        publishCollectedMarkers();

        pose_counter_ = 0;
        object_counter_ = 0;

        update = true;
        updateObjectMarker();
    }

}
MarkerArray ISMPosePredictionVisualizerRVIZ::generateValidPositions(ISM::RecognitionResultPtr result){
    MarkerArray ret_marker;
    std_msgs::ColorRGBA vp_sphere_color;
    std_msgs::ColorRGBA vp_vector_color;
    std::string voter_namespace;
    int i = 1;

    typedef boost::variate_generator<boost::mt19937, boost::uniform_real<> > UniformDistributionGenerator;
    UniformDistributionGenerator* udg = new UniformDistributionGenerator(boost::mt19937(time(0)), boost::uniform_real<>(-sphere_radius_, sphere_radius_));

    std::vector<std::string> object_axis;
    boost::split(object_axis, object_axis_map_, boost::is_any_of(";"));

    std::map<std::string, int> axis_map;
    for(uint k = 0; k < object_axis.size(); k++){
        if(!object_axis[k].empty()){
            std::vector<std::string> temp_tokens;
            boost::split(temp_tokens, object_axis[k], boost::is_any_of("-"));
            if(!temp_tokens[0].empty() && !temp_tokens[1].empty()){
                int axis = 0;
                if(temp_tokens[1].compare("X") == 0) axis = 0;
                else if(temp_tokens[1].compare("Y") == 0) axis = 1;
                else if(temp_tokens[1].compare("Z") == 0) axis = 2;
                axis_map.insert(std::make_pair(temp_tokens[0], axis));
            }
        }
    }


    for(auto voter_to_pose_map_iter : object_type_to_poses_map_)
    {
        int id = 0;
        voter_namespace = "positions_of_" + voter_to_pose_map_iter.first.type;

        vp_sphere_color = VizHelperRVIZ::hsvToRGBA(120 + (240.0 / (object_type_to_poses_map_.size() + 1)) * i , 1.0, pose_prediction_value_);
        vp_sphere_color.a = pose_prediction_alpha_;
        vp_vector_color = VizHelperRVIZ::hsvToRGBA(120 + (240.0 / (object_type_to_poses_map_.size() + 1)) * i , 1.0, sample_value_);
        vp_vector_color.a = sample_alpha_;
        int axis = 0;
        if(axis_map.find(voter_to_pose_map_iter.first.type) != axis_map.end()){
            axis = axis_map[voter_to_pose_map_iter.first.type];
        }
        for(ISM::PosePtr pose : voter_to_pose_map_iter.second){
            ret_marker.markers.push_back(VizHelperRVIZ::createSphereMarker(pose->point , base_frame_, voter_namespace, id++, sphere_radius_ * 2, vp_sphere_color, marker_lifetime_));
            MarkerArray temp_marker = VizHelperRVIZ::createPositionVector(pose, base_frame_, voter_namespace, vp_vector_color, udg, object_sampels_, axis, id, sphere_radius_ * sample_scale_, max_angle_scale_ * sample_scale_, sphere_radius_, marker_lifetime_);
            id += object_sampels_*2;
            ret_marker.markers.insert(ret_marker.markers.end(), temp_marker.markers.begin(), temp_marker.markers.end());

        }
        i++;
    }
    return ret_marker;
}


void ISMPosePredictionVisualizerRVIZ::updateObjectMarker(){
    if(!update)
        return;
    if(id_to_object_name_map_.find(object_counter_) == id_to_object_name_map_.end()){
        // current Object not in map. skipping.
        return;
    }

    ISM::Object object_ptr = id_to_object_name_map_[object_counter_];
    ISM::PosePtr pose = object_type_to_poses_map_[object_ptr][pose_counter_];

    std::string marker_namespace = "selected_prediction_pose";


    boost::filesystem::path object_ressource_path = object_ptr.ressourcePath;

    ColorRGBA object_color = VizHelperRVIZ::getColorOfObject(object_ptr);

    // generate and publish object Markers
    Marker marker = VizHelperRVIZ::createMeshMarker(pose, base_frame_, marker_namespace, 1, object_color, marker_lifetime_, object_ressource_path.string());
    MarkerArray ring_marker;
    if(dist_color_.size() == 6){
        ring_marker = VizHelperRVIZ::createRingMarker(pose,base_frame_, "ring_marker", 0.1, dist_color_[0], dist_color_[1], dist_color_[2],
                dist_color_[3], dist_color_[4], dist_color_[5], marker_lifetime_);
    }else{
        ColorRGBA color_red = VizHelperRVIZ::createColorRGBA(1.0, 0.0, 0.0, 1.0);
        ring_marker = VizHelperRVIZ::createRingMarker(pose,base_frame_, "ring_marker", 0.1, color_red, color_red, color_red, color_red, color_red, color_red, marker_lifetime_);
    }


    addMarker(marker);
    addMarker(ring_marker);
    publishCollectedMarkers();
}

void ISMPosePredictionVisualizerRVIZ::calculateDistColor(ISM::ObjectPtr camera_object_ptr){
    if(object_type_to_poses_map_.find(id_to_object_name_map_[object_counter_]) != object_type_to_poses_map_.end()){
        // Maps well initialised, set color to red

        if(id_to_object_name_map_[object_counter_].type == camera_object_ptr->type){
            // given objects matches one object in Map, calculating absolute

            dist_color_.clear();
            ColorRGBA temp_dist_color;
            ISM::PointPtr point_ptr = object_type_to_poses_map_[id_to_object_name_map_[object_counter_]][pose_counter_]->point;
            ISM::PointPtr camera_point_ptr = camera_object_ptr->pose->point;
            ISM::QuaternionPtr quat_ptr = object_type_to_poses_map_[id_to_object_name_map_[object_counter_]][pose_counter_]->quat;
            ISM::QuaternionPtr camera_quat_ptr = camera_object_ptr->pose->quat;



            // calculate distanceColor with the fiven maximum distance

            temp_dist_color = VizHelperRVIZ::hsvToRGBA(120 - std::min(1.0, std::abs(point_ptr->eigen.x() - camera_point_ptr->eigen.x()) / max_distance_in_overlay_) * 120, 1.0, 1.0);
            dist_color_.push_back(temp_dist_color);
            temp_dist_color = VizHelperRVIZ::hsvToRGBA(120 - std::min(1.0, std::abs(point_ptr->eigen.z() - camera_point_ptr->eigen.z()) / max_distance_in_overlay_) * 120, 1.0, 1.0);
            dist_color_.push_back(temp_dist_color);
            temp_dist_color = VizHelperRVIZ::hsvToRGBA(120 - std::min(1.0, std::abs(point_ptr->eigen.y() - camera_point_ptr->eigen.y()) / max_distance_in_overlay_) * 120, 1.0, 1.0);
            dist_color_.push_back(temp_dist_color);

            Eigen::Quaternion<double> rot = ISM::GeometryHelper::quatToEigenQuat(quat_ptr);
            Eigen::Matrix3d euler_angle = rot.toRotationMatrix();
            Eigen::Vector3d vector = euler_angle.eulerAngles(2, 0, 2);

            Eigen::Quaternion<double> camera_rot = ISM::GeometryHelper::quatToEigenQuat(camera_quat_ptr);
            Eigen::Matrix3d camera_euler_angle = camera_rot.toRotationMatrix();
            Eigen::Vector3d camera_vector = camera_euler_angle.eulerAngles(2, 0, 2);


            temp_dist_color = VizHelperRVIZ::hsvToRGBA(120 - std::min(1.0, std::abs(vector[0] - camera_vector[0]) / max_distance_in_overlay_) * 120, 1.0, 1.0);
            dist_color_.push_back(temp_dist_color);
            temp_dist_color = VizHelperRVIZ::hsvToRGBA(120 - std::min(1.0, std::abs(vector[2] - camera_vector[2]) / max_distance_in_overlay_) * 120, 1.0, 1.0);
            dist_color_.push_back(temp_dist_color);
            temp_dist_color = VizHelperRVIZ::hsvToRGBA(120 - std::min(1.0, std::abs(vector[1] - camera_vector[1]) / max_distance_in_overlay_) * 120, 1.0, 1.0);
            dist_color_.push_back(temp_dist_color);
        }
        updateObjectMarker();
    }
}

MarkerArray ISMPosePredictionVisualizerRVIZ::generatePredictionMarker(ISM::RecognitionResultPtr result){
    MarkerArray ret_marker;
    MarkerArray temp_arrow_markers;
    std_msgs::ColorRGBA vote_color;
    std::string voter_namespace;

    int i = 1;
    for(auto iter: object_type_to_poses_map_)
    {
        voter_namespace = "prediction_of_" + iter.first.type;

        vote_color = VizHelperRVIZ::hsvToRGBA(120 + (240.0 / (object_type_to_poses_map_.size() + 1)) * i , 1.0, 1.0);
        vote_color.a = 0.5;
        std::vector<std::pair<ISM::PointPtr, std::vector<ISM::PosePtr>>> pose_vector;
        pose_vector.push_back(std::make_pair(result->referencePose->point, iter.second));
        temp_arrow_markers = VizHelperRVIZ::createCoordinateArrow(pose_vector, base_frame_, voter_namespace,
                                                                  line_scale_, axis_scale_, vote_color, marker_lifetime_);
        ret_marker.markers.insert(ret_marker.markers.end(), temp_arrow_markers.markers.begin(), temp_arrow_markers.markers.end());


        i++;
    }
    return ret_marker;
}

void ISMPosePredictionVisualizerRVIZ::nextObject(){
    if(object_type_to_poses_map_.size() <= 0)
        return;
    object_counter_ = (object_counter_ + 1 + object_type_to_poses_map_.size()) % object_type_to_poses_map_.size();
    pose_counter_ = 0;
    dist_color_.clear();
    updateObjectMarker();
}

void ISMPosePredictionVisualizerRVIZ::prevObject(){
    if(object_type_to_poses_map_.size() <= 0)
        return;
    object_counter_ = (object_counter_ - 1 + object_type_to_poses_map_.size()) % object_type_to_poses_map_.size();
    pose_counter_ = 0;
    dist_color_.clear();
    updateObjectMarker();
}

void ISMPosePredictionVisualizerRVIZ::nextPose(){
    if(object_type_to_poses_map_.size() <= 0)
        return;
    pose_counter_ = (pose_counter_ + 1 + object_type_to_poses_map_[id_to_object_name_map_[object_counter_]].size()) % object_type_to_poses_map_[id_to_object_name_map_[object_counter_]].size();
    updateObjectMarker();
}

void ISMPosePredictionVisualizerRVIZ::prevPose(){
    if(object_type_to_poses_map_.size() <= 0)
        return;
    pose_counter_ = (pose_counter_ - 1 + object_type_to_poses_map_[id_to_object_name_map_[object_counter_]].size()) % object_type_to_poses_map_[id_to_object_name_map_[object_counter_]].size();
    updateObjectMarker();
}
}
