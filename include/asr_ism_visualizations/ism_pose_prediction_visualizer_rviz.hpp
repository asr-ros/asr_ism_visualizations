/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Meißner Pascal, Reckling Reno, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once
//Ilcas includes
#include <asr_ism_visualizations/visualizer_rviz.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>
#include <ISM/common_type/RecognitionResult.hpp>
#include <asr_msgs/AsrAttributedPointCloud.h>
#include <asr_msgs/AsrAttributedPoint.h>

//ros includes
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <asr_ism_visualizations/ism_pose_prediction_visualizerConfig.h>
#include <asr_ism_visualizations/ism_valid_position_visualizerConfig.h>

namespace VIZ
{


class ISMPosePredictionVisualizerRVIZ : public VisualizerRVIZ
{
public:
    ISMPosePredictionVisualizerRVIZ(const ros::Publisher& publisher, double bin_size, double max_projection_angle_deviation, const std::map<std::string, boost::filesystem::path> type_to_ressource_path_map, const ros::NodeHandle& nh, const ros::NodeHandle& nh_vp)
        : VisualizerRVIZ(publisher), mesh_resource_paths_(type_to_ressource_path_map){

        sphere_radius_ = std::sqrt(3.0) * bin_size / 2;
        max_angle_scale_ = std::tan((max_projection_angle_deviation / 2) * (M_PI / 180)) * 2 * sphere_radius_;

        reconfigure_server_ = new dynamic_reconfigure::Server<asr_ism_visualizations::ism_pose_prediction_visualizerConfig>(nh);
        dynamic_reconfigure::Server<asr_ism_visualizations::ism_pose_prediction_visualizerConfig>::CallbackType reconf_callback = boost::bind(&ISMPosePredictionVisualizerRVIZ::dynamicReconfCallback, this, _1, _2);
        reconfigure_server_->setCallback(reconf_callback);

        reconfigure_server_vp_ = new dynamic_reconfigure::Server<asr_ism_visualizations::ism_valid_position_visualizerConfig>(nh_vp);
        dynamic_reconfigure::Server<asr_ism_visualizations::ism_valid_position_visualizerConfig>::CallbackType reconf_callback_vp = boost::bind(&ISMPosePredictionVisualizerRVIZ::dynamicReconfCallbackVP, this, _1, _2);
        reconfigure_server_vp_->setCallback(reconf_callback_vp);
    }

    ~ISMPosePredictionVisualizerRVIZ() {

    }

    void releaseCallback() {
        reconfigure_server_->clearCallback();
        reconfigure_server_vp_->clearCallback();
        delete reconfigure_server_;
        delete reconfigure_server_vp_;
    }

    void dynamicReconfCallback(asr_ism_visualizations::ism_pose_prediction_visualizerConfig &config, uint32_t level) {
        ROS_DEBUG_STREAM("Pose prediction visualizer dyn-params updated.");

        line_scale_ = config.lineScale;
        marker_lifetime_ = config.markerLifetime;
        base_frame_ = config.baseFrame;
        axis_scale_ = config.axisScale;
        max_distance_in_overlay_ = config.maxDistanceInOverlay;
        max_angle_in_overlay_ = config.maxAngleInOverlay;
    }

    void dynamicReconfCallbackVP(asr_ism_visualizations::ism_valid_position_visualizerConfig &config, uint32_t level) {
        ROS_DEBUG_STREAM("Valid position visualizer dyn-params updated.");

        object_axis_map_ = config.objectAxisMap;
        object_sampels_ = config.objectSampels;
        sample_value_ = config.sampleValue;
        sample_alpha_ = config.sampleAlpha;
        pose_prediction_alpha_ = config.posePredictionAlpha;
        pose_prediction_value_ = config.posePredictionValue;
        sample_scale_ = config.sampelScale;
    }

    void setGlobalParams(double marker_lifetime, std::string base_frame){
        marker_lifetime_ = marker_lifetime;
        base_frame_ = base_frame;
    }

    /// generates and publish prediction markers
    /// @param result result of the given attributed point cloud
    /// @param attributedPointCloud attributedPointCloud calculated in posepredictin ism
    void addVisualization(ISM::RecognitionResultPtr result, asr_msgs::AsrAttributedPointCloud attributed_point_cloud, bool valid_position_viz);

    /// calculates distance color
    /// @param cameraObjectPtr pose and type of recognized object
    void calculateDistColor(ISM::ObjectPtr camera_object_ptr);


    /// increments object counter and uptates object marker
    void nextObject();
    void prevObject();

    /// increments posecounter and uptates object marker
    void nextPose();
    void prevPose();

    ///deletes all markers of topic and sets update to false
    void clearAllMarkerOfTopic(){
        VisualizerRVIZ::clearAllMarkerOfTopic();
        update = false;
    }

private:

    /// generates prediction markers
    MarkerArray generatePredictionMarker(ISM::RecognitionResultPtr result);
    /// generates markers for validPositionSpace visualization
    MarkerArray generateValidPositions(ISM::RecognitionResultPtr result);

    /// updates marker of selected object
    void updateObjectMarker();

    ///PP stuff
    std::string base_frame_;
    double marker_lifetime_;
    double axis_scale_;
    double line_scale_;
    double max_distance_in_overlay_;
    double max_angle_in_overlay_;

    double sample_scale_;

    int object_counter_;
    int pose_counter_;
    bool update = false;

    ///VP stuff
    std::string object_axis_map_;
    int object_sampels_;
    double sample_value_;
    double sample_alpha_;
    double pose_prediction_alpha_;
    double pose_prediction_value_;
    double sphere_radius_;
    double max_angle_scale_;



    std::map<std::string, boost::filesystem::path> mesh_resource_paths_;
    std::map<ISM::Object, std::vector<ISM::PosePtr>> object_type_to_poses_map_;
    std::map<int, ISM::Object> id_to_object_name_map_;
    //std::map<std::string, std::string> object_to_observed_id_map_;
    std::vector<ColorRGBA> dist_color_;
    dynamic_reconfigure::Server<asr_ism_visualizations::ism_pose_prediction_visualizerConfig>* reconfigure_server_;
    dynamic_reconfigure::Server<asr_ism_visualizations::ism_valid_position_visualizerConfig>* reconfigure_server_vp_;

};
typedef boost::shared_ptr<ISMPosePredictionVisualizerRVIZ> ISMPosePredictionVisualizerRVIZPtr;

}
