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
#include <ISM/typedef.hpp>

//ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <asr_ism_visualizations/ism_result_visualizerConfig.h>

namespace VIZ
{
class ISMResultVisualizerRVIZ : public VisualizerRVIZ
{

public:
    ISMResultVisualizerRVIZ(const ros::Publisher& publisher, const ros::NodeHandle& nh) :  VisualizerRVIZ(publisher) {
        reconfigure_server_ = new dynamic_reconfigure::Server<asr_ism_visualizations::ism_result_visualizerConfig>(nh);
        dynamic_reconfigure::Server<asr_ism_visualizations::ism_result_visualizerConfig>::CallbackType reconf_callback = boost::bind(&ISMResultVisualizerRVIZ::dynamicReconfCallback, this, _1, _2);
        reconfigure_server_->setCallback(reconf_callback);
    }

    ~ISMResultVisualizerRVIZ() {
        reconfigure_server_->clearCallback();
        delete reconfigure_server_;
    }


    void dynamicReconfCallback(asr_ism_visualizations::ism_result_visualizerConfig &config, uint32_t level) {
        ROS_DEBUG_STREAM("Result vizualizer dyn-params updated.");

        line_scale_ = config.lineScale;
        tree_depth_size_ = config.treeDepthSize;
        ism_marker_scale_ = config.ismMarkerScale;
        is_pose_relative_ = config.isPoseRelative;
        ignore_z_offset_ = config.ignoreZOffset;
        use_scene_coloring_ = config.useSceneColoring;
        use_z_max_ = config.ignoreZOffset;
        marker_lifetime_ = config.markerLifetime;
        base_frame_ = config.baseFrame;
    }

    void setSceneCount(int scene_count){
        scene_count_ = scene_count;
        scene_iterator_ = 0;

        ism_to_color_map_.clear();
        cylinder_map_.clear();
    }

    void addVisualization(const ISM::RecognitionResultPtr recognition_result);

private:
    visualization_msgs::MarkerArray genTestMarker();

    std::vector<ISM::RecognitionResultPtr> traverseTree(ISM::RecognitionResultPtr result, int depth);

    visualization_msgs::MarkerArray getMarkersFromResult(const ISM::RecognitionResultPtr result);

    ///Helper method to translate a point. So we can see the tree depth objects residing in trough their corresponding marker
    ISM::PosePtr getAdjustedPose(ISM::PosePtr pose, int depth, bool is_child);
    ISM::QuaternionPtr calculateOrientation(ISM::PointPtr from_point_ptr, ISM::PointPtr to_point_ptr, bool pose_relative = false);
    ISM::PosePtr calculateCylinderPose(const ISM::PosePtr from_pose_ptr, const ISM::PointPtr to_point_ptr, double height);

    bool is_pose_relative_;
    bool ignore_z_offset_;
    double tree_depth_size_;
    double line_scale_;
    double ism_marker_scale_;
    std::string base_frame_;
    double marker_lifetime_;

    std::map<std::string, int> ism_to_depth_map_;
    std::map<std::string, ISM::PosePtr> ism_to_parent_ism_map_;
    std::map<std::string, VIZ::ColorRGBA> ism_to_color_map_;
    std::map<std::string, std::pair<ISM::PosePtr, double>> cylinder_map_;

    double z_offset_;
    bool use_z_max_;
    bool use_scene_coloring_;
    int scene_count_;
    int scene_iterator_;

    ISM::PosePtr ref_pose_;
    ISM::PointPtr last_ref_pose_;

    dynamic_reconfigure::Server<asr_ism_visualizations::ism_result_visualizerConfig>* reconfigure_server_;

};
typedef boost::shared_ptr<ISMResultVisualizerRVIZ> ISMResultVisualizerRVIZPtr;
}	



