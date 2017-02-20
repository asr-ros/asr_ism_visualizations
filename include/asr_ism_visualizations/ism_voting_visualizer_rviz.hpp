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
#include <asr_ism_visualizations/ism_voting_visualizerConfig.h>

namespace VIZ
{
class ISMVotingVisualizerRVIZ : public VisualizerRVIZ
{
public:
    ISMVotingVisualizerRVIZ(const ros::Publisher& publisher, double bin_size, const ros::NodeHandle& nh)
        : VisualizerRVIZ(publisher), bin_size_(bin_size)
    {
        reconfigure_server_ = new dynamic_reconfigure::Server<asr_ism_visualizations::ism_voting_visualizerConfig>(nh);
        dynamic_reconfigure::Server<asr_ism_visualizations::ism_voting_visualizerConfig>::CallbackType reconf_callback = boost::bind(&ISMVotingVisualizerRVIZ::dynamicReconfCallback, this, _1, _2);
        reconfigure_server_->setCallback(reconf_callback);
    }

    ~ISMVotingVisualizerRVIZ() {
        reconfigure_server_->clearCallback();
        delete reconfigure_server_;
    }

    void dynamicReconfCallback(asr_ism_visualizations::ism_voting_visualizerConfig &config, uint32_t level) {
        ROS_DEBUG_STREAM("Voting vizualizer dyn-params updated.");

        line_scale_ = config.lineScale;
        marker_lifetime_ = config.markerLifetime;
        base_frame_ = config.baseFrame;
        axis_scale_ = config.axisScale;
        grid_scale_ = config.gridScale;
        object_marker_scale_ = config.objectMarkerScale;
        bin_opacity_ = config.binOpacity;
        grid_opacity_ = config.gridOpacity;
        vote_opacity_ = config.voteOpacity;
        object_marker_opacity_ = config.objectMarkerOpacity;
        confidence_sphere_opacity_ = config.confidenceSphereOpacity;
        pattern_name_ = config.patternname;
        scene_name_ = config.scenename;
    }

    void setParams(double line_scale, double axis_scale, double bin_scale, double grid_scale, double object_marker_scale,
                   double bin_opacity, double grid_opacity, double vote_opacity, double object_marker_opacity, double confidence_sphere_opacity, std::string pattern_name, std::string scene_name){

        bin_scale_ = bin_scale;
        line_scale_ = line_scale;
        axis_scale_ = axis_scale;
        bin_scale_ = bin_scale;
        grid_scale_ = grid_scale;
        object_marker_scale_ = object_marker_scale;
        bin_opacity_ = bin_opacity;
        grid_opacity_ = grid_opacity;
        vote_opacity_ = vote_opacity;
        object_marker_opacity_ = object_marker_opacity;
        confidence_sphere_opacity_ = confidence_sphere_opacity;
        pattern_name_ = pattern_name;
        scene_name_ = scene_name;
    }


    void setGlobalParams(double marker_lifetime, std::string base_frame){
        marker_lifetime_ = marker_lifetime;
        base_frame_ = base_frame;
    }

    /// adds Visualization and draws Votes
    /// @param votingSpace the whole votingspace
    /// @param results all results of recognition
    void addVisualization(const ISM::PatternNameAndVotingSpaceTuple& voting_space, std::vector<ISM::RecognitionResultPtr> results);

    /// returns Name of pattern for visualization
    std::string getPatternName(){
        return pattern_name_;
    }

private:

    /// Generates the Voxelgrid:
    /// tuple<minX, minY, minZ, maxX, maxY, maxZ>
    /// @param gridBB the BoundingBox of all Votes scaled to bin_size and the given frame.
    visualization_msgs::Marker generateGridMarker(std::tuple<int, int, int, int, int, int> grid_bb);

    /// Generates the Bins and then calling generateGridMarker
    /// @param votingSpace the whole votingspace
    visualization_msgs::MarkerArray generateBinAndGridMarker(ISM::VotingSpacePtr voting_space_ptr);

    /// Generates the markers of all Votes with a framemarker (xyz-axis)
    /// @param voterToPose map of all voters to its votes
    visualization_msgs::MarkerArray generateVoteMarker(std::map<ISM::ObjectPtr, std::vector<ISM::PosePtr>> voter_to_poses_map);

    /// Generates the marker of the scenereference
    /// @param result result of the current pattern and recognition
    /// @param allVotedPoses all Poses of Votes in the current votingspace
    visualization_msgs::MarkerArray generateRefMarker(ISM::RecognitionResultPtr result, std::vector<ISM::PosePtr> voted_poses);

    /// maps the Votes of the given votingspace to its Voter
    /// @param votingSpace the current votingspace
    ///
    std::map<ISM::ObjectPtr, std::vector<ISM::PosePtr>> getSortedVotedPoses(ISM::VotingSpacePtr voting_space_ptr);
    ISM::RecognitionResultPtr getRecognitionResultPtrOfPattern(ISM::RecognitionResultPtr result);
    visualization_msgs::Marker generateResultVoteOverlay(ISM::RecognitionResultPtr result);

    std::string base_frame_;
    double marker_lifetime_;

    std::string pattern_name_;
    std::string scene_name_;
    double bin_size_;

    double bin_scale_;
    double grid_scale_;
    double axis_scale_;
    double line_scale_;
    double object_marker_scale_;
    double bin_opacity_;
    double grid_opacity_;
    double vote_opacity_;
    double object_marker_opacity_;
    double confidence_sphere_opacity_;

    dynamic_reconfigure::Server<asr_ism_visualizations::ism_voting_visualizerConfig>* reconfigure_server_;

};
typedef boost::shared_ptr<ISMVotingVisualizerRVIZ> ISMVotingVisualizerRVIZPtr;

}
