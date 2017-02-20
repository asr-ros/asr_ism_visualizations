/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Meißner Pascal, Reckling Reno, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
//Local include
#include <asr_ism_visualizations/model_visualizer_rviz.hpp>

//ISM includes
#include <ISM/utility/GeometryHelper.hpp>

//#define USE_LINES


namespace VIZ
{
void ModelVisualizerRVIZ::addVisualization(const std::string pattern_name, const ISM::TracksPtr tracks, const ISM::TracksPtr ism_object_tracks,
                                           const std::map<int, std::vector<ISM::VoteSpecifierPtr>> track_index_to_votes)
{
    initObjectToColorMap(tracks, ism_object_tracks);
    ISM::TracksPtr overall_tracks = ISM::TracksPtr(new ISM::Tracks(tracks->tracks));
    overall_tracks->tracks.insert(overall_tracks->tracks.end(), ism_object_tracks->tracks.begin(), ism_object_tracks->tracks.end());


    visualization_msgs::MarkerArray current_marker_array = generateModelMarkers(pattern_name, overall_tracks, track_index_to_votes);

    addMarker(current_marker_array);
    publishCollectedMarkers();
}

visualization_msgs::MarkerArray ModelVisualizerRVIZ::generateModelMarkers(std::string pattern_name, const ISM::TracksPtr tracks, const std::map<int, std::vector<ISM::VoteSpecifierPtr>> track_index_to_votes)
{
    visualization_msgs::MarkerArray return_markers;

    for (const std::pair<int, std::vector<ISM::VoteSpecifierPtr>>& votes : track_index_to_votes)
    {
        int index = votes.first;
        if (index < 0)
        {
            continue;
        }

        /* find reference */
        ISM::ObjectPtr ref;
        for (ISM::VoteSpecifierPtr vote : votes.second)
        {
            if (ISM::GeometryHelper::isSelfVote(vote))
            {
                ISM::TrackPtr track = tracks->getTrackByTypeAndId(vote->objectType, vote->observedId);
                if (track != nullptr)
                {
                    if(static_cast<size_t>(index) >= track->objects.size())
                    {
                        std::cout << "Index: " << index << "exceeds objects size" << std::endl;
                        continue;
                    }
                    ref = track->objects[index];
                }

                break;
            }
        }

        if (ref == nullptr)
        {
            std::cout << "no reference for " << pattern_name
                      << " at the track-index " << votes.first << "\n";
            continue;
        }

        /* find corresponding vote/object pair and generate marker */
        for (const ISM::VoteSpecifierPtr& vote : votes.second)
        {
            ISM::ObjectPtr obj;
            ISM::TrackPtr track = tracks->getTrackByTypeAndId(vote->objectType, vote->observedId);
            if (track == nullptr)
            {
                continue;
            }

            if(static_cast<size_t>(index) >= track->objects.size())
            {
                std::cout << "Index: " << index << "exceeds objects size" << std::endl;
                continue;
            }

            obj = track->objects[index];
            if (ref == nullptr)
            {
                continue;
            }

            ISM::PosePtr expected_object_pose = ISM::GeometryHelper::getSourcePose(ref->pose, ISM::GeometryHelper::getSourcePoint(ref->pose, vote->refToObjectQuat, vote->radius), vote->refToObjectPoseQuat);

            if (ISM::GeometryHelper::poseEqual(expected_object_pose, obj->pose))
            {
                #ifdef USE_LINES
                    return_markers.markers.push_back(VizHelperRVIZ::createLine(obj->pose->point, ref->pose->point, 0.002, index, base_frame_, prefix_ + pattern_name + "_vote_from_" + obj->type + "_" + obj->observedId,
                                                                               object_to_color_map_[obj->type][obj->observedId], marker_lifetime_));
                #else
                    return_markers.markers.push_back(VizHelperRVIZ::createArrowMarkerToPoint(obj->pose->point, ref->pose->point, base_frame_, prefix_ + pattern_name + "_vote_from_" + obj->type + "_" + obj->observedId,
                                                                                             index, 0.002, 0.007, 0.015, object_to_color_map_[obj->type][obj->observedId], marker_lifetime_));
                #endif

            }
            else
            {
                ISM::VoteSpecifierPtr vote;
                std::cout << "Vote from: " << vote->objectType  << " with ID: " << vote->observedId
                          << " at track-index: " << index << " couldn't be drawn!\n";
            }
        }
    }

    return return_markers;
}

void ModelVisualizerRVIZ::initObjectToColorMap(ISM::TracksPtr tracks, ISM::TracksPtr ism_object_tracks)
{
    double value = 0.6;
    double color_step_size = 240.0 / (tracks->tracks.size() + 1);
    for (size_t i = 0; i < tracks->tracks.size(); i++)
    {
        object_to_color_map_[tracks->tracks[i]->type][tracks->tracks[i]->observedId] = VizHelperRVIZ::hsvToRGBA(color_step_size * (i + 1), 1.0, value);
    }

    color_step_size = 120.0 / (ism_object_tracks->tracks.size());
    for (size_t i = 0; i < ism_object_tracks->tracks.size(); i++)
    {
        object_to_color_map_[ism_object_tracks->tracks[i]->type][ism_object_tracks->tracks[i]->observedId] = VizHelperRVIZ::hsvToRGBA((color_step_size * i) + 240, 1.0, value);
    }
}
}
