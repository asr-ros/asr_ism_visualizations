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
#include <asr_ism_visualizations/vote_visualizer_rviz.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

//ISM includes
#include <ISM/utility/GeometryHelper.hpp>

namespace VIZ
{
void VoteVisualizerRVIZ::addVisualization(const std::map<ISM::ObjectPtr, std::vector<ISM::VoteSpecifierPtr>>& object_to_votes)
{
    visualization_msgs::MarkerArray current_marker_array = generateVoteMarkers(object_to_votes);

    addMarker(current_marker_array);
    publishCollectedMarkers();
}

visualization_msgs::MarkerArray VoteVisualizerRVIZ::generateVoteMarkers(const std::map<ISM::ObjectPtr, std::vector<ISM::VoteSpecifierPtr>>& object_to_votes)
{
    visualization_msgs::MarkerArray return_markers;

    int i = 0;
    for (std::map<ISM::ObjectPtr, std::vector<ISM::VoteSpecifierPtr>>::const_iterator obj_to_votes_it = object_to_votes.begin(); obj_to_votes_it != object_to_votes.end(); ++obj_to_votes_it)
    {
        visualization_msgs::Marker marker;
        std::string name_space = prefix_ + obj_to_votes_it->first->type + "_" + obj_to_votes_it->first->observedId;
        double hue_step = 360 / object_to_votes.size();
        double hue_value = hue_step * i++;

        int id = 0;
        for (const ISM::VoteSpecifierPtr& vote : obj_to_votes_it->second)
        {
            ISM::PointPtr dest = ISM::GeometryHelper::applyQuatAndRadiusToPose(obj_to_votes_it->first->pose, vote->objectToRefQuat, vote->radius);
            marker = VizHelperRVIZ::createArrowMarkerToPoint(obj_to_votes_it->first->pose->point, dest, base_frame_, name_space + "_votes", id++, 0.0015, 0.003, 0.003, VizHelperRVIZ::hsvToRGBA(hue_value, 1.0, 1.0), 0, 0.003);
            return_markers.markers.push_back(marker);
        }
    }

    return return_markers;
}
}
