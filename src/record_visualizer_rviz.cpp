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
#include <asr_ism_visualizations/record_visualizer_rviz.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

namespace VIZ
{
void RecordVisualizerRVIZ::addVisualization(const ISM::TracksPtr record)
{
    visualization_msgs::MarkerArray current_marker_array = RecordVisualizerRVIZ::generateRecordMarkers(record);
    addMarker(current_marker_array);
    publishCollectedMarkers();
}

visualization_msgs::MarkerArray RecordVisualizerRVIZ::generateRecordMarkers(const ISM::TracksPtr record)
{
    visualization_msgs::MarkerArray return_markers;
    double color_step_size = 240.0 / (record->tracks.size() + 1);

    /* create actual marker for each track */
    std::vector<ISM::PosePtr> track_poses;
    std_msgs::ColorRGBA track_color;
    visualization_msgs::MarkerArray temp_markers;
    std::string marker_namespace;
    for (size_t i = 0; i < record->tracks.size(); i++)
    {
        track_poses = VizHelperRVIZ::posesFromTrack(record->tracks[i]);
        track_color = VizHelperRVIZ::hsvToRGBA(color_step_size * (i + 1), 1.0, 1.0);
        marker_namespace = this->prefix_ + record->tracks[i]->type + "_" + record->tracks[i]->observedId + "_track";

        temp_markers = VizHelperRVIZ::createTrackMarkers(track_poses, this->base_frame_, marker_namespace, 0.006, track_color, this->marker_lifetime_);
        return_markers.markers.insert(return_markers.markers.end(), temp_markers.markers.begin(), temp_markers.markers.end());
    }

    return return_markers;
}
}
