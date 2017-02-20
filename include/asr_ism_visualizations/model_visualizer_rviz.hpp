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
//Local includes
#include <asr_ism_visualizations/visualizer_rviz.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

//ISM inlcudes
#include <ISM/common_type/VoteSpecifier.hpp>
#include <ISM/common_type/Tracks.hpp>


namespace VIZ
{
class ModelVisualizerRVIZ : public VisualizerRVIZ{

public:
    ModelVisualizerRVIZ(ros::Publisher publisher, std::string base_frame, std::string prefix, double marker_lifetime)
        : VisualizerRVIZ(publisher), base_frame_(base_frame), prefix_(prefix), marker_lifetime_(marker_lifetime)
    {};


    /**
     * Manages generation and publication of marker for model visualization.
     *
     * @param pattern_name Name of (sub)pattern.
     * @param tracks Tracks of real objects (objects which don’t represent ISMs).
     * @param ism_object_tracks Tracks of objects which represent ISMs.
     * @param track_index_to_votes Votes ordered by track index.
     */
    void addVisualization(const std::string pattern_name, const ISM::TracksPtr tracks, const ISM::TracksPtr ism_object_tracks,
                          const std::map<int, std::vector<ISM::VoteSpecifierPtr>> track_index_to_votes);


private:

    /**
     * Generates markers for model visualization.
     *
     * @param pattern_name Name of (sub)pattern.
     * @param tracks Tracks of all objects.
     * @param track_index_to_votes Votes ordered by track index.
     */
    visualization_msgs::MarkerArray generateModelMarkers(std::string pattern_name, const ISM::TracksPtr tracks, const std::map<int, std::vector<ISM::VoteSpecifierPtr>> track_index_to_votes);

    void initObjectToColorMap(ISM::TracksPtr tracks, ISM::TracksPtr ism_object_tracks);

    std::string base_frame_;
    std::string prefix_;
    double marker_lifetime_;
    std::map<std::string, std::map<std::string, std_msgs::ColorRGBA>> object_to_color_map_;
};
}	



