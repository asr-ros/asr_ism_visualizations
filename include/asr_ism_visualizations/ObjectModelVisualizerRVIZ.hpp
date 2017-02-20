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
#include <ISM/common_type/Object.hpp>

namespace VIZ
{
class ObjectModelVisualizerRVIZ : public VisualizerRVIZ {



public:

    ObjectModelVisualizerRVIZ(ros::Publisher publisher, std::string baseFrame, std::string prefix, double markerLifetime)
        : VisualizerRVIZ(publisher), baseFrame(baseFrame), prefix(prefix), markerLifetime(markerLifetime)
    {};

    /**
     *TODO:
     * @param objects
     * @param objects_to_hue_map
     */
    void drawObjectModels(const std::vector<ISM::ObjectPtr>& objects, const std::map<ISM::ObjectPtr, double>& objects_to_hue_map = std::map<ISM::ObjectPtr, double>());

private:

    visualization_msgs::MarkerArray getMarkersFromObjects(const std::vector<ISM::ObjectPtr> objects, const std::map<ISM::ObjectPtr, double>& objects_to_hue_map);

    std::string baseFrame;
    std::string prefix;
    double markerLifetime;
    std::map<std::string, std::string> objectTypeToPathCache;

    visualization_msgs::MarkerArray markersFromLastPublication;
};
}



