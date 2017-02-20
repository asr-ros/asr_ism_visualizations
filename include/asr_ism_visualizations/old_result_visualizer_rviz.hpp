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
//Ilcas include
#include <asr_ism_visualizations/visualizer_rviz.hpp>
#include <ISM/common_type/RecognitionResult.hpp>


namespace VIZ
{
class OLD_ResultVisualizerRVIZ : public VisualizerRVIZ
{

public:
    OLD_ResultVisualizerRVIZ(const ros::Publisher& publisher, std::string baseFrame, double stepLength,
                             std::string prefix, double markerLifetime) :
        VisualizerRVIZ(publisher),
        baseFrame(baseFrame), stepLength(stepLength), prefix(prefix),
        markerLifetime(markerLifetime)
    {}
    ~OLD_ResultVisualizerRVIZ();

    void addVisualization(const ISM::RecognitionResultPtr recognition_result,
                          std::string name_space_prefix = "");

private:
    visualization_msgs::MarkerArray getMarkersFromResult(const ISM::RecognitionResultPtr result,
                                                         int depth,
                                                         std::string name_space_prefix = "");

    /* Helper method to translate a point. So we can see the tree
             *depth objects residing in trough their corresponding marker
            */
    ISM::PointPtr getAdjustedPosition(ISM::PosePtr pose, int depth);
    
    std::string baseFrame;
    double stepLength;
    std::string prefix;
    double markerLifetime;
};
typedef boost::shared_ptr<OLD_ResultVisualizerRVIZ> OLD_ResultVisualizerRVIZPtr;
}	



