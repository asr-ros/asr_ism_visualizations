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
#include <asr_ism_visualizations/old_result_visualizer_rviz.hpp>

//Ilcas includes
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>
#include <ISM/utility/GeometryHelper.hpp>

//foreigen includes
#include <Eigen/StdVector>

namespace VIZ
{
OLD_ResultVisualizerRVIZ::~OLD_ResultVisualizerRVIZ()
{
}
void OLD_ResultVisualizerRVIZ::addVisualization(const ISM::RecognitionResultPtr recognition_result,
                                                std::string name_space_prefix)
{
    visualization_msgs::MarkerArray current_marker_array = getMarkersFromResult(recognition_result,
                                                                                0,
                                                                                name_space_prefix);
    addMarker(current_marker_array);
    publishCollectedMarkers();
}

visualization_msgs::MarkerArray OLD_ResultVisualizerRVIZ::getMarkersFromResult(const ISM::RecognitionResultPtr result,
                                                                               int depth,
                                                                               std::string name_space_prefix)
{
    clearAllMarkerOfTopic();
    visualization_msgs::MarkerArray retMarkers;

    if (result->referencePose == nullptr)
    {
        return retMarkers;
    }

    std::string markerNamespace = name_space_prefix + prefix + "ism_" + result->patternName;

    /* create marker for reference-pose */
    auto refPose = result->referencePose;
    int id = 0;
    std_msgs::ColorRGBA refColor = (depth == 0) ? VizHelperRVIZ::createColorRGBA(0.0, 0.0, 1.0, 1.0)
                                                : VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 1.0);

    auto refMarker = VizHelperRVIZ::createSphereMarker(getAdjustedPosition(refPose, depth),
                                                       this->baseFrame, markerNamespace + "_ref",
                                                       id, 0.025, refColor, this->markerLifetime);

    retMarkers.markers.push_back(refMarker);

    /* create marker to show the point of ascension in the ISM-tree  */
    if(depth > 0)
    {
        std_msgs::ColorRGBA ascArrowColor = VizHelperRVIZ::createColorRGBA(0.0, 0.0, 1.0, 1.0);
        auto ascArrowMarker = VizHelperRVIZ::createArrowMarkerToPoint(getAdjustedPosition(refPose, depth),
                                                                      getAdjustedPosition(refPose, depth - 1), this->baseFrame,
                                                                      markerNamespace + "_point_of_ascension_arrow", id++, 0.024,  0.04,
                                                                      0.04, ascArrowColor, this->markerLifetime);
        retMarkers.markers.push_back(ascArrowMarker);
    }


    /* create marker(arrow) from object positions(sphere) to the corresponding reference position(sphere) */
    std_msgs::ColorRGBA confidenceColor = VizHelperRVIZ::confidenceToColor(result->confidence);
    for (ISM::ObjectPtr& obj : result->recognizedSet->objects)
    {
        auto arrowToRefMarker = VizHelperRVIZ::createArrowMarkerToPoint(getAdjustedPosition(obj->pose, depth),
                                                                        getAdjustedPosition(refPose, depth),
                                                                        this->baseFrame, markerNamespace + "_to_ref_arrow",
                                                                        id++, 0.015,  0.025, 0.04, confidenceColor, this->markerLifetime);
        retMarkers.markers.push_back(arrowToRefMarker);
    }

    /* retrieve markers for the sub results, one step deeper in the ISM-tree */
    for (ISM::RecognitionResultPtr& subPattern : result->subPatterns)
    {
        visualization_msgs::MarkerArray subMarkers = OLD_ResultVisualizerRVIZ::getMarkersFromResult(subPattern, depth + 1, name_space_prefix);
        for (auto tempMarker : subMarkers.markers)
        {
            retMarkers.markers.push_back(tempMarker);
        }
    }
    return retMarkers;
}

ISM::PointPtr OLD_ResultVisualizerRVIZ::getAdjustedPosition(ISM::PosePtr pose, int depth)
{
    Eigen::Quaternion<double> rotation = ISM::GeometryHelper::quatToEigenQuat(pose->quat);
    rotation.normalize();
    Eigen::Vector3d translation(0.0, (-depth * 0.10), 0.0);
    Eigen::Vector3d offset = rotation._transformVector(translation);
    return ISM::PointPtr(new ISM::Point(pose->point->eigen.x() + offset[0],
                         pose->point->eigen.y() + offset[1],
            pose->point->eigen.z() + offset[2]));

}
}


