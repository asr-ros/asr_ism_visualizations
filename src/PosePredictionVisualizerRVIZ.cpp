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
#include <asr_ism_visualizations/PosePredictionVisualizerRVIZ.hpp>

//Ilcas Include
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

namespace VIZ {

void PosePredictionVisualizer::addPosePredictionVisualization(ISM::PosePtr referencePosePtr,
                                                              asr_msgs::AsrAttributedPointCloud attributedPointCloud,
                                                              std::string markerNameSpace)
{
    ROS_DEBUG("Publish Debug Marker");
    std::string referenceNameSpace = markerNameSpace + "_reference";
    std::string arrowNamespace = markerNameSpace + "_arrows";
    std::string pointCloudNamespace = markerNameSpace + "_point_cloud";

    ColorRGBA referenceColor = VizHelperRVIZ::createColorRGBA(0.0, 0.0, 1.0, 1.0);
    ColorRGBA pointColor = VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 1.0);
    unsigned int id = 0;

    Marker referenceMarker = VizHelperRVIZ::createSphereMarker(referencePosePtr->point,
                                                               baseFrame, referenceNameSpace, id, referenceRadius,
                                                               referenceColor, arrowLifeTime);
    id++;
    addMarker(referenceMarker);

    std::map< std::string, ColorRGBA> objectColors;
    for (asr_msgs::AsrAttributedPoint attributedPoint: attributedPointCloud.elements)
    {
        std::string object_type = attributedPoint.type;
        ColorRGBA arrowColor;
        if (objectColors.find(object_type) != objectColors.end())
            arrowColor = objectColors[object_type];
        else
        {
            arrowColor = VizHelperRVIZ::hsvToRGBA(colorStepSize * objectColors.size(), 1.0, 1.0);
            objectColors[object_type] = arrowColor;
        }

        ISM::PointPtr ismPointPtr(new ISM::Point(attributedPoint.pose.position.x,
                                                 attributedPoint.pose.position.y,
                                                 attributedPoint.pose.position.z));
        Marker arrowMarker = VizHelperRVIZ::createArrowMarkerToPoint(referencePosePtr->point,ismPointPtr,
                                                                     baseFrame, arrowNamespace, id++, arrowScaleX,
                                                                     arrowScaleY, arrowScaleZ, arrowColor,
                                                                     arrowLifeTime);
        Marker pointCloudMarker = VizHelperRVIZ::createSphereMarker(ismPointPtr, baseFrame,
                                                                    pointCloudNamespace, id++ , pointRadius,
                                                                    pointColor, pointLifeTime);
        addMarker(arrowMarker);
        addMarker(pointCloudMarker);
    }
}
}
