/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Meißner Pascal, Reckling Reno, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
//Header includes
#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>

//Ilcas includes
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>
#include <ISM/recognizer/VotingSpace.hpp>
#include <ISM/utility/Util.hpp>
#include <ISM/utility/viz_helper.hpp>
#include <ISM/utility/GeometryHelper.hpp>

namespace VIZ
{



void  ObjectModelVisualizerRVIZ::drawObjectModels(const std::vector<ISM::ObjectPtr>& objects, const std::map<ISM::ObjectPtr, double>& objects_to_hue_map)
{
    visualization_msgs::MarkerArray vizObjects;
    vizObjects = ObjectModelVisualizerRVIZ::getMarkersFromObjects(objects, objects_to_hue_map);
    addMarker(vizObjects);
    publishCollectedMarkers();
}

visualization_msgs::MarkerArray ObjectModelVisualizerRVIZ::getMarkersFromObjects(const std::vector<ISM::ObjectPtr> objects, const std::map<ISM::ObjectPtr, double>& objects_to_hue_map)
{
    visualization_msgs::MarkerArray retMarkers;

    std::string markerNamespace = this->prefix + "Object_Mesh_";
    int id = 0;

    for(const ISM::ObjectPtr& object : objects)
    {
        std::string tempPath;
        tempPath = object->ressourcePath.string();
        if( !tempPath.empty() )
        {
            visualization_msgs::Marker tempObjectMarker;
            tempObjectMarker = VizHelperRVIZ::createMeshMarker(object->pose, this->baseFrame, markerNamespace + object->type + object->observedId, 0,
                                                               VizHelperRVIZ::createColorRGBA(ISM::getColorOfObject(object)), this->markerLifetime,
                                                               tempPath);

            // predefined highlighting exists?
            std::map<ISM::ObjectPtr, double>::const_iterator it = objects_to_hue_map.find(object);
            if (it != objects_to_hue_map.end())
            {
                tempObjectMarker.color = VizHelperRVIZ::hsvToRGBA(it->second, 1.0, 1.0);
                tempObjectMarker.color.a = 0.5;
            }


            retMarkers.markers.push_back(tempObjectMarker);
        } else
        {
            visualization_msgs::MarkerArray tempMarkers;
            tempMarkers = VizHelperRVIZ::createCoordinateMarker(object->pose, this->baseFrame, markerNamespace + object->type + object->observedId + "_FRAME" ,
                                                                id, 0.05, 0.002, this->markerLifetime);


            retMarkers.markers.insert(retMarkers.markers.end(), tempMarkers.markers.begin(), tempMarkers.markers.end());
            retMarkers.markers.insert(retMarkers.markers.end(), tempMarkers.markers.begin(), tempMarkers.markers.end());
            id += tempMarkers.markers.size();
        }
        //Visualize Orientation-Axis(X-Axis)
        visualization_msgs::Marker axisMarker = VIZ::VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace + object->type + object->observedId + "_xAXIS", id++, 0.1, 0.01, 0.01, VIZ::VizHelperRVIZ::createColorRGBA(1, 0, 0, 1), markerLifetime);
        axisMarker.type = visualization_msgs::Marker::ARROW;
        Eigen::Quaterniond transformed_X_Axis = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * ISM::GeometryHelper::quatToEigenQuat(object->pose->quat);
        axisMarker.pose.orientation = VizHelperRVIZ::quatToQuaternionMsg(ISM::GeometryHelper::eigenQuatToQuat(transformed_X_Axis));
        axisMarker.pose.position = VizHelperRVIZ::pointToPointMsg(object->pose->point);
        axisMarker.action = visualization_msgs::Marker::ADD;
        retMarkers.markers.push_back(axisMarker);
    }

    return retMarkers;
}
}
