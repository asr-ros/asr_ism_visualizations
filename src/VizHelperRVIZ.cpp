/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Meißner Pascal, Reckling Reno, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "asr_ism_visualizations/VizHelperRVIZ.hpp"
#include "ISM/utility/GeometryHelper.hpp"

//Local includes

namespace VIZ
{	

std::map<Eigen::Vector3i, bool, cmpVector3i > VizHelperRVIZ::binDrawn;
geometry_msgs::Point VizHelperRVIZ::pointToPointMsg(ISM::PointPtr point)
{
    geometry_msgs::Point retPoint;
    retPoint.x = point->eigen.x();
    retPoint.y = point->eigen.y();
    retPoint.z = point->eigen.z();
    return retPoint;
}

geometry_msgs::Quaternion VizHelperRVIZ::quatToQuaternionMsg(ISM::QuaternionPtr quat)
{
    geometry_msgs::Quaternion retQuat;
    retQuat.w = quat->eigen.w();
    retQuat.x = quat->eigen.x();
    retQuat.y = quat->eigen.y();
    retQuat.z = quat->eigen.z();
    return retQuat;
}

MarkerArray VizHelperRVIZ::createArrowMarkerFromVote(ISM::VoteSpecifierPtr vote,
                ISM::ObjectPtr o, std::string baseFrame, std::string ns, int32_t id, double bin_size,
                std_msgs::ColorRGBA binColor,
                std_msgs::ColorRGBA voteColor)
{
    visualization_msgs::MarkerArray result;
    ISM::PointPtr referencePoint = ISM::GeometryHelper::applyQuatAndRadiusToPose(o->pose,
                    vote->objectToRefQuat, vote->radius);
    ISM::PosePtr destPose = ISM::GeometryHelper::getReferencePose(o->pose, referencePoint,
                    vote->objectToRefPoseQuat);
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = baseFrame;
    m.ns = ns;
    m.id = id;
    m.type =  visualization_msgs::Marker::ARROW;
    m.scale.x = 0.003;
    m.scale.y = 0.003;
    m.scale.z = 0.003;
    geometry_msgs::Point pO;
    pO.x = o->pose->point->eigen.x();
    pO.y = o->pose->point->eigen.y();
    pO.z = o->pose->point->eigen.z();
    geometry_msgs::Point pD;
    pD.x = destPose->point->eigen.x();
    pD.y = destPose->point->eigen.y();
    pD.z = destPose->point->eigen.z();
    m.points.push_back(pO);
    m.points.push_back(pD);
    m.lifetime = ros::Duration();
    m.color = voteColor;
    result.markers.push_back(m);

    auto refBin = getBinFromPoint(referencePoint, bin_size, baseFrame, binColor);
    auto destBin = getBinFromPoint(destPose->point, bin_size, baseFrame, binColor);

    ros::Duration d;
    visualization_msgs::MarkerArray orient = createSphereMarkerWithOrientation(destPose, baseFrame, ns,
                id + 1, 0.005, voteColor, d.toSec(), id + 2);
    result.markers.insert(result.markers.end(), orient.markers.begin(), orient.markers.end());
    return result;
}

visualization_msgs::MarkerArray VizHelperRVIZ::createSphereMarkerWithOrientation
        (ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float radius,
                        std_msgs::ColorRGBA color, double markerLifetime, int axesFirstId)
{
    const double length = radius * 2;
    const float scale = radius / 5 * 2;
    visualization_msgs::MarkerArray result;
    result.markers.push_back(createSphereMarker(pose->point, baseFrame, markerNamespace, id,
                radius, color, markerLifetime));

    std_msgs::ColorRGBA xAxeColor = createColorRGBA(1.0, 0.0, 0.0, 1.0);
    std_msgs::ColorRGBA yAxeColor = createColorRGBA(0.0, 1.0, 0.0, 1.0);
    std_msgs::ColorRGBA zAxeColor = createColorRGBA(0.0, 0.0, 1.0, 1.0);

    auto rot = ISM::GeometryHelper::quatToEigenQuat(pose->quat);
    Eigen::Vector3d xV = rot._transformVector(Eigen::Vector3d::UnitX());
    Eigen::Vector3d yV = rot._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d zV = rot._transformVector(Eigen::Vector3d::UnitZ());

    visualization_msgs::Marker xAxe = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame,
            markerNamespace, axesFirstId, scale, scale, scale, xAxeColor, markerLifetime);
    xAxe.type = visualization_msgs::Marker::ARROW;
    xAxe.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point xAxePStart;
    xAxePStart.x = pose->point->eigen.x();
    xAxePStart.y = pose->point->eigen.y();
    xAxePStart.z = pose->point->eigen.z();
    geometry_msgs::Point xAxePEnd;
    xAxePEnd.x = pose->point->eigen.x() + xV.x() * length;
    xAxePEnd.y = pose->point->eigen.y() + xV.y() * length;
    xAxePEnd.z = pose->point->eigen.z() + xV.z() * length;
    xAxe.points.push_back(xAxePStart);
    xAxe.points.push_back(xAxePEnd);
    result.markers.push_back(xAxe);

    visualization_msgs::Marker yAxe = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame,
            markerNamespace, axesFirstId + 1, scale, scale, scale, yAxeColor, markerLifetime);
    yAxe.type = visualization_msgs::Marker::ARROW;
    yAxe.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point yAxePStart;
    yAxePStart.x = pose->point->eigen.x();
    yAxePStart.y = pose->point->eigen.y();
    yAxePStart.z = pose->point->eigen.z();
    geometry_msgs::Point yAxePEnd;
    yAxePEnd.x = pose->point->eigen.x() + yV.x() * length;
    yAxePEnd.y = pose->point->eigen.y() + yV.y() * length;
    yAxePEnd.z = pose->point->eigen.z() + yV.z() * length;
    yAxe.points.push_back(yAxePStart);
    yAxe.points.push_back(yAxePEnd);
    result.markers.push_back(yAxe);

    visualization_msgs::Marker zAxe = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame,
            markerNamespace, axesFirstId + 2, scale, scale, scale, zAxeColor, markerLifetime);
    zAxe.type = visualization_msgs::Marker::ARROW;
    zAxe.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point zAxePStart;
    zAxePStart.x = pose->point->eigen.x();
    zAxePStart.y = pose->point->eigen.y();
    zAxePStart.z = pose->point->eigen.z();
    geometry_msgs::Point zAxePEnd;
    zAxePEnd.x = pose->point->eigen.x() + zV.x() * length;
    zAxePEnd.y = pose->point->eigen.y() + zV.y() * length;
    zAxePEnd.z = pose->point->eigen.z() + zV.z() * length;
    zAxe.points.push_back(zAxePStart);
    zAxe.points.push_back(zAxePEnd);
    result.markers.push_back(zAxe);

    return result;
}


std::vector<ISM::PosePtr> VizHelperRVIZ::posesFromTrack(ISM::TrackPtr track)
{
    std::vector<ISM::PosePtr> trackPoses;
    for (auto& obj : track->objects)
    {
        if (obj)
        {
            trackPoses.push_back(obj->pose);
        }
        else
        {
            /* to maintain the trajectory length */
            trackPoses.push_back(ISM::PosePtr());
        }
    }

    return trackPoses;
}

ColorRGBA VizHelperRVIZ::createColorRGBA(float red, float green, float blue, float alpha)
{
    ColorRGBA retColor;

    retColor.r = red;
    retColor.g = green;
    retColor.b = blue;
    retColor.a = alpha;

    return retColor;
}

ColorRGBA VizHelperRVIZ::createColorRGBA(std::vector<float> rgba)
{
    if(rgba.size() != 4){
        ROS_ERROR("Calling VizHelperRVIZ::createColorRGBA with a vector with size != 4 [current size: %zu]", rgba.size());
        return createColorRGBA(0, 0, 0, 0);
    }
    return createColorRGBA(rgba[0], rgba[1], rgba[2], rgba[3]);
}

ColorRGBA VizHelperRVIZ::hsvToRGBA(double hue, double saturation, double value)
{
    ColorRGBA retColor;
    
    const int hi = hue / 60;
    const double f = (hue / 60.0 - (double) hi);
    const double p = value * (1 - saturation);
    const double q = value * (1 - saturation * f);
    const double t = value * (1 - saturation * (1 - f));

    switch (hi) {
    case 1:
        retColor = VizHelperRVIZ::createColorRGBA(q, value, p, 1.0);
        break;
    case 2:
        retColor = VizHelperRVIZ::createColorRGBA(p, value, t, 1.0);
        break;
    case 3:
        retColor = VizHelperRVIZ::createColorRGBA(p, q, value, 1.0);
        break;
    case 4:
        retColor = VizHelperRVIZ::createColorRGBA(t, p, value, 1.0);
        break;
    case 5:
        retColor = VizHelperRVIZ::createColorRGBA(value, p, q, 1.0);
        break;
    default:
        retColor = VizHelperRVIZ::createColorRGBA(value, t, p, 1.0);
        break;
    }

    return retColor;
}

ColorRGBA VizHelperRVIZ::confidenceToColor(double confidence)
{
    double hue = (120.0 * std::pow(confidence, 5.0));
    return VizHelperRVIZ::hsvToRGBA(hue, 1.0, 1.0);
}

ColorRGBA VizHelperRVIZ::getColorOfObject(const ISM::Object &object)
{
    std::string observedId = object.observedId;
    std::vector<float> rgba;
    if ( ( observedId.length() == 12 ) && ( observedId.find_first_not_of("0123456789") == std::string::npos ) )
    {
        try
        {
            for (int i = 0; i <= 3; i++)
            {
                std::string temp;

                temp = observedId.substr( (i * 3), 3 );
                rgba.push_back(std::stof(temp) / 100.0);
            }
        }
        catch (std::invalid_argument& ia)
        {
            rgba.clear();
            rgba.push_back(0);
            rgba.push_back(0);
            rgba.push_back(0);
            rgba.push_back(1);
        }
    }
    else
    {
        rgba.push_back(0);
        rgba.push_back(0);
        rgba.push_back(0);
        rgba.push_back(0);
    }
    return VizHelperRVIZ::createColorRGBA(rgba);
}

ColorRGBA VizHelperRVIZ::getColorOfObject(const ISM::ObjectPtr object_ptr)
{
    return getColorOfObject(*object_ptr);
}


Marker VizHelperRVIZ::createMarkerWithoutTypeAndPose(std::string baseFrame, std::string markerNamespace, int id, float xScale, float yScale,
                                                     float zScale, ColorRGBA color, double markerLifetime)
{
    Marker retMarker;

    retMarker.header.frame_id = baseFrame;
    retMarker.header.stamp = ros::Time::now();
    retMarker.ns = markerNamespace;
    retMarker.id = id;
    retMarker.scale.x = xScale;
    retMarker.scale.y = yScale;
    retMarker.scale.z = zScale;
    retMarker.color = color;
    retMarker.lifetime = ros::Duration(markerLifetime);

    return retMarker;
}



Marker VizHelperRVIZ::createSphereMarker(ISM::PointPtr point, std::string baseFrame,
                                         std::string markerNamespace, int id, float radius, ColorRGBA color, double markerLifetime)
{
    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, radius, radius, radius, color, markerLifetime);

    const std::string conePath = "package://asr_ism_visualizations/rsc/sphere.dae";
    retMarker.pose.position = pointToPointMsg(point);

    retMarker.type = Marker::MESH_RESOURCE;
    retMarker.mesh_resource = conePath;
    retMarker.mesh_use_embedded_materials = true;
    return retMarker;
}

Marker VizHelperRVIZ::createConeMarker(ISM::PosePtr pose, std::string baseFrame,
                                         std::string markerNamespace, int id, float radius, ColorRGBA color, double markerLifetime)
{
    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, radius, radius, radius, color, markerLifetime);

    const std::string conePath = "package://asr_ism_visualizations/rsc/openCone.dae";
    retMarker.pose.position = pointToPointMsg(pose->point);
    retMarker.pose.orientation = quatToQuaternionMsg(pose->quat);

    retMarker.type = Marker::MESH_RESOURCE;
    retMarker.mesh_resource = conePath;
    retMarker.mesh_use_embedded_materials = true;

    return retMarker;
}

Marker VizHelperRVIZ::createCylinderMarker(ISM::PosePtr pose, std::string baseFrame,
                                           std::string markerNamespace, int id, float radius, float height, ColorRGBA color, double markerLifetime)
{
    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, radius, radius, height, color, markerLifetime);

    retMarker.type = Marker::CYLINDER;
    retMarker.pose.position = VizHelperRVIZ::pointToPointMsg(pose->point);
    retMarker.pose.orientation = VizHelperRVIZ::quatToQuaternionMsg(pose->quat);

    return retMarker;
}

Marker VizHelperRVIZ::createArrowMarkerToPoint(ISM::PointPtr from, ISM::PointPtr to, std::string baseFrame, std::string markerNamespace,
                                               int id, float xScale, float yScale, float zScale, ColorRGBA color, double markerLifetime)
{
    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, xScale, yScale, zScale, color, markerLifetime);

    retMarker.type = Marker::ARROW;
    retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(from));
    retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(to));

    return retMarker;
}

Marker VizHelperRVIZ::createArrowMarkerToPoint(ISM::PointPtr from, ISM::PointPtr to, std::string baseFrame, std::string markerNamespace,
                                               int id, float xScale, float yScale, float zScale, ColorRGBA color, double markerLifetime, double minLength)
{
    if (ISM::GeometryHelper::getDistanceBetweenPoints(from, to) < minLength)
    {
        ISM::PointPtr newFrom(new ISM::Point(to->eigen.x(), to->eigen.y(), to->eigen.z()));
        newFrom->eigen.z() += minLength;

        return VizHelperRVIZ::createArrowMarkerToPoint(newFrom, to, baseFrame, markerNamespace, id, xScale, yScale, zScale, color, markerLifetime);
    }
    else
    {
        return VizHelperRVIZ::createArrowMarkerToPoint(from, to, baseFrame, markerNamespace, id, xScale, yScale, zScale, color, markerLifetime);
    }

}

MarkerArray VizHelperRVIZ::createCoordinateArrow(std::vector< std::pair< ISM::PointPtr, std::vector<ISM::PosePtr> > > poses_vec, std::string baseFrame,
                                                 std::string markerNamespace, float arrowScale, float axisScale, ColorRGBA color, double markerLifetime)
{

    MarkerArray retMarkers;
    std::vector<geometry_msgs::Point> axisPoints;

    Marker arrowMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 0 ,arrowScale, 0, 0, color, markerLifetime);
    Marker xAxisMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 1, arrowScale, 0, 0, VizHelperRVIZ::createColorRGBA(1, 0, 0, 0.5), markerLifetime);;
    Marker yAxisMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 2, arrowScale, 0, 0, VizHelperRVIZ::createColorRGBA(0, 1, 0, 0.5), markerLifetime);;
    Marker zAxisMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 3, arrowScale, 0, 0, VizHelperRVIZ::createColorRGBA(0, 0, 1, 0.5), markerLifetime);;

    arrowMarker.type = Marker::LINE_LIST;
    xAxisMarker.type = Marker::LINE_LIST;
    yAxisMarker.type = Marker::LINE_LIST;
    zAxisMarker.type = Marker::LINE_LIST;

    for(size_t i = 0; i < poses_vec.size(); i++){
        for (ISM::PosePtr to : poses_vec[i].second)
        {
            arrowMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(poses_vec[i].first));
            arrowMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(to->point));

            axisPoints = VizHelperRVIZ::calculateAxesPoints(to->point, to->quat, axisScale);
            xAxisMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(to->point));
            xAxisMarker.points.push_back(axisPoints[0]);
            yAxisMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(to->point));
            yAxisMarker.points.push_back(axisPoints[1]);
            zAxisMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(to->point));
            zAxisMarker.points.push_back(axisPoints[2]);

        }
    }

    retMarkers.markers.push_back(arrowMarker);
    retMarkers.markers.push_back(xAxisMarker);
    retMarkers.markers.push_back(yAxisMarker);
    retMarkers.markers.push_back(zAxisMarker);

    return retMarkers;
}

Marker VizHelperRVIZ::createLineArrow(ISM::PointPtr fromPoint, std::vector<ISM::PosePtr> toPoses, std::string baseFrame,
                                      std::string markerNamespace, int& id, float arrowScale, ColorRGBA color, double markerLifetime)
{

    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, ++id, arrowScale, 0, 0, color, markerLifetime);
    retMarker.type = Marker::LINE_LIST;
    for (ISM::PosePtr toPose : toPoses)
    {
        retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(fromPoint));
        retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(toPose->point));
    }
    return retMarker;
}
Marker VizHelperRVIZ::createLineMarkerFormPair(std::vector<std::pair<ISM::PointPtr, ISM::PointPtr>> pointPair, std::string baseFrame,
                                      std::string markerNamespace, float arrowScale, ColorRGBA color, double markerLifetime)
{

    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 0, arrowScale, 0, 0, color, markerLifetime);
    retMarker.type = Marker::LINE_LIST;
    for (std::pair<ISM::PointPtr, ISM::PointPtr> onePointPair : pointPair)
    {
        retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(onePointPair.first));
        retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(onePointPair.second));
    }
    return retMarker;
}

Marker VizHelperRVIZ::createMeshMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id,
                                       ColorRGBA color, double markerLifetime, std::string resourcePath)

{
    double scaling = 0.001;/* the model size unit is mm */
    if(resourcePath.compare("package://asr_visualization_server/res/maus.dae") == 0){
        scaling = 0.0007;
    }
    else if(resourcePath.compare("package://asr_visualization_server/res/monitor.dae") == 0){
        scaling = 0.0006;
    }
    else if(resourcePath.compare("package://asr_visualization_server/res/tastatur.dae") == 0){
        scaling = 0.001;
    }
    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id,
                                                                  scaling, scaling, scaling,
                                                                  color, markerLifetime);

    retMarker.type = Marker::MESH_RESOURCE;
    retMarker.mesh_resource = resourcePath;
    retMarker.mesh_use_embedded_materials = true;

    retMarker.pose.position = VizHelperRVIZ::pointToPointMsg(pose->point);
    retMarker.pose.orientation = VizHelperRVIZ::quatToQuaternionMsg(pose->quat);

    return retMarker;
}

MarkerArray VizHelperRVIZ::createTrackMarkers(std::vector<ISM::PosePtr> trackPoses, std::string baseFrame,
                                              std::string markerNamespace, float lineWidth,
                                              ColorRGBA color, double markerLifetime)
{
    MarkerArray retMarkers;
    Marker tempMarker;
    MarkerArray tempMarkers;

    /* actual line representing the track */
    Marker lineMarker;
    lineMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 0, lineWidth, 0.0, 0.0,
                                                               color, markerLifetime);
    lineMarker.type = Marker::LINE_STRIP;

    bool stillSearchingStartPosition = true;
    for (unsigned int i = 0; i < trackPoses.size(); i++)
    {
        if (trackPoses[i] == nullptr)
        {
            continue;
        }

        //ToDo make useFrames dynamic_reconfigure
        bool useFrames = true;
        /* sphere to determine the temporary position on the track */
        if (stillSearchingStartPosition)
        {
            if(useFrames){
                tempMarkers = VizHelperRVIZ::createCoordinateMarker(trackPoses[i], baseFrame, markerNamespace + "_start_position", i * 3, lineWidth * 1, lineWidth / 2, markerLifetime);
            }else{
                tempMarker = VizHelperRVIZ::createSphereMarker(trackPoses[i]->point, baseFrame, markerNamespace + "_start_position", i, lineWidth * 1.05,
                                                               VizHelperRVIZ::createColorRGBA(1.0, 0.0, 0.0, 1.0), markerLifetime);
            }

            stillSearchingStartPosition = false;
        }
        else
        {
            if(useFrames){
                tempMarkers = VizHelperRVIZ::createCoordinateMarker(trackPoses[i], baseFrame, markerNamespace + "_start_position", i * 3, lineWidth * 1, lineWidth / 4, markerLifetime);
            }else{
                tempMarker = VizHelperRVIZ::createSphereMarker(trackPoses[i]->point, baseFrame, markerNamespace + "_positions", i, lineWidth, color, markerLifetime);

            }
        }
        if(useFrames){
            retMarkers.markers.insert(retMarkers.markers.end(), tempMarkers.markers.begin(), tempMarkers.markers.end());
        }else{
            retMarkers.markers.push_back(tempMarker);
        }


        lineMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(trackPoses[i]->point));
    }
    retMarkers.markers.push_back(lineMarker);

    return retMarkers;
}

MarkerArray VizHelperRVIZ::getBinFromPoint (
                const ISM::PointPtr point, double sensivity, std::string baseFrame, std_msgs::ColorRGBA color)
{
    visualization_msgs::MarkerArray binM;
    Eigen::Vector3i bin;
    bin.x() = static_cast<int>(floor(point->eigen.x() / sensivity));
    bin.y() = static_cast<int>(floor(point->eigen.y() / sensivity));
    bin.z() = static_cast<int>(floor(point->eigen.z() / sensivity));
    if(binDrawn[bin] == false)
    {
        std::stringstream ss;
        ss << "Bin: "<< "X:" << bin.x() << " Y:" << bin.y() << " Z:" << bin.z();
        binM.markers.push_back(VizHelperRVIZ::getBinMarker(bin, sensivity, ss.str(), 0, baseFrame, color));
        binDrawn[bin] = true;
    }
    return binM;
}


visualization_msgs::Marker VizHelperRVIZ::getBinMarker(Eigen::Vector3i bin, double bin_size,
                std::string desc, int id, std::string baseFrame, std_msgs::ColorRGBA color)
{
    return getBinMarker(bin.x(), bin.y(), bin.z(), bin_size, desc, id, baseFrame, color);
}

visualization_msgs::Marker VizHelperRVIZ::getBinMarker(int x, int y, int z, double bin_size,
                std::string desc, int id, std::string baseFrame, std_msgs::ColorRGBA color)
{
    Eigen::Vector3d bin(x,y,z);
    Eigen::Vector3d binCenter = bin * bin_size;
    return pointToCube(desc, id, baseFrame, binCenter.x(), binCenter.y(), binCenter.z(),
                bin_size, color);
}

std::vector<geometry_msgs::Point> VizHelperRVIZ::calculateAxesPoints(ISM::PointPtr point,ISM::QuaternionPtr quat, float scale)
{
    std::vector<geometry_msgs::Point> result;

    const Eigen::Quaternion<double> rot = ISM::GeometryHelper::quatToEigenQuat(quat);

    // Fix: wrong driection for X and Y. But why?
    const Eigen::Vector3d xV = rot._transformVector(-Eigen::Vector3d::UnitX());
    const Eigen::Vector3d yV = rot._transformVector(-Eigen::Vector3d::UnitY());
    const Eigen::Vector3d zV = rot._transformVector(Eigen::Vector3d::UnitZ());


    result.push_back(VizHelperRVIZ::createPoint(point->eigen.x() + xV.x() * scale, point->eigen.y() + xV.y() * scale, point->eigen.z() + xV.z() * scale));
    result.push_back(VizHelperRVIZ::createPoint(point->eigen.x() + yV.x() * scale, point->eigen.y() + yV.y() * scale, point->eigen.z() + yV.z() * scale));
    result.push_back(VizHelperRVIZ::createPoint(point->eigen.x() + zV.x() * scale, point->eigen.y() + zV.y() * scale, point->eigen.z() + zV.z() * scale));

    return result;
}




geometry_msgs::Point VizHelperRVIZ::createPoint(double x, double y, double z)
{
    geometry_msgs::Point retPoint;
    retPoint.x = x;
    retPoint.y = y;
    retPoint.z = z;
    return retPoint;
}

MarkerArray VizHelperRVIZ::createRingMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, float scale, ColorRGBA x, ColorRGBA y,
                                            ColorRGBA z, ColorRGBA a, ColorRGBA b, ColorRGBA g, double markerLifetime)
{
    MarkerArray retMarkers;

    const std::string arrowPath = "package://asr_ism_visualizations/rsc/arrow.dae";
    const std::string ringPath = "package://asr_ism_visualizations/rsc/ring.dae";

    const geometry_msgs::Point markerPoint =  pointToPointMsg(pose->point);

    const geometry_msgs::Quaternion qx = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 90.0, 0.0));
    const geometry_msgs::Quaternion qy = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, -90.0, 0.0, 0.0));
    const geometry_msgs::Quaternion qz = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 0.0, 0.0));


    Marker xMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 0, scale, scale, scale, x, markerLifetime);
    xMarker.pose.position = markerPoint;
    xMarker.pose.orientation = qx;
    xMarker.type = Marker::MESH_RESOURCE;
    xMarker.mesh_resource = arrowPath;
    retMarkers.markers.push_back(xMarker);

    Marker yMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 1, scale, scale, scale, y, markerLifetime);
    yMarker.pose.position = markerPoint;
    yMarker.pose.orientation = qy;
    yMarker.type = Marker::MESH_RESOURCE;
    yMarker.mesh_resource = arrowPath;
    retMarkers.markers.push_back(yMarker);

    Marker zMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 2, scale, scale, scale, z, markerLifetime);
    zMarker.pose.position = markerPoint;
    zMarker.pose.orientation = qz;
    zMarker.type = Marker::MESH_RESOURCE;
    zMarker.mesh_resource = arrowPath;
    retMarkers.markers.push_back(zMarker);

    Marker aMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 3, scale, scale, scale, a, markerLifetime);
    aMarker.pose.position = markerPoint;
    aMarker.pose.orientation = qx;
    aMarker.type = Marker::MESH_RESOURCE;
    aMarker.mesh_resource = ringPath;
    retMarkers.markers.push_back(aMarker);

    Marker bMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 4, scale, scale, scale, b, markerLifetime);
    bMarker.pose.position = markerPoint;
    bMarker.pose.orientation = qy;
    bMarker.type = Marker::MESH_RESOURCE;
    bMarker.mesh_resource = ringPath;
    retMarkers.markers.push_back(bMarker);

    Marker gMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 5, scale, scale, scale, g, markerLifetime);
    gMarker.pose.position = markerPoint;
    gMarker.pose.orientation = qz;
    gMarker.type = Marker::MESH_RESOURCE;
    gMarker.mesh_resource = ringPath;
    retMarkers.markers.push_back(gMarker);

    return retMarkers;
}
MarkerArray VizHelperRVIZ::createCubeArrow(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int& id, float scale,
                                           ColorRGBA cubeColor, ColorRGBA arrowColor, double markerLifetime)
{
    MarkerArray retMarkers;

    const std::string openCubeMeshResourcePath = "package://asr_ism_visualizations/rsc/openCube.dae";
    const std::string openPyramidMeshResourcePath = "package://asr_ism_visualizations/rsc/openPyramid.dae";


    Marker cube = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, ++id, scale, scale, scale, cubeColor, markerLifetime);
    cube.type = Marker::MESH_RESOURCE;
    cube.mesh_resource = openCubeMeshResourcePath;
    cube.pose.position = pointToPointMsg(pose->point);
    cube.pose.orientation = quatToQuaternionMsg(pose->quat);
    retMarkers.markers.push_back(cube);

    Marker arrow = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, ++id, scale, scale, scale, arrowColor, markerLifetime);
    arrow.type = Marker::MESH_RESOURCE;
    arrow.mesh_resource = openPyramidMeshResourcePath;
    arrow.pose.position = pointToPointMsg(pose->point);
    arrow.pose.orientation = quatToQuaternionMsg(pose->quat);
    retMarkers.markers.push_back(arrow);

    return retMarkers;
}    

geometry_msgs::Point VizHelperRVIZ::genCuboidPoint(int xp, int yp, int zp, double x, double y,
            double z, double xwidth, double ywidth, double zwidth)
{
    geometry_msgs::Point p;
    p.x = x + (double)xp * xwidth / (double) 2;
    p.y = y + (double)yp * ywidth / (double) 2;
    p.z = z + (double)zp * zwidth / (double) 2;
    return p;
}

Marker VizHelperRVIZ::pointToCuboid(std::string ns, int32_t id, std::string frame,
                double x, double y, double z, double xwitdh, double ywidth, double zwidth,
                std_msgs::ColorRGBA color)
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker pMark;

    pMark.header.frame_id = frame;

    pMark.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pMark.ns = ns;
    pMark.id = id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    pMark.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the marker action.  Options are ADD and DELETE
    pMark.action = visualization_msgs::Marker::ADD;

    boost::function< geometry_msgs::Point (int,int,int) > pointF = boost::bind(genCuboidPoint, _1, _2, _3, x, y, z, xwitdh, ywidth, zwidth);

    pMark.points.push_back(pointF(-1,-1,-1));
    pMark.points.push_back(pointF(-1,-1,1));
    pMark.points.push_back(pointF(-1,1,1));
    pMark.points.push_back(pointF(-1,1,-1));
    pMark.points.push_back(pointF(-1,-1,-1));
    pMark.points.push_back(pointF(1,-1,-1));
    pMark.points.push_back(pointF(1,-1,1));
    pMark.points.push_back(pointF(1,1,1));
    pMark.points.push_back(pointF(1,1,-1));
    pMark.points.push_back(pointF(1,-1,-1));
    pMark.points.push_back(pointF(1,1,-1));
    pMark.points.push_back(pointF(-1,1,-1));
    pMark.points.push_back(pointF(-1,1,1));
    pMark.points.push_back(pointF(1,1,1));
    pMark.points.push_back(pointF(1,-1,1));
    pMark.points.push_back(pointF(-1,-1,1));


    // Set the color -- be sure to set alpha to something non-zero!
    pMark.color = color;
    pMark.scale.x = 0.008;

    pMark.lifetime = ros::Duration();
    return pMark;

}

Marker VizHelperRVIZ::createCylinderLine(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, float scale, float length,
                                            ColorRGBA color, double markerLifetime)
{
    const std::string openCylinderMeshResourcePath = "package://asr_ism_visualizations/res/openCylinder.dae";


    Marker cylinder = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, 99, scale, scale, length, color, markerLifetime);
    cylinder.type = Marker::MESH_RESOURCE;
    cylinder.mesh_resource = openCylinderMeshResourcePath;
    cylinder.pose.position = pointToPointMsg(pose->point);
    cylinder.pose.orientation = quatToQuaternionMsg(pose->quat);

    return cylinder;
}
visualization_msgs::Marker VizHelperRVIZ::pointToCube(std::string ns, int32_t id, std::string frame, double x,
            double y, double z, double bin_size, std_msgs::ColorRGBA color)
{
    return pointToCuboid(ns, id, frame, x, y, z, bin_size, bin_size, bin_size, color);
}

visualization_msgs::MarkerArray VizHelperRVIZ::createPositionVector(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, ColorRGBA color, UniformDistributionGenerator* udg, int sampels, int axis, int id, float length, float openAngle, float sphereRadius, double markerLifetime){
    MarkerArray retMarkers;

    const std::string directionPath = "package://asr_ism_visualizations/rsc/rotCone.dae";

    const ISM::PointPtr markerPoint =  pose->point;

    for(int j = 0; j < sampels; j++){
        ISM::PointPtr tmpPoint =ISM::PointPtr(new ISM::Point());

        if(j > 0){
            double r_x, r_y, r_z;

            do{
                r_x = udg->operator ()();
                r_y = udg->operator ()();
                r_z = udg->operator ()();
            }while(std::sqrt(r_x * r_x + r_y * r_y + r_z * r_z) > sphereRadius);

            tmpPoint->eigen.x() = markerPoint->eigen.x() + r_x;
            tmpPoint->eigen.y() = markerPoint->eigen.y() + r_y;
            tmpPoint->eigen.z() = markerPoint->eigen.z() + r_z;

        }else{
            tmpPoint = pose->point;
        }
        retMarkers.markers.push_back(VizHelperRVIZ::createSphereMarker(tmpPoint, baseFrame, markerNamespace, id + j * 2, length, color, markerLifetime));


        Marker marker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id + j * 2 + 1, openAngle, openAngle, length, color, markerLifetime);
        marker.pose.position = pointToPointMsg(tmpPoint);
        if(axis == 1){
            marker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 90.0, 0.0));
        }else if(axis == 2){
            marker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, -90.0, 0.0, 0.0));
        }else if(axis == 3){
            marker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 0.0, 0.0));
        }else{
            marker.pose.orientation = quatToQuaternionMsg(pose->quat);
        }
        marker.type = Marker::MESH_RESOURCE;
        marker.mesh_resource = directionPath;
        retMarkers.markers.push_back(marker);
    }
    return retMarkers;
}


visualization_msgs::MarkerArray VizHelperRVIZ::createCoordinateMarkerWithAngle(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float length, float openAngle, double markerLifetime){
    MarkerArray retMarkers;

    const std::string directionPath = "package://asr_ism_visualizations/rsc/rotCone.dae";
    const geometry_msgs::Point markerPoint =  pointToPointMsg(pose->point);

    Marker rMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, openAngle, openAngle, length, VizHelperRVIZ::createColorRGBA(1.0, 0.0, 0.0, 1.0), markerLifetime);
    rMarker.pose.position = markerPoint;
    rMarker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 90.0, 0.0));
    rMarker.type = Marker::MESH_RESOURCE;
    rMarker.mesh_resource = directionPath;
    retMarkers.markers.push_back(rMarker);

    Marker gMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id+1, openAngle, openAngle, length, VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 1.0), markerLifetime);
    gMarker.pose.position = markerPoint;
    gMarker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, -90.0, 0.0, 0.0));
    gMarker.type = Marker::MESH_RESOURCE;
    gMarker.mesh_resource = directionPath;
    retMarkers.markers.push_back(gMarker);

    Marker bMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id+2, openAngle, openAngle, length, VizHelperRVIZ::createColorRGBA(0.0, 0.0, 1.0, 1.0), markerLifetime);
    bMarker.pose.position = markerPoint;
    bMarker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 0.0, 0.0));
    bMarker.type = Marker::MESH_RESOURCE;
    bMarker.mesh_resource = directionPath;
    retMarkers.markers.push_back(bMarker);

    return retMarkers;
}
visualization_msgs::MarkerArray VizHelperRVIZ::createCoordinateMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float length, float radius, double markerLifetime){
    MarkerArray retMarkers;

    const std::string directionPath = "package://asr_ism_visualizations/rsc/transCylinder.dae";
    const geometry_msgs::Point markerPoint =  pointToPointMsg(pose->point);

    Marker rMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, radius, radius, length, VizHelperRVIZ::createColorRGBA(1.0, 0.0, 0.0, 1.0), markerLifetime);
    rMarker.pose.position = markerPoint;
    rMarker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 90.0, 0.0));
    rMarker.type = Marker::MESH_RESOURCE;
    rMarker.mesh_resource = directionPath;
    retMarkers.markers.push_back(rMarker);

    Marker gMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id+1, radius, radius, length, VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 1.0), markerLifetime);
    gMarker.pose.position = markerPoint;
    gMarker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, -90.0, 0.0, 0.0));
    gMarker.type = Marker::MESH_RESOURCE;
    gMarker.mesh_resource = directionPath;
    retMarkers.markers.push_back(gMarker);

    Marker bMarker = createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id+2, radius, radius, length, VizHelperRVIZ::createColorRGBA(0.0, 0.0, 1.0, 1.0), markerLifetime);
    bMarker.pose.position = markerPoint;
    bMarker.pose.orientation = quatToQuaternionMsg(ISM::GeometryHelper::getQuatFromRPY(pose->quat, 0.0, 0.0, 0.0));
    bMarker.type = Marker::MESH_RESOURCE;
    bMarker.mesh_resource = directionPath;
    retMarkers.markers.push_back(bMarker);

    return retMarkers;
}
visualization_msgs::Marker VizHelperRVIZ::createLine(ISM::PointPtr fromPoint, ISM::PointPtr toPoint, float width, int id, std::string baseFrame, std::string markerNamespace, std_msgs::ColorRGBA color, double markerLifetime){
    Marker retMarker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(baseFrame, markerNamespace, id, width, 0, 0, color, markerLifetime);
    retMarker.type = Marker::LINE_STRIP;
    retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(fromPoint));
    retMarker.points.push_back(VizHelperRVIZ::pointToPointMsg(toPoint));
    return retMarker;
}


}
