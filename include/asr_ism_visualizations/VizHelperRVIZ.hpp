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

//Global includes
#include <Eigen/Geometry>
#include <ros/ros.h>

//Global includes

//Package includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//ISM includes
#include "ISM/recognizer/VotingSpace.hpp"

#include <ISM/common_type/Point.hpp>
#include <ISM/common_type/Quaternion.hpp>
#include <ISM/common_type/Pose.hpp>
#include <ISM/common_type/Track.hpp>
#include <ISM/common_type/VoteSpecifier.hpp>

#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
//Local includes


namespace VIZ
{	
    typedef boost::variate_generator<boost::mt19937, boost::uniform_real<> > UniformDistributionGenerator;
    typedef visualization_msgs::Marker Marker;
    typedef visualization_msgs::MarkerArray MarkerArray;
    typedef std_msgs::ColorRGBA ColorRGBA;


    struct cmpVector3i 
    {
        bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
            if(a.x() != b.x() || a.y() != b.y() || a.z() != b.z())
            {
                return true;
            }
            return false;
        }
    };

	class VizHelperRVIZ {
		public:
		
			static geometry_msgs::Point pointToPointMsg(ISM::PointPtr point);
            static geometry_msgs::Quaternion quatToQuaternionMsg(ISM::QuaternionPtr quat);
            static std::vector<ISM::PosePtr> posesFromTrack(ISM::TrackPtr track);
            static geometry_msgs::Point EVector3DToPointMsg(ISM::PointPtr point, Eigen::Vector3d v3d);

			/**
			* \param red, green, blue, alpha in [0.0,1.0]
			*/
            static ColorRGBA createColorRGBA(float red, float green, float blue, float alpha);
            static ColorRGBA createColorRGBA(std::vector<float> rgba);

            /**
			*	\param hue in [0.0,360.0]; saturation and value in [0.0,1.0]
			*/
            static ColorRGBA hsvToRGBA(double hue, double saturation, double value);

            /**
			*	\param confidence in [0.0,1.0]
			*/
            static ColorRGBA confidenceToColor(double confidence);

            static ColorRGBA getColorOfObject(const ISM::Object& object);
            static ColorRGBA getColorOfObject(const ISM::ObjectPtr object_ptr);

            /**
            *	\param markerLifetime if 0 then the marker will not deleted automatically, else markerLifetime will be interpreted as seconds.
            */
            static Marker createMarkerWithoutTypeAndPose(std::string baseFrame, std::string markerNamespace, int id, float xScale, float yScale, float zScale, ColorRGBA color, double markerLifetime);
			
            static Marker createSphereMarker(ISM::PointPtr point, std::string baseFrame, std::string markerNamespace, int id, float radius, ColorRGBA color, double markerLifetime);

            static Marker createConeMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float radius, ColorRGBA color, double markerLifetime);

            static Marker createCylinderMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float radius, float height, ColorRGBA color, double markerLifetime);

            static Marker createArrowMarkerToPoint(ISM::PointPtr fromPoint, ISM::PointPtr to, std::string baseFrame, std::string markerNamespace, int id, float xScale, float yScale, float zScale, ColorRGBA color, double markerLifetime);

            static Marker createArrowMarkerToPoint(ISM::PointPtr fromPoint, ISM::PointPtr to, std::string baseFrame, std::string markerNamespace, int id, float xScale, float yScale, float zScale, ColorRGBA color, double markerLifetime, double minLength);

            static Marker createMeshMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, ColorRGBA color, double markerLifetime, std::string resourcePath);
			
            static MarkerArray createTrackMarkers(std::vector<ISM::PosePtr> trackPoses, std::string baseFrame, std::string markerNamespace, float lineWidth, ColorRGBA color, double markerLifetime);

            static visualization_msgs::MarkerArray createSphereMarkerWithOrientation
                (ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id,
                    float radius, std_msgs::ColorRGBA color, double markerLifetime, int axesFirstId);

            static Marker createLineMarkerFormPair(std::vector<std::pair<ISM::PointPtr, ISM::PointPtr>> pointPair, std::string baseFrame, std::string markerNamespace, float arrowScale, ColorRGBA color, double markerLifetime);

            static std::vector<geometry_msgs::Point> calculateAxesPoints(ISM::PointPtr point, ISM::QuaternionPtr quat, float scale);

            static MarkerArray createCoordinateArrow(std::vector<std::pair<ISM::PointPtr, std::vector<ISM::PosePtr> > > poses_vec, std::string baseFrame, std::string markerNamespace, float arrowScale, float axisScale, ColorRGBA color, double markerLifetime);

            static MarkerArray createCoordinateMarkerWithAngle(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float length, float openAngle, double markerLifetime);

            static MarkerArray createRingMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, float scale, ColorRGBA x, ColorRGBA y, ColorRGBA z, ColorRGBA a, ColorRGBA b, ColorRGBA g, double markerLifetime);

            static Marker createLineArrow(ISM::PointPtr fromPoint, std::vector<ISM::PosePtr> toPoses, std::string baseFrame, std::string markerNamespace, int &id, float arrowScale, ColorRGBA color, double markerLifetime);

            static MarkerArray createCubeArrow(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int &id, float scale, ColorRGBA cubeColor, ColorRGBA arrowColor, double markerLifetime);

            static Marker createCylinderLine(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, float scale, float length, ColorRGBA color, double markerLifetime);

            static MarkerArray createPositionVector(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, ColorRGBA color, UniformDistributionGenerator* udg, int sampels, int axis, int id, float length, float openAngle, float sphereRadius, double markerLifetime);

            static geometry_msgs::Point createPoint(double x, double y, double z);

            static visualization_msgs::MarkerArray createArrowMarkerFromVote(ISM::VoteSpecifierPtr vote,
                ISM::ObjectPtr o, std::string baseFrame, std::string ns, int32_t id, double bin_size,
                std_msgs::ColorRGBA binColor, std_msgs::ColorRGBA voteColor = VizHelperRVIZ::createColorRGBA(1.0, 0.0, 1.0, 0.5));
            static MarkerArray createCoordinateMarker(ISM::PosePtr pose, std::string baseFrame, std::string markerNamespace, int id, float length, float radius, double markerLifetime);

            static Marker pointToCuboid(std::string ns, int32_t id, std::string frame,
                double x, double y, double z, double xwitdh, double ywidth, double zwidth,
                std_msgs::ColorRGBA color);

            static geometry_msgs::Point genCuboidPoint(int xp, int yp, int zp, double x, double y, double z, double xwidth, double ywidth, double zwidth);

            static Marker pointToCube(std::string ns, int id, std::string frame,
                double x, double y, double z, double bin_size, std_msgs::ColorRGBA color);

            static Marker getBinMarker(Eigen::Vector3i bin, double bin_size,
                            std::string desc, int id, std::string baseFrame, std_msgs::ColorRGBA color);

            static Marker getBinMarker(int x, int y, int z, double bin_size,
                std::string desc, int id, std::string baseFrame, std_msgs::ColorRGBA color);

            static MarkerArray getBinFromPoint (const ISM::PointPtr point,
                double sensivity, std::string baseFrame, std_msgs::ColorRGBA color);

            static std::map<Eigen::Vector3i, bool, cmpVector3i > binDrawn;

            static Marker createLine(ISM::PointPtr fromPoint, ISM::PointPtr toPoint, float width, int id, std::string baseFrame, std::string markerNamespace, std_msgs::ColorRGBA color, double markerLifetime);
    };
}
