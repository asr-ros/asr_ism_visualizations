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
#include <ISM/common_type/Pose.hpp>
#include <asr_msgs/AsrAttributedPointCloud.h>

//ROS includes
#include <dynamic_reconfigure/server.h>
#include <asr_ism_visualizations/rp_pose_prediction_visualizerConfig.h>

//foreigen includes
#include <boost/shared_ptr.hpp>

namespace VIZ
{
class PosePredictionVisualizer: public VisualizerRVIZ
{
public:
    PosePredictionVisualizer(ros::Publisher posePredictionMarkerPublisher, const ros::NodeHandle& nh): VisualizerRVIZ(posePredictionMarkerPublisher) {
        reconfigure_server_ = new dynamic_reconfigure::Server<asr_ism_visualizations::rp_pose_prediction_visualizerConfig>(nh);
        dynamic_reconfigure::Server<asr_ism_visualizations::rp_pose_prediction_visualizerConfig>::CallbackType reconf_callback = boost::bind(&PosePredictionVisualizer::dynamicReconfCallback, this, _1, _2);
        reconfigure_server_->setCallback(reconf_callback);
    }

    ~PosePredictionVisualizer(){
        reconfigure_server_->clearCallback();
        delete reconfigure_server_;
    }

    void addPosePredictionVisualization(ISM::PosePtr referencePosePtr, asr_msgs::AsrAttributedPointCloud attributedPointCloud, std::string markerNameSpace);

    void dynamicReconfCallback(asr_ism_visualizations::rp_pose_prediction_visualizerConfig &config, uint32_t level) {
        ROS_DEBUG_STREAM("Pose prediction visualizer dyn-params updated.");
        baseFrame = config.baseFrame;
        pointLifeTime = config.pointLifeTime;
        arrowLifeTime = config.arrowLifeTime;
        referenceRadius = config.referenceRadius;
        pointRadius = config.pointRadius;
        arrowScaleX = config.arrowScaleX;
        arrowScaleY = config.arrowScaleY;
        arrowScaleZ = config.arrowScaleZ;
        colorStepSize = config.colorStepSize;

    }

private:
    typedef visualization_msgs::MarkerArray MarkerArray;
    ros::Publisher posePredictionMarkerPublisher;
    std::string baseFrame;
    double pointLifeTime;
    double arrowLifeTime;
    double referenceRadius;
    double pointRadius;
    double arrowScaleX;
    double arrowScaleY;
    double arrowScaleZ;
    double colorStepSize;

    dynamic_reconfigure::Server<asr_ism_visualizations::rp_pose_prediction_visualizerConfig>* reconfigure_server_;

};

typedef boost::shared_ptr<PosePredictionVisualizer> PosePredictionVisualizerPtr;
}
