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
#include <asr_ism_visualizations/ism_voting_visualizer_rviz.hpp>

//other includes
#include <map>
#include <tuple>


namespace VIZ
{
void ISMVotingVisualizerRVIZ::addVisualization(const ISM::PatternNameAndVotingSpaceTuple& voting_space, std::vector<ISM::RecognitionResultPtr> results)
{
    ISM::RecognitionResultPtr result;
    for(std::size_t i = 0; i < results.size(); i++){
        if(results[i]->patternName == scene_name_){
            result = results[i];
            break;
        }
    }
    if(result == NULL){
        ROS_ERROR("Recognition Result doesn't contain result of pattern %s.\nSkipping visualization.", pattern_name_.c_str());
        return;
    }

    if(voting_space.first.compare(pattern_name_) != 0){
        ROS_ERROR("Votingspace doesn't contain pattern %s.\nSkipping visualization.", pattern_name_.c_str());
        return;
    }
    ROS_INFO("Drawing votes of scene \"%s\" pattern \"%s\"", scene_name_.c_str(), pattern_name_.c_str());

    ISM::VotingSpacePtr matching_voting_space_ptr = voting_space.second;
    std::map<ISM::ObjectPtr, std::vector<ISM::PosePtr>> voter_to_poses_map = getSortedVotedPoses(matching_voting_space_ptr);
    std::vector<ISM::PosePtr> poses_ptr;
    for(auto iter: voter_to_poses_map){
        poses_ptr.insert(poses_ptr.end(), iter.second.begin(), iter.second.end());
    }


    MarkerArray bin_markers = generateBinAndGridMarker(matching_voting_space_ptr);
    MarkerArray vote_markers = generateVoteMarker(voter_to_poses_map);
    MarkerArray ref_markers = generateRefMarker(getRecognitionResultPtrOfPattern(result), poses_ptr);

    addMarker(vote_markers);
    addMarker(bin_markers);
    addMarker(ref_markers);
    publishCollectedMarkers();

}
Marker ISMVotingVisualizerRVIZ::generateResultVoteOverlay(ISM::RecognitionResultPtr result){
    Marker ret_marker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(base_frame_, "result_votes_overlay", 0, line_scale_, 0, 0, VizHelperRVIZ::createColorRGBA(0.0, 0.0, 0.0, 1.0), marker_lifetime_);
    ret_marker.type = Marker::LINE_LIST;

    for (ISM::SummarizedVotedPosePtr& summarized_vote : result->summarizedVotes)
    {
        ret_marker.points.push_back(VizHelperRVIZ::pointToPointMsg(summarized_vote.first->pose->point));
        ret_marker.points.push_back(VizHelperRVIZ::pointToPointMsg(summarized_vote.first->source->pose->point));
    }

    return ret_marker;
}


MarkerArray ISMVotingVisualizerRVIZ::generateRefMarker(ISM::RecognitionResultPtr result, std::vector<ISM::PosePtr> voted_poses){
    MarkerArray ret_markers;
    ColorRGBA confidence_color = VizHelperRVIZ::confidenceToColor(result->confidence);
    confidence_color.a = confidence_sphere_opacity_;

    ret_markers.markers.push_back(VizHelperRVIZ::createCylinderMarker(result->referencePose, base_frame_, "reference", 0,
                                                                      object_marker_scale_, object_marker_scale_, VizHelperRVIZ::createColorRGBA(1.0, 1.0, 1.0, object_marker_opacity_), marker_lifetime_));
    ret_markers.markers.push_back(VizHelperRVIZ::createSphereMarker(result->referencePose->point, base_frame_, "confidence_sphere", 0,
                                                                    sqrt(3) * bin_size_, confidence_color, marker_lifetime_));
    int id = 0;
    ret_markers.markers.push_back(VizHelperRVIZ::createLineArrow(result->referencePose->point, voted_poses, base_frame_, "votesToReferenceLines", id,
                                                                 line_scale_ * 0.1, VizHelperRVIZ::createColorRGBA(0.0, 0.0, 0.0, 0.15), marker_lifetime_));
    ret_markers.markers.push_back(generateResultVoteOverlay(result));

    return ret_markers;
}

MarkerArray ISMVotingVisualizerRVIZ::generateBinAndGridMarker(ISM::VotingSpacePtr voting_space_ptr){
    MarkerArray ret_markers;
    std::map<std::tuple<double, double, double>, int> bin_to_votecount_map;

    //tuple<minX, minY, minZ, maxX, maxY, maxZ>
    std::tuple<int, int, int, int, int, int> grid_bb {INT_MAX, INT_MAX, INT_MAX, -INT_MAX, -INT_MAX, -INT_MAX};
    double x;
    double y;
    double z;
    ISM::XIndexToYIndex grid = voting_space_ptr->voteGrid;

    for(ISM::XIndexToYIndex::iterator x_iter = grid.begin(); x_iter!=grid.end(); ++x_iter){
        for(ISM::YIndexToZIndex::iterator y_iter = x_iter->second.begin(); y_iter!=x_iter->second.end(); ++y_iter){
            for(ISM::ZIndexToVotingBinPtr::iterator z_iter = y_iter->second.begin(); z_iter!=y_iter->second.end(); ++z_iter){
                for(ISM::TypeToInnerMap::iterator type_iter = z_iter->second->votes.begin(); type_iter != z_iter->second->votes.end(); ++type_iter){
                    for(ISM::IdToVoteMap::iterator map_iter = type_iter->second.begin(); map_iter !=type_iter->second.end(); ++map_iter){
                        x = x_iter->first * bin_size_;
                        y = y_iter->first * bin_size_;
                        z = z_iter->first * bin_size_;

                        std::get<0>(grid_bb) = std::min(x_iter->first, (int)std::get<0>(grid_bb));
                        std::get<1>(grid_bb) = std::min(y_iter->first, (int)std::get<1>(grid_bb));
                        std::get<2>(grid_bb) = std::min(z_iter->first, (int)std::get<2>(grid_bb));


                        std::get<3>(grid_bb) = std::max(x_iter->first, std::get<3>(grid_bb));
                        std::get<4>(grid_bb) = std::max(y_iter->first, std::get<4>(grid_bb));
                        std::get<5>(grid_bb) = std::max(z_iter->first, std::get<5>(grid_bb));


                        if(bin_to_votecount_map.find(std::make_tuple(x, y, z)) == bin_to_votecount_map.end()){
                            bin_to_votecount_map.insert(std::make_pair(std::make_tuple(x, y, z), 0));
                        }
                        bin_to_votecount_map[std::make_tuple(x,y,z)] += map_iter->second.size();
                    }
                }
            }
        }
    }

    int max_count = (int)-INFINITY;
    int min_count = (int)INFINITY;

    int i = 0;
    for(std::map<std::tuple<double, double, double>, int>::iterator iter=bin_to_votecount_map.begin(); iter!=bin_to_votecount_map.end(); ++iter) {
        if(iter->second > max_count){
            max_count = iter->second;
        }
        if(iter->second < min_count){
            min_count = iter->second;
        }
    }
    ColorRGBA color;
    Marker temp_marker;
    for(std::map<std::tuple<double, double, double>, int>::iterator iter  = bin_to_votecount_map.begin(); iter!=bin_to_votecount_map.end(); ++iter) {
        color = VizHelperRVIZ::hsvToRGBA((120 * (min_count + iter->second) / max_count), 1.0, 1.0);
        color.a = bin_opacity_;

        double scale = bin_size_ * bin_scale_;

        temp_marker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(base_frame_, "binMarker", i, scale, scale, scale, color, marker_lifetime_);
        temp_marker.type = Marker::CUBE;
        temp_marker.action = Marker::ADD;
        temp_marker.pose.position = VizHelperRVIZ::createPoint(std::get<0>(iter->first), std::get<1>(iter->first), std::get<2>(iter->first));
        ret_markers.markers.push_back(temp_marker);

        i++;
    }

    ret_markers.markers.push_back(generateGridMarker(grid_bb));
    return ret_markers;
}


MarkerArray ISMVotingVisualizerRVIZ::generateVoteMarker(std::map<ISM::ObjectPtr, std::vector<ISM::PosePtr>> voter_to_poses_map){
    MarkerArray ret_markers;
    MarkerArray temp_arrow_markers;
    Marker temp_voter_marker;
    std_msgs::ColorRGBA vote_color;
    std::string voter_namespace;

    std::map<std::string, std::vector< std::pair<ISM::PointPtr, std::vector<ISM::PosePtr> > > > united_votes_of_voter;

    for(auto iter: voter_to_poses_map)
    {
        if(united_votes_of_voter.find(iter.first->type) == united_votes_of_voter.end()){
            std::vector< std::pair<ISM::PointPtr, std::vector<ISM::PosePtr> > > temp_vector;
            united_votes_of_voter[iter.first->type] = temp_vector;
        }
        united_votes_of_voter[iter.first->type].push_back(std::make_pair(iter.first->pose->point, iter.second));
    }


    int i = 1;
    for(auto iter: united_votes_of_voter)
    {
        voter_namespace = "votes_of_" + iter.first;
        vote_color = VizHelperRVIZ::hsvToRGBA(120 + (240.0 / (united_votes_of_voter.size() + 1)) * i , 1.0, 1.0);
        vote_color.a = vote_opacity_;

        temp_arrow_markers = VizHelperRVIZ::createCoordinateArrow(iter.second, base_frame_, voter_namespace,
                                                                  line_scale_, axis_scale_, vote_color, marker_lifetime_);
        ret_markers.markers.insert(ret_markers.markers.end(), temp_arrow_markers.markers.begin(), temp_arrow_markers.markers.end());

        for(size_t j = 0; j < iter.second.size(); j++){
            temp_voter_marker = VizHelperRVIZ::createSphereMarker(iter.second[j].first, base_frame_, iter.first, j,
                                                                  object_marker_scale_, vote_color, marker_lifetime_);
            temp_voter_marker.color.a = 1.0;
            ret_markers.markers.push_back(temp_voter_marker);
        }

        i++;
    }

    return ret_markers;
}

Marker ISMVotingVisualizerRVIZ::generateGridMarker(std::tuple<int, int, int, int, int, int> grid_bb){
    Marker ret_marker = VizHelperRVIZ::createMarkerWithoutTypeAndPose(base_frame_, "voxel_grid", 0, grid_scale_, 0, 0,
                                                                      VizHelperRVIZ::createColorRGBA(0.5, 0.5, 0.5, grid_opacity_), marker_lifetime_);
    ret_marker.type = Marker::LINE_LIST;


    double x_min = std::get<0>(grid_bb) * bin_size_ - bin_size_ / 2.0;
    double y_min = std::get<1>(grid_bb) * bin_size_ - bin_size_ / 2.0;
    double z_min = std::get<2>(grid_bb) * bin_size_ - bin_size_ / 2.0;

    double x_max = std::get<3>(grid_bb) * bin_size_ + bin_size_ / 2.0;
    double y_max = std::get<4>(grid_bb) * bin_size_ + bin_size_ / 2.0;
    double z_max = std::get<5>(grid_bb) * bin_size_ + bin_size_ / 2.0;

    //prevents missing lines caused by double comparison inaccuracy
    double epsilon = grid_scale_ * 0.01;


    for(double y = y_min; y < y_max + epsilon; y += bin_size_){
        for(double z = z_min; z < z_max + epsilon; z += bin_size_){
            ret_marker.points.push_back(VizHelperRVIZ::createPoint(x_max, y, z));
            ret_marker.points.push_back(VizHelperRVIZ::createPoint(x_min, y, z));
        }
    }
    for(double x = x_min; x < x_max + epsilon; x += bin_size_){
        for(double z = z_min; z < z_max + epsilon; z += bin_size_){
            ret_marker.points.push_back(VizHelperRVIZ::createPoint(x ,y_max, z));
            ret_marker.points.push_back(VizHelperRVIZ::createPoint(x, y_min, z));
        }
    }

    for(double x = x_min; x < x_max + epsilon; x += bin_size_){
        for(double y = y_min; y < y_max + epsilon; y += bin_size_){
            ret_marker.points.push_back(VizHelperRVIZ::createPoint(x, y, z_max));
            ret_marker.points.push_back(VizHelperRVIZ::createPoint(x, y, z_min));
        }
    }

    return ret_marker;
}


std::map<ISM::ObjectPtr, std::vector<ISM::PosePtr>> ISMVotingVisualizerRVIZ::getSortedVotedPoses(ISM::VotingSpacePtr voting_space_ptr)
{
    std::map<ISM::ObjectPtr, std::vector<ISM::PosePtr>> voter_to_poses_map;
    ISM::XIndexToYIndex grid = voting_space_ptr->voteGrid;

    for(ISM::XIndexToYIndex::iterator x_iter = grid.begin(); x_iter!=grid.end(); ++x_iter){
        for(ISM::YIndexToZIndex::iterator y_iter = x_iter->second.begin(); y_iter != x_iter->second.end(); ++y_iter){
            for(ISM::ZIndexToVotingBinPtr::iterator z_iter = y_iter->second.begin(); z_iter != y_iter->second.end(); ++z_iter){
                for(ISM::TypeToInnerMap::iterator type_iter = z_iter->second->votes.begin(); type_iter != z_iter->second->votes.end(); ++type_iter){
                    for(ISM::IdToVoteMap::iterator map_iter = type_iter->second.begin(); map_iter != type_iter->second.end(); ++map_iter){
                        for(ISM::VotedPosePtrs::iterator vote_iter = map_iter->second.begin(); vote_iter != map_iter->second.end(); ++vote_iter){
                            //Maps votes to Voters
                            if(voter_to_poses_map.find((*vote_iter)->source) != voter_to_poses_map.end()){
                                voter_to_poses_map[(*vote_iter)->source].push_back((*vote_iter)->pose);
                            }else{
                                std::vector<ISM::PosePtr> temp_vec;
                                voter_to_poses_map.insert(std::make_pair((*vote_iter)->source, temp_vec));
                                voter_to_poses_map[(*vote_iter)->source].push_back((*vote_iter)->pose);
                            }
                        }
                    }
                }
            }
        }
    }
    return voter_to_poses_map;
}

ISM::RecognitionResultPtr ISMVotingVisualizerRVIZ::getRecognitionResultPtrOfPattern(ISM::RecognitionResultPtr result){
    if(result->patternName == pattern_name_){
        return result;
    }else{
        for(ISM::RecognitionResultPtr subPattern : result->subPatterns) {
            ISM::RecognitionResultPtr temp_recognition_result_ptr = getRecognitionResultPtrOfPattern(subPattern);
            if(temp_recognition_result_ptr != NULL){
                return temp_recognition_result_ptr;
            }
        }
    }
    return NULL;
}

}
