#pragma once
#include <string>
#include <iostream>


struct NDTConfig {
    // Constant Parameters
    int max_iter = 30;        // Maximum iterations
    double resolution = 1.0;  // Resolution in m
    double step_size = 0.1;   // Step size in (?)
    double trans_eps = 0.01;  // Transformation epsilon in m
    double rot_eps = 0.99862953358; // cos(angle) in axis angle representation of rotation. arccos(rot_eps) gives the angle



    std::string baselink_frame_id_ = "base_link";
    std::string map_frame_id_ = "map";

    friend std::ostream &operator<<( std::ostream& output, const NDTConfig& obj ) { 
        output << "NDTConfig : { max_iter: " << obj.max_iter << ", resolution: " << obj.resolution
        << ", step_size: " << obj.step_size << ", trans_eps: " << obj.trans_eps << ", rot_eps: " << obj.rot_eps " }";
        return output;            
    }

};