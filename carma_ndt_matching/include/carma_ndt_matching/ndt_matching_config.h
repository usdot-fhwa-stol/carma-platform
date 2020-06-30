#include <string>

struct NDTConfig {
    // Constant Parameters
    int max_iter = 30;        // Maximum iterations
    float resolution = 1.0;      // Resolution
    double step_size = 0.1;   // Step size
    double trans_eps = 0.01;  // Transformation epsilon


    std::string baselink_frame_id_ = "base_link";
    std::string map_frame_id_ = "map";

};