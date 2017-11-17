
#include <carma_transform_server/TransformServer.h>

// Main execution
int main(int argc, char** argv){

  // Initialize node
  ros::init(argc, argv, "transform_server");
  TransformServer tf_server(argc, argv);

  tf_server.run();
  return 0;
};