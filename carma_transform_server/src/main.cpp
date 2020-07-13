/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <carma_transform_server/TransformServer.h>

// Main execution
int main(int argc, char** argv){

  // Initialize node
  ros::init(argc, argv, "transform_server");
  TransformServer tf_server(argc, argv);

  // Start execution
  tf_server.run();
  return 0;
};