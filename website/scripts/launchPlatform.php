<?php

  //  Copyright (C) 2018 LEIDOS.
  // 
  //  Licensed under the Apache License, Version 2.0 (the "License"); you may not
  //  use this file except in compliance with the License. You may obtain a copy of
  //  the License at
  // 
  //  http://www.apache.org/licenses/LICENSE-2.0
  // 
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations under
  //  the License.

  //Check if remotelaunch parameter is selected. 
  if (!isset($_GET['remotelaunch'])){
    // Switch to main.html
    header("Location: ../main.html");
    exit;
  }

  // Launch Platform
  shell_exec("/var/www/html/scripts/launch.bash");

  sleep(10); // Need delay before trying to use rosbridge. TODO move to rosbridge.js
  // Switch to main.html
  header("Location: ../main.html");
?>
