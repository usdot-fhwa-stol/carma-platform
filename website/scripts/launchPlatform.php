<?php
  //Check if remotelaunch parameter is selected. 
  if (!isset($_GET['remotelaunch'])){
    // Switch to main.html
    header("Location: ../main.html");
    exit;
  }

  $rosBagRecord="true";
  if (!isset($_GET['rosbagrecorder'])){
    $rosBagRecord="false";
  }
  // Launch Platform
  // > /dev/null 2>&1 & echo $! (suppresses output and captures the bash file pid)
  shell_exec('docker-compose -f /opt/carma/vehicle/docker-compose.yml -d up');

  sleep(5); // Need delay before trying to use rosbridge. TODO move to rosbridge.js
  // Switch to main.html
  header("Location: ../main.html");
?>
