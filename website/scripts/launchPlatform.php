<?php
  //Check if remotelaunch parameter is selected. 
  if (!isset($_GET['remotelaunch'])){
    // Switch to main.html
    header("Location: ../main.html");
    exit;
  }

  // Set paths
  $CARMA_DIR = "/opt/carma";
  $HTML_DIR = $CARMA_DIR . "/app/html";

  $PID_FILE_PATH = $CARMA_DIR . "/launch.pid"; // File containing process id of roslaunch
  if (file_exists($PID_FILE_PATH)) { // If this file exists roslaunch is running
   header("Location: ../main.html"); // Move onto main.html
   exit;
  }
  // Launch Platform
  // > /dev/null 2>&1 & echo $! (suppresses output and captures the bash file pid)
  $bash_pid = shell_exec("sudo -u carma_launcher " . $HTML_DIR . "/scripts/launch.bash > /dev/null 2>&1 & echo $!");

  sleep(5); // Need delay before trying to use rosbridge. TODO move to rosbridge.js
  // Switch to main.html
  header("Location: ../main.html");
?>