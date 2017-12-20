<?php
  $PID_FILE_PATH = "/opt/carma/launch.pid"; // File containing process id of roslaunch
  if (!file_exists($PID_FILE_PATH)) { // If this file exists roslaunch is running
   header("Location: ../main.html"); // Move back main.html
   exit;
  }
  // Kill roslaunch
  $launch_pid = shell_exec("cat " . $PID_FILE_PATH);
  exec("kill -INT " . $launch_pid);
  header("Location: ../logout.html"); // Move onto main.html
  exit;
?>