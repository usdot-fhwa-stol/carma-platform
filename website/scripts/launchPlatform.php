<?php
  $PID_FILE_PATH = "/opt/carma/launch.pid"; // File containing process id of roslaunch
  if (file_exists($PID_FILE_PATH)) { // If this file exists roslaunch is running
   header("Location: ../main.html"); // Move onto main.html
   exit;
  }
  // Launch Platform
  // > /dev/null 2>&1 & echo $! (suppresses output and captures the bash file pid)
  $bash_pid = shell_exec("sudo -u mcconnelms /var/www/html/scripts/launch.bash > /dev/null 2>&1 & echo $!");

  sleep(5); // Need delay before trying to use rosbridge. TODO move to rosbridge.js
  // Switch to main.html
  header("Location: ../main.html");
  // How to kill roslaunch
  // $launch_pid = shell_exec("cat " . $PID_FILE_PATH);
  //exec("kill -INT " . $launch_pid);
?>