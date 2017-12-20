<?php
  $PID_FILE_PATH = "/opt/carma/launch.pid"; // File containing process id of roslaunch
  if (file_exists($PID_FILE_PATH)) { // If this file exists roslaunch is running
   header("Location: ../main.html"); // Move onto main.html
   exit;
  }
  // Launch Platform
  $bash_pid = shell_exec("sudo -u mcconnelms /var/www/html/scripts/launch.bash > /dev/null 2>&1 & echo $!");

  // Switch to main.html
  header("Location: ../main.html");
  // How to kill roslaunch
  // $launch_pid = shell_exec("cat " . $PID_FILE_PATH);
  //exec("kill -INT " . $launch_pid);
?>