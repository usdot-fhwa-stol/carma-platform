<?php
  $PID_FILE_PATH = "/opt/carma/launch.pid"; // File containing process id of roslaunch
  if (!file_exists($PID_FILE_PATH)) { // If this file exists roslaunch is running
   header("Location: ../main.html"); // Move back main.html
   exit;
  }
  // Get roslaunch pid
  $launch_pid = shell_exec("cat " . $PID_FILE_PATH);
  echo("Launch PID: " . $launch_pid);
  // Kill roslaunch
  $bash_pid = shell_exec("sudo -u mcconnelms /var/www/html/scripts/kill.bash " . $launch_pid . "> /dev/null 2>&1 & echo $!");
  // Move to logout page
  header("Location: ../logout.html"); // Move onto main.html
?>