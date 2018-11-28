<?php
  //Check if remotelaunch parameter is selected. 
  if (!isset($_GET['remotelaunch'])){
    // Switch to main.html
    header("Location: ../main.html");
    exit;
  }

  // Launch Platform
  // > /dev/null 2>&1 & echo $! (suppresses output and captures the bash file pid)
  shell_exec("/var/www/html/scripts/launch.bash");

  sleep(5); // Need delay before trying to use rosbridge. TODO move to rosbridge.js
  // Switch to main.html
  header("Location: ../main.html");
?>
