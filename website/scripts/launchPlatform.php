<?php
  //Check if remotelaunch parameter is selected. 
  if (!isset($_GET['remotelaunch'])){
    // Switch to main.html
    header("Location: ../main.html");
    exit;
  }

  // Launch Platform
  shell_exec("/var/www/html/scripts/launch.bash");

  sleep(30); // Need delay before trying to use rosbridge. TODO move to rosbridge.js
  // Switch to main.html
  header("Location: ../main.html");
?>
