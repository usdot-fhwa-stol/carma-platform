<?php
  // Kill the docker system
  shell_exec("/var/www/html/scripts/kill.bash");
  // Move to logout page
  header("Location: ../logout.html"); // Move onto main.html
?>