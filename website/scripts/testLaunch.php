<?php
  exec("bash /var/www/html/scripts/launch.bash");
  echo "Launching"
  sleep(8);
  $output = shell_exec("cat /var/www/html/scripts/pid.txt");
  exec("kill -INT " . $output);
?>