#! /bin/bash

# --------------------------------------------------------------
# Installs the mem_update.sh script to run automatically by cron
# 
# Ensure this script has execute permissions, e.g. 
#	chmod 755 mem_update_install.sh
# then run the script:
#	./mem_update_install.sh
# It will do the rest.  Next time the computer boots, the
# rmem_max value will be set to the desired value.
# If you want to set it immediately, without a reboot, run the
# generated script:
#	sudo <homedir>/carma_mem_update.sh
# Results will appear in the <homedir>/mem_update_log after
# each invocation.
# --------------------------------------------------------------

HOME=/home/dev
FILE=/tmp/carma-crontab-old
SCRIPT=$HOME/carma_mem_update.sh
LOG=$HOME/mem_update_log

# append the old crontab with the new command
# if a carma command was already there, replace it
#must have a CR at end of previous line or cron will ignore it
sudo /usr/bin/crontab -u root -l | grep -v carma > $FILE 2> /dev/null
echo "@reboot $SCRIPT >> $LOG" >> $FILE
echo " " >> $FILE

# replace the crontab with the new file
sudo /usr/bin/crontab -u root $FILE

echo "Root crontab updated to run the memory update upon reboot."

# Create the script that cron will run in user's home dir
echo "#! /bin/bash" > $SCRIPT
echo " " >> $SCRIPT
echo "# Configures memory to handle rosbridge load until Carma is completely migrated to ROS2" >> $SCRIPT
echo "# CAUTION: when removing this script, ensure that root's crontab has the line referring to it removed" >> $SCRIPT
echo "#          by using sudo crontab -e" >> $SCRIPT
echo "#" >> $SCRIPT
echo "date >> $LOG" >> $SCRIPT
echo "/usr/sbin/sysctl -w net.core.rmem_max=2147483647 2>> $LOG" >> $SCRIPT
echo "/usr/sbin/sysctl -a | grep rmem_max >> $LOG" >> $SCRIPT
echo "echo '-----' >> $LOG" >> $SCRIPT
chmod 755 $SCRIPT

# Ensure there is a log file that is writable
touch $LOG
chmod 664 $LOG

echo "Script created as $SCRIPT for cron to run"
