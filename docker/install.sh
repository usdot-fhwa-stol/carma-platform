#!/bin/bash

# Build the software and its dependencies
source /opt/ros/kinetic/setup.bash
cd ~/carma_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make install

mkdir -p /opt/carma/app/bin /opt/carma/params /opt/carma/routes /opt/carma/urdf /opt/carma/logs /opt/carma/launch /opt/carma/app/mock_data /opt/carma/app/engineering_tools

# Copy the installed files
cd ~/carma_ws 
cp -r install/. /opt/carma/app/bin/ 
chmod -R +x /opt/carma/app/bin 
cp -r src/CARMAPlatform/carmajava/route/src/test/resources/routes/*.yaml /opt/carma/routes
cp -r src/CARMAPlatform/carmajava/launch/params/* /opt/carma/params/
cp -r src/CARMAPlatform/carmajava/launch/*.launch /opt/carma/launch/
ln -s  /opt/carma/launch/* /opt/carma/app/bin/share/carma
cp -r src/CARMAPlatform/engineering_tools/* /opt/carma/app/engineering_tools/
cp -r src/CARMAPlatform/engineering_tools /opt/carma/app/bin/share
cp -r src/CARMAPlatform/carmajava/mock_drivers/src/test/data/. /opt/carma/app/mock_data
cp src/CARMAPlatform/entrypoint.sh /opt/carma
mkdir -p /var/www/html
cp -r ~/carma_ws/src/CARMAPlatform/website/* /var/www/html

# Setup the user login configuration
echo "source /opt/ros/kinetic/setup.bash" >> /home/$USERNAME/.bashrc
echo "source /opt/carma/app/bin/setup.bash" >> /home/$USERNAME/.bashrc
echo "cd /opt/carma" >> /home/$USERNAME/.bashrc