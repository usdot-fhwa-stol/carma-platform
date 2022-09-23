#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y linux-aws && sudo apt upgrade -y
sudo apt-get update && DEBIAN_FRONTEND=noninteractive sudo apt-get -y -o Dpkg::Options::=''--force-confdef'' -o Dpkg::Options::=''--force-confold'' dist-upgrade -y
sudo apt-get install -y python3 python3-dev python3-pip
pip3 install --upgrade awscli
pip3 install --upgrade pip boto3 requests
sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
sudo apt-get install -y build-essential
sudo bash -c "sudo fallocate -l 512MB /var/swapfile && sudo chmod 600 /var/swapfile && sudo mkswap /var/swapfile && sudo echo '/var/swapfile swap swap defaults 0 0' >> /etc/fstab"
sudo sed -i 's|//Unattended-Upgrade::InstallOnShutdown "true";|Unattended-Upgrade::InstallOnShutdown "true";|' /etc/apt/apt.conf.d/50unattended-upgrades
DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends ubuntu-desktop gdm3
DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends firefox xterm
sudo sed -i 's/^#  AutomaticLogin/AutomaticLogin/' /etc/gdm3/custom.conf
sudo sed -i 's/user1/ubuntu/' /etc/gdm3/custom.conf
sudo su -l ubuntu -c "dbus-launch gsettings set org.gnome.desktop.screensaver idle-activation-enabled 'false'"
sudo su -l ubuntu -c "dbus-launch gsettings set org.gnome.desktop.screensaver lock-enabled 'false'"
sudo su -l ubuntu -c "dbus-launch gsettings set org.gnome.desktop.lockdown disable-lock-screen 'true'"
sudo su -l ubuntu -c "dbus-launch gsettings set org.gnome.desktop.session idle-delay 0"
sudo apt-get install -y python python-dev
chown -R ubuntu:ubuntu /home/ubuntu/.local
sudo rm /var/lib/update-notifier/updates-available
echo Installing DCV for Ubuntu 20.04
wget https://d1uj6qtbmh3dt5.cloudfront.net/2022.1/Servers/nice-dcv-2022.1-13300-ubuntu2004-x86_64.tgz
tar xvfz nice-dcv-2022.1-13300-ubuntu2004-x86_64.tgz
cd nice-dcv-2022.1-13300-ubuntu2004-x86_64
DEBIAN_FRONTEND=noninteractive sudo apt-get install -y
sudo apt --fix-broken install
sudo apt install -y ./nice-dcv-server_2022.1.13300-1_amd64.ubuntu2004.deb
sudo apt install -y ./nice-xdcv_2022.1.433-1_amd64.ubuntu2004.deb
sudo apt install -y ./nice-dcv-web-viewer_2022.1.13300-1_amd64.ubuntu2004.deb

sudo usermod -aG video dcv

cat << 'EOF' > ./dcv.conf
[license]
[log]
[display]
[connectivity]
web-port=8080
web-use-https=false
[security]
authentication="none"
EOF

sudo mv ./dcv.conf /etc/dcv/dcv.conf
sudo /usr/bin/dcvusbdriverinstaller --quiet
sudo su -l ubuntu -c dbus-launch gsettings set org.gnome.shell enabled-extensions "['ubuntu-dock@ubuntu.com']"
#sudo /sbin/iptables -A INPUT -p tcp ! -s localhost --dport 8080 -j DROP
sudo systemctl start dcvserver
sudo systemctl enable dcvserver

sudo bash -c 'cat << "EOF" > /etc/systemd/system/dcvsession.service
          [Unit]
          Description=NICE DCV Session
          After=dcvserver.service

          [Service]
          User=ubuntu
          ExecStart=/usr/bin/dcv create-session cloud9-session --owner ubuntu

          [Install]
          WantedBy=multi-user.target
EOF'

sudo systemctl daemon-reload
sudo systemctl enable dcvsession
sudo systemctl start dcvsession

echo Installing ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop
echo "[[ -e /opt/ros/noetic/setup.bash ]] && source /opt/ros/noetic/setup.bash" >> /home/ubuntu/.bashrc

echo Installing ROS2 Foxy
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-foxy-desktop

sudo apt-get install -y docker-compose
sleep 10
sudo systemctl start docker

cd ~

sudo apt-get update
sudo apt-get install -y lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool
echo "source /usr/share/vcstool-completion/vcs.bash" >> ~/.bashrc

sudo rm /usr/bin/docker-compose
DESTINATION=/usr/bin/docker-compose
sudo curl -L "https://github.com/docker/compose/releases/download/1.29.1/docker-compose-$(uname -s)-$(uname -m)" -o $DESTINATION
sudo chmod 755 $DESTINATION

sudo apt install -y chromium-browser

(
echo "parse_git_branch() {"
echo "     git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'"
echo "}"
echo "#Add git branch and timestamp to bash commands"
echo export PS1='"\t \u@\h \[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] $ "'
) >> ~/.bashrc

git clone https://github.com/usdot-fhwa-stol/carma-config.git

sudo curl -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/engineering_tools/opt_carma_setup.bash > ~/opt_carma_setup.bash
sudo bash ~/opt_carma_setup.bash ~/carma-config/example_calibration_folder/vehicle/
rm ~/opt_carma_setup.bash

sudo curl -o /usr/bin/carma -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/engineering_tools/carma
sudo chmod ugo+x /usr/bin/carma
sudo curl -o /etc/bash_completion.d/__carma_autocomplete -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/engineering_tools/__carma_autocomplete
sudo chmod ugo+x /etc/bash_completion.d/__carma_autocomplete

docker pull usdotfhwastol/carma-xil-cosimulation:1.0.0-beta
docker pull usdotfhwastol/carma-carla-integration:carma-carla-1.0

carma config install usdotfhwastoldev/carma-config:aws-test-carla-integration
carma config set usdotfhwastoldev/carma-config:aws-test-carla-integration

cd /home/ubuntu
mkdir carma_ws
cd carma_ws
mkdir src
cd src

# TODO make branches a variable
git clone https://github.com/usdot-fhwa-stol/carma-carla-integration.git --branch main
git clone https://github.com/usdot-fhwa-stol/carma-simulation.git --branch update_co_sim


cd /home/ubuntu
