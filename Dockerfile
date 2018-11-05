FROM ros:kinetic-ros-base AS ros

RUN apt-get update
RUN apt-get install -y sudo

# Add basic user
ENV USERNAME carma
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/$USERNAME/.bashrc

RUN apt-get install -y ros-kinetic-rosjava git
#RUN /bin/bash -c "mkdir -p ~/rosjava/src;wstool init -j4 ~/rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/kinetic/rosjava.rosinstall;source /opt/ros/kinetic/setup.bash;cd ~/rosjava;rosdep update;rosdep install --from-paths src -i -y;catkin_make"

USER carma

# Build the software
RUN cd && \
        mkdir -p carma_ws/src/CARMAPlatform carma_ws/install carma_ws/build carma_ws/devel && \
        cd carma_ws/src

RUN rosdep update

RUN cd ~/carma_ws/src/ && git clone https://github.com/usdot-fhwa-stol/CARMAPlatform.git
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd ~/carma_ws; rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y"
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd ~/carma_ws; catkin_make install"

RUN sudo mkdir -p /opt/carma/bin /opt/carma/params /opt/carma/routes /opt/carma/urdf /opt/carma/logs /opt/carma/launch /opt/carma/mock_drivers/src/test/data /opt/carma/engineering_tools
RUN sudo chown -R carma:carma /opt/carma
RUN cp install/* /opt/carma/bin/ && \
        cp src/CARMAPlatform/carmajava/route/src/test/resources/routes/*.yaml /opt/carma/routes && \
        cp src/CARMAPlatform/carmajava/launch/params /opt/carma/params && \
        cp src/CARMAPlatform/carmajava/launch/*.launch /opt/carma/launch && \
        cp src/CARMAPlatform/engineering_tools /opt/carma/engineering_tools && \
        cp src/CARMAPlatform/website /var/www/html && 

RUN echo "source /opt/carma/bin/setup.bash" >> /home/$USERNAME/.bashrc

RUN sudo apt-get -y install tmux

ENTRYPOINT [ "tmux new-session -- roslaunch carma saxton_cav.launch" ]
