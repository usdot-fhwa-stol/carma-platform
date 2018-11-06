FROM ros:kinetic-ros-base AS source-code
RUN apt-get update
RUN apt-get install -y git

ARG SSH_PRIVATE_KEY
ARG EXTRA_PACKAGES
ENV EXTRA_PACKAGES ${EXTRA_PACKAGES}

RUN mkdir /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# Acquire the software source
ARG carma_version=develop
RUN \
        mkdir src && \
        cd src && \
        git clone https://github.com/usdot-fhwa-stol/CARMAPlatform.git --branch ${carma_version} && \
        cd .. && \
        ./src/CARMAPlatform/docker/checkout.sh

FROM ros:kinetic-ros-base AS install

# Install necessary packages
RUN apt-get update
RUN apt-get install -y ros-kinetic-rosjava ros-kinetic-rosbridge-server

RUN rosdep update

RUN mkdir -p ~/carma_ws/src
COPY --from=source-code /root/src/. ~/carma_ws/src

# Copy the built files into their installed location
RUN ~/carma_ws/src/CARMAPlatform/docker/install.sh

FROM ros:kinetic-ros-base

# Add carma user
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

# Install necessary packages
RUN apt-get update
RUN apt-get install -y sudo ros-kinetic-rosjava ros-kinetic-rosbridge-server tmux apache2 php7.0 libapache2-mod-php7.0 vim less

USER carma

COPY --from=install --chown=carma /opt/carma /opt/
COPY --from=install --chown=www-data /var/www/html/. /var/www/html
COPY --from=install --chown=carma /home/carma/.bashrc /home/carma/.bashrc

EXPOSE 80

CMD [ "/opt/carma/entrypoint.sh" ]
