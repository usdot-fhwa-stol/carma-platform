#  Copyright (C) 2018 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# CARMA Docker Configuration Script
#
# Performs all necessary tasks related to generation of a generic CARMA docker image
# suitable for deployment to a vehicle. The generic image will still need to be 
# configured for each vehicle by means of volume mapping configuration files and
# networking mapping at run time
#
# Build Arguments:
# SSH_PRIVATE_KEY - If the extra package repository to be used during the build requires
#                   authentication, please pass in the necessary SSH private key in text
#                   form here, most likely via "$(cat ~/.ssh/id_rsa)". This data is not 
#                   present in the final output image. Default = none
#
# EXTRA_PACKAGES - The repo to checkout any additional packages from at build time. 
#                  Default = none


# /////////////////////////////////////////////////////////////////////////////
# Stage 1 - Install the SSH private key and acquire the CARMA source as well as any
#           extra packages
# /////////////////////////////////////////////////////////////////////////////
FROM ros:kinetic-ros-base AS source-code
RUN apt-get update
RUN apt-get install -y git ssh

ARG SSH_PRIVATE_KEY
ARG EXTRA_PACKAGES
ENV EXTRA_PACKAGES ${EXTRA_PACKAGES}
ARG EXTRA_PACKAGES_VERSION=master
ENV EXTRA_PACKAGES_VERSION ${EXTRA_PACKAGES_VERSION}

# Set up the SSH key for usage by git
RUN mkdir /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa
RUN chmod 600 /root/.ssh/id_rsa
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# Acquire the software source
COPY . /root/src/CARMAPlatform/
RUN /root/src/CARMAPlatform/docker/checkout.sh


# /////////////////////////////////////////////////////////////////////////////
# Stage 2 - Build and install the software 
# /////////////////////////////////////////////////////////////////////////////
FROM ros:kinetic-ros-base AS install

# Install necessary packages
RUN apt-get update
RUN apt-get install -y ros-kinetic-rosjava ros-kinetic-rosbridge-server
RUN rosdep update

# Copy the source files from the previous stage and build/install
COPY --from=source-code /root/src /root/carma_ws/src
RUN ~/carma_ws/src/CARMAPlatform/docker/install.sh

# /////////////////////////////////////////////////////////////////////////////
# Stage 3 - Finalize deployment
# /////////////////////////////////////////////////////////////////////////////
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
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

# Install necessary packages
RUN apt-get update
RUN apt-get install -y sudo ros-kinetic-rosjava ros-kinetic-rosbridge-server tmux apache2 php7.0 libapache2-mod-php7.0 vim less

USER carma

# Migrate the files from the install stage
COPY --from=install --chown=carma /opt/carma /opt/carma
COPY --from=install --chown=www-data /var/www/html/. /var/www/html
COPY --from=install --chown=carma /root/.bashrc /home/carma/.bashrc

EXPOSE 80

CMD [ "/opt/carma/entrypoint.sh" ]
