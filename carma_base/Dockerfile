#  Copyright (C) 2018-2019 LEIDOS.
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

# CARMA Base Image Docker Configuration Script

FROM ros:kinetic-ros-base 

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-base"
LABEL org.label-schema.description="Base operating system install for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/CARMAPlatform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

RUN apt-get update && \
        apt-get install -y \
        git \
        ssh \
        ros-kinetic-rosjava \
        ros-kinetic-rosbridge-server \
        sudo \
        tmux \
        vim \
        nano \
        less \
        apt-transport-https

RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
        apt-get update && \
        apt-get install -y ros-kinetic-astuff-sensor-msgs

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

RUN mkdir -p /opt/carma && chown carma:carma -R /opt/carma

USER carma

ADD --chown=carma package.xml /home/carma/.base-image/workspace/src/carma_base/
ADD --chown=carma entrypoint.sh /home/carma/.base-image/
ADD --chown=carma init-env.sh /home/carma/.base-image/
RUN rosdep update && \
        rosdep install --from-paths ~/.base-image/workspace/src --ignore-src -y

RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it &&\
    sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin 

# Final system setup
RUN mkdir -p /opt/carma/app/bin /opt/carma/params /opt/carma/routes /opt/carma/urdf /opt/carma/logs /opt/carma/launch /opt/carma/app/mock_data /opt/carma/app/engineering_tools /opt/carma/drivers &&\
    echo "source ~/.base-image/init-env.sh" >> ~/.bashrc &&\
    echo "cd /opt/carma" >> ~/.bashrc 

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
