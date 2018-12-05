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
# Stage 1 - Install the SSH private key and acquire the CARMA source as well as
#           any extra packages
# /////////////////////////////////////////////////////////////////////////////
FROM carma-base AS source-code

ARG SSH_PRIVATE_KEY
ARG EXTRA_PACKAGES
ENV EXTRA_PACKAGES ${EXTRA_PACKAGES}
ARG EXTRA_PACKAGES_VERSION=master
ENV EXTRA_PACKAGES_VERSION ${EXTRA_PACKAGES_VERSION}

# Set up the SSH key for usage by git
RUN mkdir ~/.ssh/ && \
        echo "${SSH_PRIVATE_KEY}" > ~/.ssh/id_rsa && \
        chmod 600 ~/.ssh/id_rsa && \
        touch ~/.ssh/known_hosts && \
        ssh-keyscan github.com >> ~/.ssh/known_hosts

# Acquire the software source
RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/CARMAPlatform/
RUN ~/src/CARMAPlatform/docker/checkout.sh

# /////////////////////////////////////////////////////////////////////////////
# Stage 2 - Build and install the software 
# /////////////////////////////////////////////////////////////////////////////
FROM carma-base AS install

# Copy the source files from the previous stage and build/install
RUN mkdir ~/carma_ws
COPY --from=source-code --chown=carma /home/carma/src /home/carma/carma_ws/src
RUN ~/carma_ws/src/CARMAPlatform/docker/install.sh

# /////////////////////////////////////////////////////////////////////////////
# Stage 3 - Finalize deployment
# /////////////////////////////////////////////////////////////////////////////
FROM carma-base

# Migrate the files from the install stage
COPY --from=install --chown=carma /opt/carma /opt/carma
COPY --from=install --chown=carma /root/.bashrc /home/carma/.bashrc

ADD carmajava/launch/* /opt/carma/vehicle/

RUN sudo chown carma:carma -R /opt/carma/vehicle && \
        ln -sf /opt/carma/vehicle/saxton_cav.urdf /opt/carma/urdf/saxton_cav.urdf && \
        ln -sf /opt/carma/vehicle/saxton_cav.launch /opt/carma/launch/saxton_cav.launch && \
        ln -sf /opt/carma/vehicle/drivers.launch /opt/carma/drivers/drivers.launch 

ENTRYPOINT [ "/opt/carma/entrypoint.sh" ]
