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

# CARMA Base CI Image Docker Configuration Script

# Use carma-base image to pull in ros dependancies
# This image is meant to be build with carma_base/build-image.sh
# The latest tag is used to grab the carma_base image build directly before this one
FROM usdotfhwastol/carma-base:latest

ARG BUILD_DATE="NULL"
ARG VCS_REF="NULL"
ARG VERSION="NULL"

# Set environment variable for SonarQube Binaries
# Two binaries are will go in this repo. 
# The Build Wrapper which executes a code build to capture C++
# The Sonar Scanner which evaluates both C++ and Java then uploads the results to SonarCloud
ENV SONAR_DIR=/opt/sonarqube

# Pull scanner from internet
RUN sudo apt-get -y install curl && \
        sudo mkdir $SONAR_DIR && \
        sudo curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-3.3.0.1492-linux.zip && \
        sudo curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip && \
        # Install Dependancy of NodeJs 6+
        sudo curl -sL https://deb.nodesource.com/setup_10.x | sudo bash - && \
        sudo sudo apt-get install -y nodejs && \
        # Install JQ Json Parser Tool
        sudo mkdir /opt/jq && \
        sudo curl -L "https://github.com/stedolan/jq/releases/download/jq-1.5/jq-linux64" -o /opt/jq/jq && \
        sudo chmod +x /opt/jq/jq


# Unzip scanner
RUN cd $SONAR_DIR && \ 
        sudo apt-get -y install unzip && \
        sudo unzip $SONAR_DIR/sonar-scanner.zip -d . && \
        sudo unzip $SONAR_DIR/build-wrapper.zip -d . && \
        # Remove zip files 
        sudo rm $SONAR_DIR/sonar-scanner.zip && \
        sudo rm $SONAR_DIR/build-wrapper.zip && \
        # Rename files 
        sudo mv $(ls $SONAR_DIR | grep "sonar-scanner-") $SONAR_DIR/sonar-scanner/ && \
        sudo mv $(ls $SONAR_DIR | grep "build-wrapper-") $SONAR_DIR/build-wrapper/ && \
        # Add scanner, wrapper, and jq to PATH
        sudo echo 'export PATH=$PATH:/opt/jq/:$SONAR_DIR/sonar-scanner/bin/:$SONAR_DIR/build-wrapper/' >> /home/carma/.base-image/init-env.sh

# Install gcovr for code coverage tests and add code_coverage script folder to path
RUN sudo apt-get -y install gcovr && \
        sudo echo 'export PATH=$PATH:/home/carma/.ci-image/engineering_tools/code_coverage/' >> /home/carma/.base-image/init-env.sh

# Add engineering tools scripts to image
ADD --chown=carma ./engineering_tools/code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

        

# Set metadata labels
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="CARMA-SonarCloud"
LABEL org.label-schema.description="Base image for CARMA CI testing using SonarCloud"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version="${VERSION}"
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/CARMAPlatform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Setup environment on login
ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
