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
ENV SONAR_DIR=/opt/sonarqube

# Pull scanner from internet
RUN sudo apt-get -y install curl && \
        sudo mkdir $SONAR_DIR && \
        sudo curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-3.3.0.1492-linux.zip && \
        sudo curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip && \
        # Install Dependancy of NodeJs 6+
        sudo curl -sL https://deb.nodesource.com/setup_10.x | sudo bash - && \
        sudo sudo apt-get install -y nodejs

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
        # Add scanner and wrapper to PATH
        sudo echo PATH=$PATH:$SONAR_DIR/sonar-scanner/bin/:$SONAR_DIR/build-wrapper/ >> /home/carma/.base-image/init-env.sh

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
