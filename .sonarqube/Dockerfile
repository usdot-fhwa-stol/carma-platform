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

# Use carma-base image to pull in ros dependancies
FROM usdotfhwastol/carma-base:2.8.3


# Install SonarQube Binaries

ARG SONAR_SCANNER_ZIP="/Sonar-Scanner/sonar-scanner-cli-3.3.0.1492-linux.zip"
ARG SONAR_WRAPPER_ZIP="/Sonar-Scanner/build-wrapper-linux-x86.zip"
ARG PROPERTIES_FILE=sonar-scanner.properties
#RUN wget https://sonarsource.bintray.com/Distribution/sonarqube/sonarqube-$SONARQUBE_VERSION.zip && \
#        apt-get -y install unzip && \
#        sudo unzip sonarqube-$SONARQUBE_VERSION.zip -d /opt && \
#        sudo mv /opt/sonarqube-$SONARQUBE_VERSION /opt/sonarqube
ENV SONAR_DIR=/opt/sonarqube

COPY $SONAR_SCANNER_ZIP $SONAR_DIR/
COPY $SONAR_WRAPPER_ZIP $SONAR_DIR/

#Try pulling from internet
RUN cd $SONAR_DIR && \
        sudo apt-get -y install unzip && \
        sudo unzip $SONAR_DIR/sonar-scanner-cli-3.3.0.1492-linux.zip -d . && \
        sudo mkdir $SONAR_DIR/sonar-scanner && \
        cd $(ls $SONAR_DIR | grep "sonar-scanner-" | grep -v zip) && \
        sudo mv * -t $SONAR_DIR/sonar-scanner && \
        cd $SONAR_DIR && \
        sudo unzip $SONAR_DIR/build-wrapper-linux-x86.zip -d . && \
        sudo mkdir $SONAR_DIR/build-wrapper && \
        cd $(ls $SONAR_DIR | grep "build-wrapper-" | grep -v zip) && \
        sudo mv * -t $SONAR_DIR/build-wrapper && \
       # Add scanner and wrapper to PATH
        export PATH=$PATH:$SONAR_DIR/sonar-scanner/bin/:$SONAR_DIR/sonar-wrapper/ 
        # TODO need to set export in /home/../init-env.
        
RUN sudo cp -r $SONAR_DIR /opt/sonar_backup
# Copy in sonar.properties file
COPY $PROPERTIES_FILE $SONAR_DIR/sonar-scanner/conf/sonar-scanner.properties
