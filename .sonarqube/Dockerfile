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
ARG PROPERTIES_FILE=sonar.properties
ARG SONARQUBE_VERSION=7.6
# Use carma-base image to pull in ros dependancies
FROM usdotfhwastol:carma-base:2.8.2

# Install SonarQube Binaries
RUN wget https://sonarsource.bintray.com/Distribution/sonarqube/sonarqube-$SONARQUBE_VERSION.zip && \
        apt-get -y install unzip && \
        sudo unzip sonarqube-$SONARQUBE_VERSION.zip -d /opt && \
        sudo mv /opt/sonarqube-$SONARQUBE_VERSION /opt/sonarqube

# Copy in sonar.properties file
COPY $PROPERTIES_FILE /opt/sonarqube/conf/sonar.properties
        

