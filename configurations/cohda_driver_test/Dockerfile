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

FROM busybox:latest

ARG BUILD_DATE
ARG VCS_REF
ARG VERSION

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-config:cohda-driver-test"
LABEL org.label-schema.description="System configuration data for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version="2.8.3"
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/CARMAPlatform"
LABEL org.label-schema.vcs-ref=${VCS_REF}}
LABEL org.label-schema.build-date=${BUILD_DATE}

ADD . /root/vehicle
VOLUME /opt/carma/vehicle

CMD  cp /root/vehicle/* /opt/carma/vehicle
