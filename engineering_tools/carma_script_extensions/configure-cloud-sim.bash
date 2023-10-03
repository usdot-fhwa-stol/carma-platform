#!/bin/bash

#  Copyright (C) 2022 LEIDOS.
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

CARMA_CLOUD_SIM_CONFIG_PATH="${CARMA_CLOUD_SIM_CONFIG_DIR}/config.txt"


configure-cloud-sim__init() {
    # Configure carma cloud sim by setting up the required resources

    # Extract config
    local USER_ARN=$(cat ${CARMA_CLOUD_SIM_CONFIG_PATH} | grep "USER_ARN=" | sed "s|USER_ARN=||")

    if [ -z "$USER_ARN" ]; then
        echo "Configuration does not include AWS User ARN. Please run: carma cloud-sim config -a <User ARN>"
    fi

    # Create VPC
    aws ec2 create-vpc --cidr-block <CIDR_Block> --tag-specifications 'ResourceType=vpc,Tags=[{Key=Name,Value=XiL-Cloud VPC}]'
    local VPC_ID=$(aws ec2 describe-vpcs --filters "Name=tag:Name,Values=XiL-Cloud VPC" --query 'Vpcs[].VpcId' --output text)
    echo $VPC_ID

}

configure-cloud-sim__help() {
    cat <<HELP
-------------------------------------------------------------------------------
| USDOT FHWA STOL CARMA                                              |
-------------------------------------------------------------------------------

Please enter one of the following commands when using the cloud-sim extension:
    carma
        
HELP
}


configure-cloud-sim() {
    local cmdname=$1; shift
    if type "configure-cloud-sim__$cmdname" >/dev/null 2>&1; then
        "configure-cloud-sim__$cmdname" "$@"
    else
        configure-cloud-sim_help
        exit 1
    fi
}