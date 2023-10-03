#!/bin/bash

#  Copyright (C) 2023 LEIDOS.
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

# Function to display help message
display_help() {
        cat <<HELP
-------------------------------------------------------------------------------
| USDOT FHWA STOL CARMA                                              |
-------------------------------------------------------------------------------
Usage: $0 {init|stop|help}
Please enter one of the following commands when using the configure-cloud-sim extension:
        init     - Create a VPC with internet gateway and route table"
        stop     - Delete the created VPC, internet gateway, and route table"
        help     - Display this help message"
HELP
}

# Function to create the VPC, internet gateway, and route table
create_resources() {

    if [ ! command -v "aws" &> /dev/null ]; then
        echo "aws cli is not installed. Attempting to install. This requires sudo."
        
        curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
        unzip awscliv2.zip
        sudo ./aws/install

        echo "NOTE: You need to manually configure the aws cli for your user access key. Refer to this video for instructions https://www.youtube.com/watch?v=Rp-A84oh4G8"
        exit 1
    fi
    
    # Try deleting XiL Cloud resources if any exist
    echo "Checking and deleting existing XiL-Cloud resources first"
    delete_resources

    vpc_success=0
    igw_success=0
    route_table_success=0

    # Set your desired CIDR block here
    CIDR_Block="10.0.0.0/16"

    # Create VPC
    vpc_id=$(aws ec2 create-vpc --cidr-block $CIDR_Block --tag-specifications 'ResourceType=vpc,Tags=[{Key=Name,Value=XiL-Cloud VPC}]' \
    --query 'Vpc.VpcId' --output text) 

    # Check the exit code to determine if the VPC creation was successful
    if [ $? -eq 0 ]; then
        vpc_success=1
        echo "Created vpc: $vpc_id"
    else
        echo "Error creating XiL-Cloud VPC: $vpc_id"
    fi

    # Create Internet Gateway
    igw_id=$(aws ec2 create-internet-gateway \
    --tag-specifications 'ResourceType=internet-gateway,Tags=[{Key=Name,Value=XiL-Cloud IGW}]' --query 'InternetGateway.InternetGatewayId' \
    --output text) 

    # Check the exit code to determine if the Internet Gateway creation was successful
    if [ $? -eq 0 ]; then
        igw_success=1
        echo "Created Internet Gateway: $igw_id"
    else
        echo "Error creating Internet Gateway: $igw_id"
    fi

    # Attach Internet Gateway to VPC
    aws ec2 attach-internet-gateway --internet-gateway-id $igw_id --vpc-id $vpc_id

    # Create Route Table
    route_table_id=$(aws ec2 create-route-table --vpc-id $vpc_id  \
    --tag-specifications 'ResourceType=route-table,Tags=[{Key=Name,Value=XiL-Cloud RouteTable}]' --query 'RouteTable.RouteTableId' \
    --output text)

    # Check the exit code to determine if the Route Table creation was successful
    if [ $? -eq 0 ]; then
        route_table_success=1
        echo "Created route table: $route_table_id"
    else
        echo "Error creating Route Table: $route_table_id"
    fi

    # Create a default route to the Internet Gateway
    aws ec2 create-route --route-table-id $route_table_id --destination-cidr-block 0.0.0.0/0 --gateway-id $igw_id &> /dev/null

    if [ $? -eq 0 ]; then
        echo "Default route created successfully!"
    else
        echo "Error creating default route to Internet Gateway."
    fi


    if [ $vpc_success -eq 1 ] && [ $igw_success -eq 1 ] && [ $route_table_success -eq 1 ]; then
        echo "XiL-Cloud required resources were created successfully!"
    else
        echo "Some resources encountered errors. See above for details."
    fi

}

# Function to delete the VPC, internet gateway, and route table based on tag name
delete_resources() {

    if [ ! command -v "aws" &> /dev/null ]; then
        echo "aws cli is not installed. Attempting to install. This requires sudo."
        
        curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
        unzip awscliv2.zip
        sudo ./aws/install

        echo "NOTE: You need to manually configure the aws cli for your user access key. Refer to this video for instructions https://www.youtube.com/watch?v=Rp-A84oh4G8"
        exit 1
    fi

    vpc_success=0
    igw_success=0
    route_table_success=0

    # Get VPC ID based on tag name
    vpc_id=$(aws ec2 describe-vpcs --filters "Name=tag:Name,Values=XiL-Cloud VPC" --query 'Vpcs[0].VpcId' --output text)

    # Get Internet Gateway ID based on tag name
    igw_id=$(aws ec2 describe-internet-gateways --filters "Name=tag:Name,Values=XiL-Cloud IGW" \
    --query 'InternetGateways[0].InternetGatewayId' --output text)


    # Get Route Table ID based on tag name
    route_table_id=$(aws ec2 describe-route-tables --filters "Name=tag:Name,Values=XiL-Cloud RouteTable" \
    --query 'RouteTables[0].RouteTableId' --output text)

    if [ -z "$route_table_id" ] || [ $route_table_id != "None" ]; then
        # Delete the route table
        aws ec2 delete-route-table --route-table-id $route_table_id
        route_table_success=1
        echo "Deleted Route Table: $route_table_id"
    fi

    if [ -z "$igw_id" ] || [ $igw_id != "None" ]; then
        # Detach and delete the Internet Gateway
        aws ec2 detach-internet-gateway --internet-gateway-id $igw_id --vpc-id $vpc_id
        aws ec2 delete-internet-gateway --internet-gateway-id $igw_id
        igw_success=1
        echo "Deleted Internet Gateway: $igw_id"
    fi

    if [ -z "$vpc_id" ] || [ $vpc_id != "None" ]; then
        # Delete the VPC
        aws ec2 delete-vpc --vpc-id $vpc_id
        vpc_success=1
        echo "Deleted VPC: $vpc_id"
    fi

    if [ $vpc_success -eq 1 ] && [ $igw_success -eq 1 ] && [ $route_table_success -eq 1 ]; then
        echo "XiL-Cloud VPC, Internet Gateway and Route Table were deleted successfully!"
    else
        echo "No resources required to be deleted"
    fi

}

configure-cloud-sim__help() {
    cat <<HELP
-------------------------------------------------------------------------------
| USDOT FHWA STOL CARMA                                              |
-------------------------------------------------------------------------------

Please enter one of the following commands when using the configure-cloud-sim extension:
        init     - Create a VPC with internet gateway and route table"
        stop     - Delete the created VPC, internet gateway, and route table"
        help     - Display this help message"
HELP
}


# Check for the argument and perform the corresponding action
case "$1" in
    init)
        create_resources
        ;;
    stop)
        delete_resources
        ;;
    help)
        display_help
        ;;
    *)
        echo "Invalid argument."
        display_help
        exit 1
        ;;
esac