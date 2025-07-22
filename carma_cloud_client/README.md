# carma_cloud_client
CARMA Cloud Client is a ROS2 node in CARMA platform which is responsible for cellular communication with CARMA Cloud.

https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2365030405/CARMA+Cloud+Client


### Troubleshooting Tips
When running CARMA Platform with `carma start all`, the SSH tunnels can occasionally fail to open. This issue is tracked under [CDAD-184 (an internal Jira task)](https://usdot-carma.atlassian.net/browse/CDAD-184).
As a workaround, opening the tunnels can be achieved by manually running the launch/scripts/call.sh script. 

In order to run the script manually, please follow the steps listed below:

1. Update the configuration parameter in launch/scripts/call.sh to point to the cloud instance running carma-cloud.
2. Obtain the appropriate .pem file and copy it into the launch directory.
3. Navigate to the scripts directory: `cd <path to carma_cloud_client>/launch/scripts/`
3. Make call.sh and open_tunnels.sh executable: `sudo chmod u+x <script_name>`
4. Run the call script: `./call.sh`

