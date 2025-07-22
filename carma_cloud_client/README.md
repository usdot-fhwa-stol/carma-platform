# carma_cloud_client
CARMA Cloud Client is a ROS2 node in CARMA platform which is responsible for cellular communication with CARMA Cloud.

https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2365030405/CARMA+Cloud+Client


### Note: 
SSH tunnels opened through the launch process can intermittently fail. Issue tracked under https://usdot-carma.atlassian.net/browse/CDAD-184
Manually opening the tunnel through the launch/call.sh has been seen to be a viable workaround. 
In order to run the script manually
1. Update the configuration parameter in launch/scripts/call.sh to point to the cloud instance running carma-cloud.
2. Copy over the .pem file into launch directory.
3. Navigate to scripts directory `cd <path to carma_cloud_client>/launch/scripts/`
3. Make call.sh and open_tunnels.sh executable (`sudo chmod u+x <script_name>`)
4. Run the call script `./call.sh`
