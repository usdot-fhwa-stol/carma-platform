#!/bin/bash
# This script configures OBU to support BSM messages and Mobility messages
# Make sure the OBU is in START mode by run: /opt/cohda/application/rc.local restart
# After running this script, make sure to restart OBU by run: /opt/cohda/application/rc.local start
# The above commands are not in this script because remote calling those two functions results in OBU application shutdown for some reasons

# Set IP address and username of target OBU
# Password for this username is: rsuadmin
USERNAME=rsu
OBU=192.168.88.40

# Set IPv6 address of host vehicle PC (no colons)
HOST=FE800000000000009AB08974D16F34D8

# Set effective date and expiration date (YYYYMMDDHHMM in hex)
STARTDATE=07e10a010000
ENDDATE=07e20a010000

# SSH into OBU and execute commands
ssh -o StrictHostKeyChecking=no ${USERNAME}@${OBU} << EOF
sudo /opt/cohda/application/rc.local standby
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.1 x 0020
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.1 x ${HOST}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.1 x 1516
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.1 x ${STARTDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.1 x ${ENDDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.2 x 00FF
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.2 x ${HOST}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.2 x 1516
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.2 x ${STARTDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.2 x ${ENDDATE}
EOF
