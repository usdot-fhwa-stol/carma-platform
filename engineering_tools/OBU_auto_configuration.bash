#!/bin/bash
# This script configures a Cohda OBU to support BSM messages and Mobility messages
# Make sure the OBU is in START mode by running this commands in OBU: /opt/cohda/application/rc.local restart
# After running this script, make sure to restart OBU by running this commands in OBU: /opt/cohda/application/rc.local start
# The above commands are not in this script because remote calling those two functions results in OBU application shutdown for some reasons

# Set IP address and username of target OBU
# Password for this username is: rsuadmin
USERNAME=rsu
OBU=192.168.0.41

# Set IPv6 address of host vehicle PC (Do not change it unless PC's IPv6 address is changed)
HOST=FE800000000000000260E0FFFE561C17

# Set effective date and expiration date (YYYYMMDDHHMM in hex)
# For example: 2017 10 01 00 00
#                 |  |  |  |  |
#              07e1 0a 01 00 00
STARTDATE=07e10a010000
ENDDATE=07e20a010000

# SSH into OBU and execute commands
# See "OBU Installation Manual.docx" for explanations
ssh -o StrictHostKeyChecking=no ${USERNAME}@${OBU} << EOF
sudo /opt/cohda/application/rc.local standby
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.1 x 0020
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.1 x ${HOST}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.1 x 1516
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.1 x ${STARTDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.1 x ${ENDDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.2 x BFEE
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.2 x ${HOST}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.2 x 1516
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.2 x ${STARTDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.2 x ${ENDDATE}
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.3 x 0000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.3 x 00000000000000000000000000000000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.3 x 0000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.3 x FFFFFFFFFFFF
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.3 x FFFFFFFFFFFF
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.4 x 0000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.4 x 00000000000000000000000000000000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.4 x 0000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.4 x FFFFFFFFFFFF
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.4 x FFFFFFFFFFFF
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.5 x 0000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.5 x 00000000000000000000000000000000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.5 x 0000
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.5 x FFFFFFFFFFFF
sudo snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.5 x FFFFFFFFFFFF
EOF
