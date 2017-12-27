#!/bin/bash
# This script configures OBU to support BSM messages and Mobility messages

# Set IP address and username of target OBU
# Password for this username is: rsuadmin
USERNAME=rsu
OBU=192.168.88.40

# Set IPv6 address of host vehicle PC (no colons)
HOST=26070000000000000260e0fffe561c17

# Set effective date and expiration date (YYYYMMDDHHMM in hex)
STARTDATE=07e10a010000
ENDDATE=07e20a010000

# SSH into OBU and execute commands
ssh -o StrictHostKeyChecking=no -t -l ${USERNAME} ${OBU} bash -c "'
sudo -i;
/opt/cphda/application/rc.local stop;
/opt/cphda/application/rc.local start;
/opt/cphda/application/rc.local standby;
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.1 x 0020;
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.1 x ${HOST};
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.1 x 1516;
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.1 x ${STARTDATE};
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.1 x ${ENDDATE};
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.2.1 x 00FF;
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.3.1 x ${HOST};
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.4.1 x 1516;
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.8.1 x ${STARTDATE};
snmpset -v 3 -l authPriv -u password -A password -X password -a SHA -x AES ${OBU} 1.0.15628.4.1.7.1.9.1 x ${ENDDATE};
/opt/cohda/application/rc.local start;
'"
