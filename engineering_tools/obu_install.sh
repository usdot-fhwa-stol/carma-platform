#!/bin/bash

#  Copyright (C) 2018-2020 LEIDOS.
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

# EXAMPLE: 192.168.0.4 (0xc0.0xa8.0x00.0x04) and port 5398
# Pacifica and Ford: 192.168.88.10 (0xc0.0xa8.0x58.0x0A) and port 5398:
# Lexus: 192.168.0.100 (0xc0.0xa8.0x00.0x64) and port 5398:
# Silver truck: 192.168.20.100 (0xc0.0xa8.0x14.0x64) and port 5398:

# Step 8b:
# Copy the following 7 lines together
# Lexus: iso.0.15628.4.1.7.1.3.1 x 0x000000000000000000000000c0a80064 \
whoami

set -x

while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -p|--pacifica)
                HEX_IP=0x000000000000000000000000c0a80004
                shift
            ;;
            -ford|--ford)
                HEX_IP=0x000000000000000000000000c0a80004
                shift
            ;;
            -l|--lexus)
                HEX_IP=0x000000000000000000000000c0a80064
                shift
            ;;
            -st|--silver-truck)
                HEX_IP=0x000000000000000000000000c0a81464
                shift
      esac
done

/opt/cohda/application/rc.local stop

sleep 8

net-snmp-config --create-snmpv3-user -A rsuadmin -X rsuadmin -a SHA -x AES rsu

/opt/cohda/application/rc.local start

sleep 8

snmpset -v3 -lauthPriv -ursu -Arsuadmin -Xrsuadmin -aSHA -xAES -mRSU-MIB -M/mnt/rw/rsu1609/snmp/mibs -O T 127.0.0.1 iso.0.15628.4.1.99.0 i 2

sleep 8

#Enable Logging
snmpset -v3 -lauthPriv -ursu -Arsuadmin -Xrsuadmin -aSHA -xAES -mRSU-MIB -M/mnt/rw/rsu1609/snmp/mibs -O T 127.0.0.1 \
1.0.15628.4.1.9.1.2.2 i 1 \
1.0.15628.4.1.9.1.2.3 i 1 \
1.0.15628.4.1.9.1.2.4 i 1 \
1.0.15628.4.1.9.1.2.5 i 1 \
iso.0.15628.4.1.99.0 i 4

#Forward BSMs (0x20) to IP
snmpset -v3 -lauthPriv -ursu -Arsuadmin -Xrsuadmin -aSHA -xAES -mRSU-MIB -M/mnt/rw/rsu1609/snmp/mibs -O T 127.0.0.1 \
iso.0.15628.4.1.7.1.11.1 i 4 \
iso.0.15628.4.1.7.1.2.1 x 0x20 \
iso.0.15628.4.1.7.1.3.1 x $HEX_IP \
iso.0.15628.4.1.7.1.4.1 i 2020 \
iso.0.15628.4.1.7.1.5.1 i 2 \
iso.0.15628.4.1.7.1.10.1 i 1 \
iso.0.15628.4.1.99.0 i 4



