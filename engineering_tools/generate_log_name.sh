#!/bin/bash
cd /opt/carma
rm -rf configuration/
mkdir configuration
chmod 777 configuration
cd configuration
date +%Y%m%d-%H%M%S > logname.txt
chmod 777 logname.txt
