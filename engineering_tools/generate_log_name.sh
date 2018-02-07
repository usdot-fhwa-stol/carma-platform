#!/bin/bash
cd /opt/carma
rm -rf configuration/
mkdir configuration
cd configuration
date +%Y%m%d-%H%M > logname.txt
