#!/bin/bash
cd /opt/carma
rm -f logname.txt
date +%Y%m%d-%H%M%S > logname.txt
chmod -R 666 logname.txt
