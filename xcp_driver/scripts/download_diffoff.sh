#!/bin/bash

SCRIPT_DIR="$( cd -P "$( dirname "$BASH_SOURCE[0]" )" >/dev/null 2>&1 && pwd )"
cd $SCRIPT_DIR/../
ROOT_DIR=$PWD

#echo "start building ..."
#$SCRIPT_DIR/build.sh
#echo "start running ..."
#
#$SCRIPT_DIR/can_activate.sh
#sleep 1
cd $ROOT_DIR/install/
#./xcp_driver_node --mode download --input ../json/example.json --output test.json

# GLOG_logtostderr=1 GLOG_stderrthreshold=0 ./xcp_driver_node --protocol ccp --mode download --input ../json/download.json --output /dev/shm/out.json --diff off
./xcp_driver_node --protocol ccp --mode download --input ../json/download.json --output /dev/shm/out.json --diff off
#./xcp_driver_node --mode download --input ../json/zero.json --output test.json

