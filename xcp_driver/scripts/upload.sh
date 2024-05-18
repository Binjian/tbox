#!/bin/bash

SCRIPT_DIR="$( cd -P "$( dirname "$BASH_SOURCE[0]" )" >/dev/null 2>&1 && pwd )"
cd $SCRIPT_DIR/../
ROOT_DIR=$PWD

#echo "start building ..."
#$SCRIPT_DIR/build.sh
#echo "start running ..."

#$SCRIPT_DIR/can_activate.sh
#sleep 1
cd $ROOT_DIR/install/
GLOG_logtostderr=1 GLOG_stderrthreshold=0 ./xcp_driver_node --protocol ccp --mode upload --input ../json/download.json --output ../json/out.json --diff on
#./xcp_driver_node --protocol ccp --mode upload --input ../json/download.json --output ../json/upload.json --diff on

