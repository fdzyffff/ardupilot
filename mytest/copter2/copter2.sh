#!/bin/bash

# assume we start the script from the root directory

ROOTDIR=$PWD/../../
COPTER=$ROOTDIR/build/sitl/bin/arducopter

# if [ ! -d $COPTER ]; then
# 	echo ""
# # setup for either TCP or multicast
#UARTA="tcp:0"
#UARTA="mcast:"
#UARTA="udpclient:127.0.0.1:14550"
UARTA="tcp:0"
UARTC="tcpclient:127.0.0.1:5762"
#UARTC="tcp:5772"

COPTER_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm"

echo "----runing first copter #2----"

cat <<EOF > p_copter2.parm
SYSID_THISMAV 2
WP_YAW_BEHAVIOR 1
SIM_SONAR_SCALE 10
RNGFND1_TYPE 1
RNGFND1_SCALING 10
RNGFND1_PIN 0
RNGFND1_MAX_CM 5000
RNGFND1_MIN_CM 50
EOF
$COPTER --model copter --uartA $UARTA --uartC $UARTC --instance 1 --defaults $COPTER_DEFAULTS,p_copter2.parm &


mavproxy.py --master=tcp:127.0.0.1:5770 --out=udp:127.0.0.1:14551
