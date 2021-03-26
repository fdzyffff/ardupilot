#!/bin/bash

# assume we start the script from the root directory
ROOTDIR=$PWD
COPTER=$ROOTDIR/../build/sitl/bin/arducopter

GCS_IP=$1

BASE_DEFAULTS="$ROOTDIR/../Tools/autotest/default_params/copter.parm"

# [ -x "$COPTER" ] || {
#     ./waf configure --board sitl
#     ./waf copter
# }

echo "----cleaning all previous sitl----"
ps -ef|grep sitl|grep -v grep|awk '{print $2}' |xargs kill -9

# start up main rover in the current directory
$COPTER --model copter --uartC tcp:0 --uartA mcast:127.0.0.1:14555 --defaults $BASE_DEFAULTS &

# now start another copter to follow the first, using
# a separate directory to keep the eeprom.bin and logs separate
# for increasing the number of copters, change the number in seq
for i in $(seq 1); do
    echo "Starting copter $i"
    mkdir -p copter$i

    SYSID=$(expr $i + 1)
    FOLL_SYSID=$(expr $SYSID - 1)
    FOLL_OFX_X=$(expr $i + 5)

    # create default parameter file for the follower
    cat <<EOF > copter$i/follow.parm
SYSID_THISMAV $SYSID
FOLL_ENABLE 1
FOLL_OFS_X -$FOLL_OFX_X
FOLL_OFS_TYPE 1
FOLL_SYSID $FOLL_SYSID
FOLL_DIST_MAX 1000
EOF
    pushd copter$i
    $COPTER --model copter --uartC udpclient:127.0.0.1:14550 --uartA mcast:127.0.0.1:14555 --instance $i --defaults $BASE_DEFAULTS,follow.parm &
    popd
done
wait
