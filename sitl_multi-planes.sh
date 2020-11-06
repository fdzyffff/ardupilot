#!/bin/bash

# assume we start the script from the root directory

ROOTDIR=$PWD
PLANE=$ROOTDIR/build/sitl/bin/arduplane

# if [ ! -d $PLANE ]; then
# 	echo ""
# # setup for either TCP or multicast
#UARTA="tcp:0"
#UARTA="mcast:"
UARTA="udpclient:127.0.0.1:14550"

PLANE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/plane.parm"

echo "----cleaning all previous sitl----"
ps -ef|grep sitl|grep -v grep|awk '{print $2}' |xargs kill -9
#rm -r swarm


echo "----runing first plane #0----"

mkdir -p swarm/plane0
cd swarm/plane0
$PLANE --model plane --uartA $UARTA --uartD tcp:5764 --uartF tcp:5765 --uartE tcp:5766 --defaults $PLANE_DEFAULTS &

cd ../../

sleep 3
echo "----clone plane #1----"
# create default parameter file for the second plane

mkdir -p swarm/plane1
cd swarm/plane1
cat <<EOF > plane_multi1.parm
SYSID_THISMAV 2
EOF
$PLANE --model plane --uartA $UARTA --uartD tcp:5774 --uartF tcp:5775 --uartE tcp:5776 --instance 1 --defaults $PLANE_DEFAULTS,plane_multi1.parm &

cd ../../

sleep 3
echo "----clone plane #2----"
# create default parameter file for the second plane

mkdir -p swarm/plane2
cd swarm/plane2
cat <<EOF > plane_multi2.parm
SYSID_THISMAV 3
EOF
$PLANE --model plane --uartA $UARTA --uartD tcp:5784 --uartF tcp:5785 --uartE tcp:5786 --instance 2 --defaults $PLANE_DEFAULTS,plane_multi2.parm &

cd ../../
sleep 2
wait