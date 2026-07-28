#!/usr/bin/env bash

mkdir -p multi_sitl_test/target_233
cd multi_sitl_test/target_233 || exit 1

../../build/sitl/bin/arducopter --model + --serial0 tcp:5770 --serial1 tcpclient:127.0.0.1:5761 --instance 2 --defaults ../../target_233.parm,../../Tools/autotest/default_params/copter.parm --home -35.3625273,149.1653642,583.0,0
