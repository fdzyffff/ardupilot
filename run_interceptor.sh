#!/usr/bin/env bash

mkdir -p multi_sitl_test/interceptor
cd multi_sitl_test/interceptor || exit 1

../../Tools/autotest/run_in_terminal_window.sh ArduCopter ../../build/sitl/bin/arducopter -S --model + --slave 0 --serial0 tcp:5760 --serial1 udpclient:127.0.0.1:14550 --serial2 tcp:5762 --serial4 tcp:5766 --instance 0 --defaults ../../Tools/autotest/default_params/copter.parm --home -35.3627273,149.1651642,583.0,0
