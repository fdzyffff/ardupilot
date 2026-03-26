mkdir multi_sitl_test
cd multi_sitl_test
mkdir copter_1
cd copter_1

../../build/sitl/bin/arducopter --model + --serial0 tcp:5760 --serial1 tcp:5761 --serial4 tcp:5766 --instance 0 --defaults ../../Tools/autotest/default_params/copter.parm --home 39.9784849,116.3394964,50.0,0
