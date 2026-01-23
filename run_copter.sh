mkdir multi_sitl_test
cd multi_sitl_test
mkdir copter_233
cd copter_233

../../build/sitl/bin/arducopter --model + --serial0 tcp:5770 --serial1 tcpclient:127.0.0.1:5766 --instance 2 --defaults ../../extra.parm,../../Tools/autotest/default_params/copter.parm --home 39.9784849,116.3394964,50.0,0