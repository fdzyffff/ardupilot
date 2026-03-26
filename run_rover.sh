mkdir multi_sitl_test
cd multi_sitl_test
mkdir rover_233
cd rover_233

../../build/sitl/bin/ardurover --model rover --serial0 tcp:5770 --serial1 tcpclient:127.0.0.1:5761 --serial2 tcp:127.0.0.1:5788 --instance 2 --defaults ../../extra.parm,../../Tools/autotest/default_params/rover.parm --home 39.9784849,116.3394964,50.0,0