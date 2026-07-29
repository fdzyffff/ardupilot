mkdir multi_sitl_test
cd multi_sitl_test
mkdir copter_233
cd copter_233

../../build/sitl/bin/arducopter --model + --serial0 tcp:5770 --serial1 tcpclient:127.0.0.1:5761 --serial2 tcp:127.0.0.1:5788 --instance 2 --defaults ../../extra.parm,../../Tools/autotest/default_params/copter.parm --home -35.3627273,149.1651642,583.0,0