mkdir multi_sitl_test
cd multi_sitl_test
mkdir plane_th8
cd plane_th8

../../Tools/autotest/run_in_terminal_window.sh ArduPlane ../../build/sitl/bin/arduplane -S --model plane --serial0 tcp:5770 --serial1 tcpclient:127.0.0.1:5761 --serial2 tcp:127.0.0.1:5788 --instance 2  --instance 0 --defaults ../../Tools/autotest/models/plane.parm --home 39.9784849,116.3394964,50.0,0
# ../../build/sitl/bin/arduplane --model quadplane-copter_tailsitter --serial0 tcp:5760 --serial1 tcp:5761 --serial4 mcast:226.0.0.22:7071 --instance 0 --defaults ../../Tools/autotest/default_params/quadplane.parm,../../Tools/autotest/default_params/quadplane-copter_tailsitter.parm --home 39.9789617,116.3399792,50.0,0