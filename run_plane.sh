mkdir multi_sitl_test
cd multi_sitl_test
mkdir plane_1
cd plane_1

../../build/sitl/bin/arduplane --model plane --serial0 tcp:5760 --serial1 tcp:5766 --instance 0 --defaults ../../Tools/autotest/models/plane.parm --home 39.9789617,116.3399792,50.0,0
# ../../build/sitl/bin/arduplane --model quadplane-copter_tailsitter --serial0 tcp:5760 --serial1 tcp:5766 --instance 0 --defaults ../../Tools/autotest/default_params/quadplane.parm,../../Tools/autotest/default_params/quadplane-copter_tailsitter.parm --home 39.9789617,116.3399792,50.0,0