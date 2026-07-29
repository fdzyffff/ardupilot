mkdir multi_sitl_test
cd multi_sitl_test
mkdir plane_1
cd plane_1

../../Tools/autotest/run_in_terminal_window.sh ArduPlane ../../build/sitl/bin/arduplane -S --model quadplane-copter_tailsitter --slave 0 --serial0 tcp:5760 --serial1 tcp:5761 --serial2 tcp:5762 --serial4 tcp:5766 --instance 0 --defaults ../../Tools/autotest/default_params/quadplane.parm,../../Tools/autotest/default_params/quadplane-copter_tailsitter.parm --home -35.3625273,149.1653642,583.0,0
# ../../build/sitl/bin/arduplane --model quadplane-copter_tailsitter --serial0 tcp:5760 --serial1 tcp:5761 --serial4 mcast:226.0.0.22:7071 --instance 0 --defaults ../../Tools/autotest/default_params/quadplane.parm,../../Tools/autotest/default_params/quadplane-copter_tailsitter.parm --home -35.3625273,149.1653642,583.0,0