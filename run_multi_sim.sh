echo "------------- Kill all arducopter ------------- "
killall arducopter
killall ardurover
mkdir multi_sim

echo "------------- Run compiler -------------"
# ./waf copter
# ./waf rover

echo "------------- Run Copter1 -------------"
mkdir multi_sim/Copter1
cd multi_sim/Copter1
../../build/sitl/bin/arducopter --model + --uartA udpclient:127.0.0.1:14555 --serial1 udpclient:127.0.0.1:14550 --serial2 mcast:226.0.0.22:6666 --instance 0 --defaults ../../multi_sim_param/default_params/copter.parm,../../multi_sim_param/copter1.parm  --home 39.9788959,116.3397217,353.0,353 >copter1.out&
cd ../../


echo "------------- Run Copter2 -------------"
mkdir multi_sim/Copter2
cd multi_sim/Copter2
../../build/sitl/bin/arducopter --model + --uartA udpclient:127.0.0.1:14555 --serial1 udpclient:127.0.0.1:14550 --serial2 mcast:226.0.0.22:6666 --instance 1 --defaults ../../multi_sim_param/default_params/copter.parm,../../multi_sim_param/copter2.parm  --home 39.9788507,116.3398303,353.0,353 >copter2.out&
cd ../../


echo "------------- Run Copter3 -------------"
mkdir multi_sim/Copter3
cd multi_sim/Copter3
../../build/sitl/bin/arducopter --model + --uartA udpclient:127.0.0.1:14555 --serial1 udpclient:127.0.0.1:14550 --serial2 mcast:226.0.0.22:6666 --instance 1 --defaults ../../multi_sim_param/default_params/copter.parm,../../multi_sim_param/copter3.parm  --home 39.9789268,116.3399074,353.0,353 >copter3.out&
cd ../../


echo "------------- Run Copter4 -------------"
mkdir multi_sim/Copter4
cd multi_sim/Copter4
../../build/sitl/bin/arducopter --model + --uartA udpclient:127.0.0.1:14555 --serial1 udpclient:127.0.0.1:14550 --serial2 mcast:226.0.0.22:6666 --instance 1 --defaults ../../multi_sim_param/default_params/copter.parm,../../multi_sim_param/copter4.parm  --home 39.9787983,116.3398089,353.0,353 >copter4.out&
cd ../../


echo "------------- Run Rover10 -------------"
mkdir multi_sim/Rover10
cd multi_sim/Rover10
../../build/sitl/bin/ardurover --model rover --uartA udpclient:127.0.0.1:14555 --serial1 udpclient:127.0.0.1:14550 --serial2 mcast:226.0.0.22:6666 --instance 1 --defaults ../../multi_sim_param/default_params/rover.parm,../../multi_sim_param/rover10.parm  --home 39.9789535,116.3398015,353.0,353 >Rover10.out&
cd ../../


echo ""------------- Run Mavproxy "-------------"

# mavproxy.py --out 127.0.0.1:14550 --master udp:127.0.0.1:14555
mavproxy.py  --master udp:127.0.0.1:14555